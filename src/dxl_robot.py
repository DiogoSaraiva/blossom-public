"""
Dynamixel‑based robot wrapper that offers a *PyPot‑like* interface while
leveraging the **Robotis DynamixelSDK** directly.  The goal is to replace the
current PyPot dependency transparently: method names, signatures and value
ranges (°) stay identical so the rest of the Blossom code base can stay
untouched.

Only **XL‑320** servos using *protocol 2.0* are supported for now, but the
class is structured so other models / protocols can be added later.
"""
from __future__ import annotations

from typing import Dict, List, Callable
import math
import threading
import time

# DynamixelSDK — **do _not_ import PyPot here!**
from dynamixel_sdk import PacketHandler, PortHandler, COMM_SUCCESS

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

_DEG2RAW = 1023.0 / 300.0          # XL‑320: 300 ° span maps to 0–1023
_RAW2DEG = 1.0 / _DEG2RAW


def deg2raw(angle_deg: float) -> int:
    """Convert **degrees** to Dynamixel **raw position units** (XL‑320)."""
    return int(round(angle_deg * _DEG2RAW)) & 0x3FF  # 0‑1023, wrap‑around safe


def raw2deg(raw: int) -> float:
    """Convert Dynamixel **raw position units** to **degrees** (XL‑320)."""
    return raw * _RAW2DEG


# -----------------------------------------------------------------------------
# Core class
# -----------------------------------------------------------------------------

class DxlRobot:
    """Minimal Dynamixel robot with a **PyPot‑compatible API**.

    Parameters
    ----------
    cfg : dict
        Configuration dictionary returned by
        :pyfunc:`scan_dxl.build_config_dynamic`.  Keys of interest::

            cfg = {
                "controllers": {"ctrl": {"port": "COM3", "baudrate": 1_000_000}},
                "motors": {
                    "tower_1": {"id": 1, "orientation": "direct", "offset": 0.0},
                    ...
                }
            }
    """

    # --------------------- construction & teardown -------------------------- #

    def __init__(self, cfg: dict):
        self._cfg = cfg

        ctrl_name, ctrl_cfg = next(iter(cfg["controllers"].items()))
        port     = ctrl_cfg["port"]
        baudrate = ctrl_cfg.get("baudrate", 1_000_000)
        protocol = ctrl_cfg.get("protocol", 2)

        # Low‑level handlers -------------------------------------------------- #
        self._ph = PortHandler(port)
        if not self._ph.openPort():
            raise IOError(f"Cannot open port {port}")
        if not self._ph.setBaudRate(baudrate):
            raise IOError(f"Cannot set baudrate {baudrate} on {port}")
        self._pkt = PacketHandler(protocol)

        # Motor table --------------------------------------------------------- #
        self.motors: Dict[str, int] = {
            name: meta["id"] for name, meta in cfg["motors"].items()
        }
        self._orientation = {
            name: 1 if meta.get("orientation", "direct") == "direct" else -1
            for name, meta in cfg["motors"].items()
        }
        self._offset = {
            name: float(meta.get("offset", 0.0)) for name, meta in cfg["motors"].items()
        }

        # Compliance flag ----------------------------------------------------- #
        self._compliant = True
        self.set_compliant(False)  # make sure torque is on at start‑up

        # Default *rest* pose -------------------------------------------------- #
        self.reset_pos: Dict[str, float] = {
            "tower_1": 50.0,
            "tower_2": 50.0,
            "tower_3": 50.0,
            "base"   :  0.0,
            "ears"   :100.0,
        }
        self._believed_pose: Dict[str, float] = self.reset_pos.copy()
        self.reset_position()

        # Async command thread ------------------------------------------------ #
        self._queue: List[Callable[[], None]] = []
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

    # --------------------------- PyPot aliases ------------------------------ #

    def goto_position(self,
                      motor_pos: Dict[str, float],
                      delay: int = 200,
                      wait: bool = True) -> None:
        """PyPot‑style *wrapper* around :py:meth:`goto`.

        Parameters
        ----------
        motor_pos : dict
            ``{"motor_name": angle_deg, …}``
        delay : int, optional
            Motion duration **in milliseconds** (PyPot convention).
        wait : bool, optional
            If *False* the call returns immediately and the move is executed in
            a background thread – matching PyPot’s non‑blocking semantics.
        """
        seconds = max(0.0, delay / 1000.0)
        self.goto(motor_pos, seconds, wait)

    # 1‑line lambdas kept for legacy reflections ----------------------------- #
    get_motor_pos = lambda self: self.read_pose()

    # Property used by Blossom code base ------------------------------------- #
    @property
    def believed_motor_pos(self) -> Dict[str, float]:
        """Last pose the software *believes* the robot is in (deg)."""
        return self._believed_pose.copy()

    # --------------------------- public API --------------------------------- #

    def goto(self,
             pose: Dict[str, float],
             duration: float = 0.2,
             wait: bool = True) -> None:
        """Move one or more motors to the requested *pose* (degrees).

        The call is **blocking** unless *wait* is *False*, in which case the
        command is queued and executed asynchronously by the internal worker
        thread.  This preserves the optional *non‑blocking* behaviour of
        ``pypot.robot.goto_position``.
        """
        if not wait:
            # enqueue closure to be executed later --------------------------- #
            self._queue.append(lambda: self.goto(pose, duration, True))
            return

        # ----------------------------------------------------- actual motion #
        if self._compliant:
            self.set_compliant(False)

        for name, angle in pose.items():
            if name not in self.motors:
                continue  # unknown motor – ignore silently (PyPot behaviour)

            goal_deg = self._orientation[name] * angle + self._offset[name]
            goal_raw = deg2raw(goal_deg)
            dxl_id   = self.motors[name]

            # Conservative velocity profile so we never slam a joint ---------- #
            vel_raw = max(1, int(round(60 * duration)))  # very rough heuristic
            acc_raw = vel_raw  # proportional choice
            self._pkt.write2ByteTxRx(self._ph, dxl_id, 112,      acc_raw)  # Profile Accel
            self._pkt.write2ByteTxRx(self._ph, dxl_id, 112 + 2,  vel_raw)  # Profile Vel
            self._pkt.write2ByteTxRx(self._ph, dxl_id, 116,     goal_raw)  # Goal Position

        if wait:
            for name in pose.keys():
                if name not in self.motors:
                    continue
                self._wait_until_reached(self.motors[name], deg2raw(
                    self._orientation[name] * pose[name] + self._offset[name]))

        self._believed_pose.update(pose)

    # .....................................................................
    def read_pose(self) -> Dict[str, float]:
        """Return the *current* pose (degrees) as reported by the motors."""
        pose: Dict[str, float] = {}
        for name, dxl_id in self.motors.items():
            raw, result, _ = self._pkt.read2ByteTxRx(self._ph, dxl_id, 132)  # Present Pos
            if result == COMM_SUCCESS:
                pose[name] = (self._orientation[name] * raw2deg(raw)
                               - self._offset[name])
        self._believed_pose.update(pose)
        return pose

    # .....................................................................
    def set_compliant(self, compliant: bool = True) -> None:
        """Enable (=**True**) or disable (=**False**) torque on all joints."""
        for dxl_id in self.motors.values():
            self._pkt.write1ByteTxRx(self._ph, dxl_id, 64, 0 if compliant else 1)
        self._compliant = compliant

    # .....................................................................
    def reset_position(self) -> None:
        """Move the robot instantly back to the predefined *rest pose*."""
        self.goto(self.reset_pos, duration=0.0)

    # ---------------------- background worker loop ------------------------- #

    def _loop(self):
        """Worker that executes queued, non‑blocking *goto* calls sequentially."""
        while True:
            if self._queue:
                cb = self._queue.pop(0)
                try:
                    cb()
                except Exception as exc:
                    print("[DxlRobot] command failed:", exc)
            time.sleep(0.002)

    # -------------------------- internal utils ----------------------------- #

    def _wait_until_reached(self, dxl_id: int, goal_raw: int) -> None:
        """Busy‑wait until *Present Position* ≈ *Goal Position*."""
        while True:
            cur_raw, _, _ = self._pkt.read2ByteTxRx(self._ph, dxl_id, 132)
            if abs(cur_raw - goal_raw) <= 2:  # within 2 units ≈ 0.6 °
                break
            time.sleep(0.005)

    # --------------------------- destructor -------------------------------- #

    def close(self) -> None:
        """Disable torque on all joints and release the serial port."""
        self.set_compliant(True)
        self._ph.closePort()

    # Context‑manager sugar -------------------------------------------------- #
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
