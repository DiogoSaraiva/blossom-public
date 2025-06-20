# scan_dxl.py
"""
Utilities to discover Dynamixel servos connected to the host
and to build, on-the-fly, a minimal configuration dictionary
for a 'Woody'-style robot (IDs 1-5).

Functions
---------
scan_all_ports()        -> Dict[str, List[int]]
build_config_dynamic()  -> Tuple[str, dict] | None
"""

from typing import List, Dict, Tuple
import time
from serial.tools import list_ports
from dynamixel_sdk import PortHandler, PacketHandler


PROTOCOL    = 2                        # Dynamixel protocol 2.0
ID_RANGE    = range(1, 21)             # IDs 1-20 will be pinged
BAUDRATES   = (1_000_000, 57_600)      # baud rates to test (high → low)
PING_DELAY  = 30                       # ms between pings
LABELS = ["tower_1", "tower_2", "tower_3", "base", "ears"]

def scan_all_ports() -> Dict[str, List[int]]:
    """
    Scan every available serial port (at each baud-rate in ``BAUDRATES``) and
    ping all IDs in ``ID_RANGE``.
    Ports that respond with at least one ID are returned.

    Returns
    -------
    Dict[str, List[int]]
        Keys are strings of the form ``"<port>@<baud>"``; the corresponding
        value is the list of IDs that replied on that port/baud pair.
    """
    detected: Dict[str, List[int]] = {}
    packet = PacketHandler(PROTOCOL)

    for port_name in (p.device for p in list_ports.comports()):
        for baud in BAUDRATES:
            port = PortHandler(port_name)
            if not port.openPort():          # port unavailable / permission denied
                continue

            port.setBaudRate(baud)
            ids: List[int] = []

            for dxl_id in ID_RANGE:
                _, comm_result, _ = packet.ping(port, dxl_id)
                if comm_result == 0:         # Tx/Rx success
                    ids.append(dxl_id)
                time.sleep(PING_DELAY / 1000)

            port.closePort()

            if ids:                          # at least one servo answered
                detected[f"{port_name}@{baud}"] = ids
                break                        # do not retry other baud-rates

    return detected

def build_config_dynamic(scan: dict[str, list[int]]
                         ) -> dict[str, dict] | None:
    """
    Build a minimal *runtime* configuration for a ‘Woody’ robot (IDs 1-5).

    Parameters
    ----------
    scan : Dict[str, List[int]]
        The dictionary returned by :func:`scan_all_ports`.

    Returns
    -------
    Tuple[str, dict] | None
        ``(port@baud, cfg_dict)`` if the pattern ``[1,2,3,4,5]`` is found,
        otherwise ``None``.
    """

    for port_id, ids in scan.items():
        if sorted(ids) == [1, 2, 3, 4, 5]:
            port, baud_str = port_id.split("@")
            baud = int(baud_str)

            motors_cfg = {
                label: {
                    "id": dxl_id,
                    "orientation": "direct",
                    "offset": 0.0,
                    "angle_limit": (-150, 150) if label != "ears" else (50, 130),
                    "type": "XL-320",
                }
                for label, dxl_id in zip(LABELS, ids)
            }

            full_cfg = {
                "controllers": {
                    "dxl": {
                        "port": port,
                        "baudrate": baud,
                        "protocol": 2,
                        "sync_read": False,
                        "attached_motors": list(motors_cfg.keys()),
                    }
                },
                "motorgroups": {
                    "tower": ["tower_1", "tower_2", "tower_3"],
                    "bases": ["base"],
                    "head": ["ears"],
                },
                "motors": motors_cfg,
            }

            return {port_id: full_cfg}
    return None
