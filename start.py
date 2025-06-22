"""
Start up the blossom webserver, CLI client, and web client.
"""

from __future__ import annotations

import sys
from pathlib import Path
import subprocess
import argparse
import os
import shutil
import signal
from collections import OrderedDict
from typing import List, Optional

from src.scan_dxl import scan_all_ports, build_config_dynamic
from src.dxl_robot import DxlRobot
from src import robot, sequence
from src.server import server
from src import server as srvr
from src.sequence import SimpleSequencePlayer

import threading
import webbrowser
import re
from serial.serialutil import SerialException
import random
import time
import uuid
import requests
import logging
# seed time for better randomness
random.seed(time.time())


seq_thread: Optional[threading.Thread] = None
# turn off Flask logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# main robot
master_robot = None
# list of robots
robots = []
second_robot = None
# speed factor to playback sequences
speed = 1.0
# amplitude factor
amp = 1.0
# posture factor
post = 0.0

# yarn process (for web/UI stuff)
yarn_process = None

# CLI prompt
prompt = "(l)ist sequences / (s)equence play / (q)uit: "

loaded_seq = []

class SequenceRobot(DxlRobot):
    """
    Robot that loads, plays, and records sequences
    Extends Robot class
    """

    def __init__(self, name: str, cfg: dict):
        super().__init__(cfg)
        self.name = name
        self.config = cfg
        self.seq_list: OrderedDict[str, sequence.Sequence] = OrderedDict()
        # threads for playing and recording sequences
        self.seq_thread = self.seq_stop = None
        self.rec_thread = self.rec_stop = None
        # load all sequences for this robot
        self.load_seq()

        # speed, amp range from 0.5 to 1.5
        self.speed = 1.0
        self.amp = 1.0
        # posture ranges from -100 to 100
        self.post = 0.0

    def load_seq(self) -> None:
        """
        Scan ``src/sequences/<robot_name>`` (recursively) and register every
        ``*.json`` animation that is not yet in ``self.seq_list``.

        The path is computed *relative to this file*, so it works no matter
        where the process is launched from.
        """
        # ------------------------------------------------------------------ #
        # 1. Resolve directory (…/src/sequences/<robot_name>)                #
        # ------------------------------------------------------------------ #
        root_dir = Path(__file__).resolve().parent          # …/blossom_public/src
        seq_dir  = root_dir / "sequences" / self.name
        seq_dir.mkdir(parents=True, exist_ok=True)

        # ------------------------------------------------------------------ #
        # 2. Walk every subdirectory and load new *.json files              #
        # ------------------------------------------------------------------ #
        for json_path in sorted(seq_dir.rglob("*.json")):   # Path objects
            json_str = str(json_path)
            if json_str in loaded_seq:                      # already cached
                continue

            try:
                self.load_single_sequence(json_path)
                loaded_seq.append(str(json_path))
            except Exception as exc:
                print(f"[load_seq] Falhou a carregar {json_path}: {exc}")

    # ------------------------------------------------------------------
    def load_single_sequence(self, json_path: Path) -> None:
        """Load ONE gesture file and add it to seq_list."""
        seq_obj = sequence.Sequence.from_json(str(json_path), rad=True)
        seq_name = seq_obj.seq_name
        if seq_name in self.seq_list:
            print(f"[load_seq] '{seq_name}' already loaded – ignore.")
            return
        self.seq_list[seq_name] = seq_obj

    # ------------------------------------------------------------------
    def assign_time_length(self, keys, vals):
        return [
            (name, seq.frames[-1].millis / 1000.0)  # ms  ➔  s
            for name, seq in zip(keys, vals)
            if seq.frames  # ignore empty sequences
        ]

    def get_time_sequences(self):
        temp_keys = list(self.seq_list.keys())
        temp_vals = list(self.seq_list.values())
        temp_map = self.assign_time_length(temp_keys, temp_vals)
        return temp_map

    def get_sequences(self):
        """
        Get all sequences loaded on robot
        """
        return self.seq_list.keys()

    def play_seq_json(self, seq_json):
        """
        Play a sequence from json
        args:
            seq_json    sequence raw json
        returns:
            the thread setting motor position in the sequence
        """
        seq = sequence.Sequence.from_json_object(seq_json, rad=True)


        if self.seq_thread and self.seq_thread.is_alive():
            self.seq_thread.stop()

        # start playback thread
        self.seq_thread = SimpleSequencePlayer(
            self, seq, speed=self.speed, amp=self.amp, post=self.post, loop=False
        )
        self.seq_thread.start()

        # return thread
        return self.seq_thread

    def play_recording(self, seq_name: str, idler=False, speed=speed, amp=amp, post=post):
        """
        Play a recorded sequence
        args:
            seq     sequence name
            idler   whether to loop sequence or not
        returns:
            the thread setting motor position in the sequence
        """
        if self.seq_thread and self.seq_thread.is_alive():
            self.seq_thread.stop()

        # loop if idler
        if ('idle' in seq_name):
            seq_name = seq_name.replace('idle', '').replace(' ', '').replace('/', '')
            idler = True


        seq_obj = self.seq_list[seq_name]
        # start playback thread
        self.seq_thread = SimpleSequencePlayer(
            self, seq_obj,
            speed=self.speed, amp=self.amp, post=self.post,
            loop=idler
        )
        self.seq_thread.start()
        # return thread
        return self.seq_thread

    def start_recording(self):
        """
        Begin recording a sequence
        """
        # create stop flag object
        self.rec_stop = threading.Event()

        # start recording thread
        self.rec_thread = robot.sequence.RecorderPrimitive(self, self.rec_stop)
        self.rec_thread.start()



'''
CLI Code
'''


def start_cli(robot: SequenceRobot) -> None:
    t = threading.Thread(target=run_cli, args=[robot])
    t.daemon = True
    t.start()


def run_cli(bot: SequenceRobot) -> None:
    """
    Handle CLI inputs indefinitely
    """
    while True:
        cmd_str = input(prompt)
        cmd, *raw = re.split(r'[ /]', cmd_str)
        args = [x for x in raw if x] or None

        # call the dispatcher **on the robot we received**
        handle_input(bot, cmd, args)


def handle_quit():
    """
    Close the robot object and clean up any temporary files.
     Manually kill the flask server because there isn't an obvious way to do so gracefully.

    Raises:
        ???: Occurs when yarn failed to start but yarn_process was still set to true.
    """
    print("Exiting...")
    for bot in robots:
        # clean up tmp dirs and close robots
        tmp_dir = './src/sequences/%s/tmp' % bot.name
        if os.path.exists(tmp_dir):
            shutil.rmtree(tmp_dir)
        bot.robot.close()
    print("Bye!")
    # TODO: Figure out how to kill flask gracefully
    _terminate_yarn(yarn_process)


last_cmd,last_args = 'rand',[]
def handle_input(bot,
                 cmd: str,
                 args: Optional[List[str]] = None) -> None:
    if args is None:
        args = []
    """
    handle CLI input

    Args:
        bot: the robot affected by the given command
        cmd: a robot command
        args: additional args for the command
    """
    # manipulate the global speed and amplitude vars
    # global speed
    # global amp
    # global post
    # print(cmd, args)
    # separator between sequence and idler
    global last_cmd, last_args, seq_thread
    idle_sep = '='
    # play sequence
    if cmd in['s', 'rand']:
        # if random, choose random sequence
        if cmd == 'rand':
            args = [random.choice(list(bot.seq_list.keys()))]
        # default to not idling
        # idler = False
        # get sequence if not given
        if not args:
            args = ['']
            # args[0] = input('Sequence: ')
            seq = input('Sequence: ')
        else:
            seq = args[0]
        # check if should be idler
        # elif (args[0] == 'idle'):
        #     args[0] = args[1]
        #     idler = True
        idle_seq = ''
        if idle_sep in seq:
            (seq, idle_seq) = re.split(idle_sep + '| ', seq)

        # catch hardcoded idle sequences
        if seq == 'random':
            random.seed(time.time())
            seq = random.choice(['calm', 'slowlook', 'sideside'])
        if idle_seq == 'random':
            random.seed(time.time())
            idle_seq = random.choice(['calm', 'slowlook', 'sideside'])
        if seq in ['calm', 'slowlook', 'sideside']:
            idle_seq = seq

        # speed = 1.0
        # amp = 1.0
        # post = 0.0

        # if (len(args)>=2) : speed = args[1]
        # if (len(args)>=3) : amp = args[2]
        # if (len(args)>=4) : post = args[3]

        # play the sequence if it exists
        if seq in bot.seq_list:
            # print("Playing sequence: %s"%(args[0]))
            # iterate through all robots
            for bot in robots:
                if bot.seq_thread and bot.seq_thread.is_alive():
                    bot.seq_thread.stop()
                seq_thread = bot.play_recording(seq, idler=False)
            # go into idler
            if idle_seq != '':
                while seq_thread.is_alive():
                    # sleep necessary to smooth motion
                    time.sleep(0.1)
                    continue
                for bot in robots:
                    bot.play_recording(idle_seq, idler=True)
        # sequence not found
        else:
            print("Unknown sequence name:", seq)
            return

    # record sequence
    # elif cmd == 'r':
    #     record(robot)
    #     input("Press 'enter' to stop recording")
    #     stop_record(robot, input("Seq name: "))

    # reload gestures
    elif cmd == 'r':
        for bot in robots:
            bot.load_seq()

    # list and print sequences (only for the first attached robot)
    elif cmd in ['l', 'ls']:
        # remove asterisk (terminal holdover)
        if args:
            args[0] = args[0].replace('*','')
        for seq_name in bot.seq_list.keys():
            # skip if argument is not in the current sequence name
            if args and args[0]!=seq_name[:len(args[0])]:
                continue
            print(seq_name)

    # exit
    elif cmd == 'q':
        handle_quit()

    # debugging stuff
    # manually move
    elif cmd == 'm':
        # get motor and pos if not given
        if not args:
            args = ['', '']
            args[0] = input('Motor: ')
            args[1] = input('Position: ')
        for bot in robots:
            if args[0] == 'all':
                bot.goto_position({'tower_1': float(args[1]), 'tower_2': float(
                    args[1]), 'tower_3': float(args[1])}, 0, True)
            else:
                bot.goto_position({args[0]: float(args[1])}, 0, True)

    # adjust speed (0.5 to 2.0)
    elif cmd == 'e':
        for bot in robots:
            bot.speed = float(input('Speed factor: '))
    # adjust amplitude (0.5 to 2.0)
    elif cmd == 'a':
        for bot in robots:
            bot.amp = float(input('Amplitude factor: '))
    # adjust posture (-150 to 150)
    elif cmd == 'p':
        for bot in robots:
            bot.post = float(input('Posture factor: '))

    # help
    elif cmd == 'h':
        exec('help(' + input('Help: ') + ')')

    # manual control
    elif cmd == 'man':
        while True:
            try:
                exec(input('Command: '))
            except KeyboardInterrupt:
                break

    elif cmd=='':
        handle_input(bot,last_cmd,last_args)
        return
    # directly call a sequence (skip 's')
    elif cmd in bot.seq_list.keys():
        args=[cmd]
        cmd='s'
        handle_input(bot,cmd,args)
    # directly call a random sequence by partial name match
    elif [cmd in seq_name for seq_name in bot.seq_list.keys()]:
        # print(args[0])
        if 'mix' not in cmd:
            seq_list = [seq_name for seq_name in bot.seq_list.keys() if cmd in seq_name and 'mix' not in seq_name]
        else:
            seq_list = [seq_name for seq_name in bot.seq_list.keys() if cmd in seq_name]

        if len(seq_list)==0:
            print("No sequences matching name: %s" % cmd)
            return
        handle_input(bot,'s',[random.choice(seq_list)])
        cmd=cmd

    # elif cmd == 'c':
    #     robot.calibrate()

    else:
        print("Invalid input")
        return
    last_cmd,last_args=cmd,args

def record(bot):
    """
    Start new recording session on the robot
    """
    # stop recording if one is happening
    if not bot.rec_stop:
        bot.rec_stop = threading.Event()
    # start recording thread
    bot.rec_stop.set()
    bot.start_recording()


def stop_record(bot, seq_name=""):
    """
    Stop recording
    args:
        robot       the robot under which to save the sequence
        seq_name    the name of the sequence
    returns:
        the name of the saved sequence
    """
    # stop recording
    bot.rec_stop.set()

    # if provided, save sequence name
    if seq_name:
        seq = bot.rec_thread.save_rec(seq_name, robots=robots)
        store_gesture(seq_name, seq)
    # otherwise, give it random temporary name
    else:
        seq_name = uuid.uuid4().hex
        bot.rec_thread.save_rec(seq_name, robots=robots, tmp=True)

    # return name of saved sequence
    return seq_name


def store_gesture(name, seq, label=""):
    """
    Save a gesture sequence to the GCP datastore via HTTP POST.

    Args:
        name (str): The name of the sequence.
        seq (dict): The sequence data to be stored.
        label (str): An optional label associated with the sequence.
    """

    url = "https://classification-service-dot-blossom-gestures.appspot.com/gesture"
    payload = {
        "name": name,
        "sequence": seq,
        "label": label,
    }
    requests.post(url, json=payload)


'''
Main Code
'''


def start_server(host, port, hide_browser):
    """
    Initialize the blossom webserver and open the web client.

    Args:
        host: the hostname of the server socket
        port: the port of the server socket
        hide_browser: does not open the web client if set to true
    """
    global yarn_process
    print(" ")
    import prettytable as pt

    sentence = "%s:%d" % (host, port)
    width = 26

    t = pt.PrettyTable()

    t.field_names = ['IP ADDRESS']
    [t.add_row([sentence[i:i + width]]) for i in range(0, len(sentence), width)]

    print(t)

    #start_yarn()
    if not hide_browser:

        addr = "%s:%d" % (host, port)
        webbrowser.open("http:"+addr, new=2)

    print("\nExample command: s -> *enter* -> yes -> *enter*")
    server.set_funcs(master_robot, robots, handle_input,
                     record, stop_record, store_gesture)
    server.start_server(host, port)


def start_yarn():
    """
    Run `yarn dev` to start the React app. The process id is saved to a global variable so it can be killed later.
    """
    global yarn_process

    command = "yarn dev"
    yarn_process = subprocess.Popen(
        command, shell=True, cwd="./blossom_web", stdout=subprocess.PIPE, preexec_fn=os.setsid)


def main(args):
    """
    Start robots, start up server, handle CLI
    """
    # get robots to start
    global master_robot, robots

    configs = build_config_dynamic()

    if configs is None:
        print("No robot found")
        sys.exit(1)

    master_port, master_cfg = next(iter(configs.items()))
    master_robot = safe_init_robot(master_port, master_cfg)

    # start robots
    robots = [master_robot]

    master_robot.reset_position()


    # start CLI
    start_cli(master_robot)
    # start server
    start_server(args.host, args.port, args.browser_disable)


def safe_init_robot(name, config):
    """
    Safely start and initialize a robot instance, retrying if motor startup fails.

    Args:
        name (str): The name of the robot to start.
        config (dict): The motor configuration of the robot.

    Returns:
        SequenceRobot: The initialized robot object.
    """
    # SequenceRobot
    bot = None
    # limit of number of attempts
    attempts = 10

    # keep trying until number of attempts reached
    while bot is None:
        try:
            bot = SequenceRobot(name, config)
        except (RuntimeError, NotImplementedError, SerialException) as e:
            if attempts <= 0:
                raise e
            print(e, "retrying...")
            attempts -= 1
    return bot


def _terminate_yarn(proc):
    if proc is None:
        return
    try:
        if os.name == "posix":
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        else:
            proc.terminate()
    except ProcessLookupError:
        print("yarn no longer exists.")
    except PermissionError as exc:
        print(f"No permission to terminate yarn: {exc}")

def parse_args(args):
    """
    Parse command-line arguments passed to the script.

    Args:
        args (list of str): The list of arguments from the terminal (typically sys.argv[1:]).

    Returns:
        argparse.Namespace: The parsed arguments as a namespace object.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument('--names', '-n', type=str, nargs='+',
                        help='Name of the robot.', default=["woody"])
    parser.add_argument('--port', '-p', type=int,
                        help='Port to start server on.', default=8000)
    parser.add_argument('--host', '-i', type=str, help='IP address of webserver',
                        default=srvr.get_ip_address())
    parser.add_argument('--browser-disable', '-b',
                        help='prevent a browser window from opening with the blossom UI',
                        action='store_true')
    parser.add_argument('--list-robots', '-l',
                        help='list all robot names', action='store_true')

    parser.add_argument('--simulate', '-s', help='Allows to run without a robot', action='store_true')
    return parser.parse_args(args)


"""
Generic main handler
"""
if __name__ == "__main__":
    main(parse_args(sys.argv[1:]))
