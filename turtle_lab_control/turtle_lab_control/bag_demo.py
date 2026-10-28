"""Record a square or replay velocity commands with only turtlesim running."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def cli(*arguments: str) -> None:
    """Run a ROS CLI command and fail on any nonzero return code."""
    subprocess.run(['ros2', *arguments], check=True)


def main() -> None:
    """Record requires the lab; replay requires all lab nodes stopped."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['record', 'replay'])
    parser.add_argument('bag', type=Path)
    args = parser.parse_args()
    if args.mode == 'record':
        if args.bag.exists():
            parser.error('Output path already exists; choose a new bag path')
        recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(args.bag),
                                     '/turtle1/cmd_vel', '/turtle1/pose'])
        try:
            time.sleep(3.0)
            if recorder.poll() is not None:
                raise RuntimeError('Recorder exited before the square started')
            cli('action', 'send_goal', '/draw_square',
                'turtle_lab_interfaces/action/DrawSquare', '{side_length: 2.0}')
            time.sleep(1.0)
        finally:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=15)
    else:
        simulator = subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'],
                                     start_new_session=True)
        try:
            time.sleep(3.0)
            cli('service', 'call', '/turtle1/set_pen', 'turtlesim/srv/SetPen', '{"off": 1}')
            cli('service', 'call', '/turtle1/teleport_absolute',
                'turtlesim/srv/TeleportAbsolute', '{x: 5.544445, y: 5.544445, theta: 0.0}')
            cli('service', 'call', '/clear', 'std_srvs/srv/Empty', '{}')
            cli('service', 'call', '/turtle1/set_pen', 'turtlesim/srv/SetPen',
                '{r: 179, g: 184, b: 255, width: 3, "off": 0}')
            cli('bag', 'play', str(args.bag), '--topics', '/turtle1/cmd_vel')
            print('Replay finished. Inspect the canvas; press Ctrl+C to close turtlesim.', flush=True)
            simulator.wait()
        except KeyboardInterrupt:
            pass
        finally:
            if simulator.poll() is None:
                os.killpg(simulator.pid, signal.SIGINT)
            simulator.wait(timeout=15)
