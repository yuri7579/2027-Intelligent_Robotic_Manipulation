#!/usr/bin/env python3
"""
omniRviz.py
===========
Launches the **master** (Omni) robot visualization in RViz.

It is a thin python wrapper that calls ``launch/omni_rviz.launch.py`` with
``ros2 launch``, so that a teleoperation script can bring the master window up
and take it down again without the user typing anything::

    master_rviz = OmniRviz(gui=True)
    master_rviz.start()
    ...
    master_rviz.shutdown()

Standalone use (master only, drag the sliders to move it)::

    python3 omni_sim/omniRviz.py
"""

import argparse
import os
import signal
import subprocess
import sys
import time

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'omni_rviz.launch.py')
DEFAULT_RVIZ_CONFIG = os.path.join(PKG_DIR, 'launch', 'omni.rviz')


class OmniRviz:
    """Owns the ``ros2 launch`` process of the master visualization."""

    NAME = 'omni (master)'

    def __init__(self, gui=True, rviz=True, rviz_config=DEFAULT_RVIZ_CONFIG,
                 launch_file=LAUNCH_FILE, quiet=False):
        """
        Args:
            gui (bool): start joint_state_publisher_gui (the slider window).
            rviz (bool): start the RViz window.
            rviz_config (str): RViz configuration file.
            launch_file (str): launch file to run.
            quiet (bool): silence the launch output.
        """
        self.gui = gui
        self.rviz = rviz
        self.rviz_config = rviz_config
        self.launch_file = launch_file
        self.quiet = quiet
        self._proc = None

    # ------------------------------------------------------------------ #
    @property
    def command(self):
        """The ``ros2 launch ...`` command line as a list."""
        return ['ros2', 'launch', self.launch_file,
                'gui:={}'.format(str(self.gui).lower()),
                'rviz:={}'.format(str(self.rviz).lower()),
                'rviz_config:={}'.format(self.rviz_config)]

    def start(self, wait=2.0):
        """Spawn the launch process.

        Args:
            wait (float): seconds to give the nodes to come up before returning.
        """
        if self.is_running():
            return self._proc

        out = subprocess.DEVNULL if self.quiet else None
        # start_new_session=True puts the launch process in its own process
        # group, so shutdown() can signal every node it started at once.
        self._proc = subprocess.Popen(self.command, stdout=out, stderr=out,
                                      start_new_session=True)
        print('[{}] launched: {}'.format(self.NAME, ' '.join(self.command)))
        if wait > 0:
            time.sleep(wait)
        return self._proc

    def is_running(self):
        """True while the launch process is alive."""
        return self._proc is not None and self._proc.poll() is None

    def shutdown(self, timeout=5.0):
        """Ask the launch process (and every node it started) to stop."""
        if not self.is_running():
            self._proc = None
            return
        try:
            # SIGINT is what ros2 launch expects for a clean shutdown.
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            self._proc.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        finally:
            print('[{}] shut down'.format(self.NAME))
            self._proc = None

    # ------------------------------------------------------------------ #
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.shutdown()
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description='Launch the Omni (master) RViz visualization.')
    parser.add_argument('--no-gui', action='store_true',
                        help='do not start joint_state_publisher_gui')
    parser.add_argument('--no-rviz', action='store_true', help='do not start RViz')
    parser.add_argument('--rviz-config', default=DEFAULT_RVIZ_CONFIG)
    args = parser.parse_args(argv)

    viz = OmniRviz(gui=not args.no_gui, rviz=not args.no_rviz,
                   rviz_config=args.rviz_config)
    viz.start(wait=0.0)
    try:
        viz._proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        viz.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
