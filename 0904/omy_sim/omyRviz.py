#!/usr/bin/env python3
"""
omyRviz.py
==========
Launches the **slave** (OMY-3M) robot visualization in RViz.

Same wrapper as ``omni_sim/omniRviz.py``, but for the slave robot: it calls
``launch/omy_rviz.launch.py`` with ``ros2 launch``::

    slave_rviz = OmyRviz()          # no sliders - the teleop node commands it
    slave_rviz.start()
    ...
    slave_rviz.shutdown()

Standalone use (slave only, with sliders to check the model)::

    python3 omy_sim/omyRviz.py --gui
"""

import argparse
import os
import signal
import subprocess
import sys
import time

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'omy_rviz.launch.py')
DEFAULT_RVIZ_CONFIG = os.path.join(PKG_DIR, 'launch', 'omy.rviz')


class OmyRviz:
    """Owns the ``ros2 launch`` process of the slave visualization."""

    NAME = 'omy (slave)'

    def __init__(self, gui=False, rviz=True, rviz_config=DEFAULT_RVIZ_CONFIG,
                 launch_file=LAUNCH_FILE, quiet=False):
        """
        Args:
            gui (bool): start joint_state_publisher_gui.  Keep it False during
                teleoperation - the sliders would fight the teleop commands.
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
    parser = argparse.ArgumentParser(description='Launch the OMY (slave) RViz visualization.')
    parser.add_argument('--gui', action='store_true',
                        help='also start joint_state_publisher_gui (standalone test)')
    parser.add_argument('--no-rviz', action='store_true', help='do not start RViz')
    parser.add_argument('--rviz-config', default=DEFAULT_RVIZ_CONFIG)
    args = parser.parse_args(argv)

    viz = OmyRviz(gui=args.gui, rviz=not args.no_rviz, rviz_config=args.rviz_config)
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
