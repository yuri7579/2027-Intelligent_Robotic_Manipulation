"""
Robot description of the slave robot (ROBOTIS OMY-3M).

    urdf/omy_3m.urdf    kinematic description (6 revolute joints: joint1 ~ joint6)
    meshes/*.stl        visual meshes referenced by the URDF

Source: https://github.com/ROBOTIS-GIT/open_manipulator (jazzy branch,
open_manipulator_description).  The only change made to the original file is
the mesh URI prefix: ``package://open_manipulator_description/meshes/omy_3m/``
was rewritten to ``package://omy_sim/description/meshes/`` so that the model is
self-contained inside this folder.  :func:`load_urdf` then resolves those URIs
to absolute ``file://`` URIs, because we never run ``colcon build`` in this
class and RViz would not be able to look the package up otherwise.
"""

import os
from urllib.parse import quote

PKG_NAME = 'omy_sim'
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(PKG_DIR, 'description', 'urdf', 'omy_3m.urdf')


def load_urdf(urdf_path=URDF_PATH):
    """Return the URDF as a string with every ``package://`` URI resolved.

    Args:
        urdf_path (str): path of the URDF file. Defaults to ``urdf/omy_3m.urdf``.

    Returns:
        str: URDF xml, ready to be used as the ``robot_description`` parameter.
    """
    with open(urdf_path, 'r') as f:
        urdf = f.read()
    # quote() keeps a normal ASCII path untouched and percent-encodes spaces or
    # non-ASCII characters, which libcurl (used by resource_retriever) requires.
    file_uri = 'file://{}/'.format(quote(PKG_DIR))
    return urdf.replace('package://{}/'.format(PKG_NAME), file_uri)


__all__ = ['PKG_NAME', 'PKG_DIR', 'URDF_PATH', 'load_urdf']
