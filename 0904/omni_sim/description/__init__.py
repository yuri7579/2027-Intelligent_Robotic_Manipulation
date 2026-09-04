"""
Robot description of the master device (Geomagic Touch / Phantom Omni).

    urdf/omni.urdf      kinematic description (6 revolute joints: m1 ~ m6)
    meshes/*.stl        visual meshes referenced by the URDF

The URDF refers to its meshes with ``package://omni_sim/description/meshes/...``.
We do **not** run ``colcon build`` in this class, so there is no ament index and
RViz cannot resolve ``package://`` by itself.  :func:`load_urdf` therefore
rewrites those URIs into absolute ``file://`` URIs before the description is
handed to robot_state_publisher / RViz.
"""

import os
from urllib.parse import quote

PKG_NAME = 'omni_sim'
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(PKG_DIR, 'description', 'urdf', 'omni.urdf')


def load_urdf(urdf_path=URDF_PATH):
    """Return the URDF as a string with every ``package://`` URI resolved.

    Args:
        urdf_path (str): path of the URDF file. Defaults to ``urdf/omni.urdf``.

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
