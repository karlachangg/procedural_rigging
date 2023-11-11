"""
neck @ rig
"""

import maya.cmds as mc

from ..base import module
from ..base import control

def build(
        neckJoints,
        headJnt,
        neckCurve,
        prefix = 'neck',
        rigScale = 1.0,
        baseRig = None
        ):

    """
    :param neckJoints: list(str) list of neck joints
    :param headJnt: str, head joint at the end of neck joint chain
    :param neckCurve: str, name of spine cubic curve with 5 cvs matching neck joints
    :param prefix:  str, prefix to name new objects
    :param rigScale: float, scale factor for size of controls
    :param baseRig: instance of base.module.Base class
    :return: dictionary with rig module objects
    """

    # make rig module

    rigmodule = module.Module(prefix = prefix, baseObj = baseRig )

    # make neck curve clusters (maybe skip for now)
