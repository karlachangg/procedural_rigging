'''

fk chain @ rig

'''

import maya.cmds as mc

from ..base import module
from ..base import control
from ..utils import name

def build(
        joints,
        rigScale = 1.0,
        fkParenting = True,
        parent = '',
        shape = 'circle',
        smallestScalePercent = 0.5,
        lockChannels = ['s', 'v'],
        offsets = ['null']
        ):

    """
    :param chainJoints: list (str), list of chain joints
    :param rigScale: float, scale factor for size of controls
    :param prefix: str, prefix to name new objects
    :param fkParenting: bool, parent each control to the previous one to make an FK chain. Default True
    :param baseRig: instance of base.module.Base class
    :return: list of FK controls created
    """

    chainControls = []
    controlScaleIncrement = (1.0 - smallestScalePercent) / len(joints)
    #mainCtrScaleFactor = 10
    jointConstraints = []

    for i in range(len(joints)):

        ctrScale = rigScale * (1.0 - (i * controlScaleIncrement))

        ctr = control.Control(prefix = name.removeSuffix(joints[i]), translateTo = joints[i], rotateTo = joints[i], parent = parent,
                              scale = ctrScale, shape = shape, lockChannels = lockChannels, offsets = offsets)

        constraint = mc.parentConstraint(ctr.C, joints[i], mo = 0)[0]
        chainControls.append(ctr)
        jointConstraints.append(constraint)

    if fkParenting:

        for i in range(len(joints)):

            if i==0:
                continue

            mc.parent(chainControls[i].Off, chainControls[i-1].C )


    return { 'controls': chainControls, 'constraints': jointConstraints , 'topControl': chainControls[0] }
