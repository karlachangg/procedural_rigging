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
        smallestScalePercent = 1.0,
        lockChannels = ['s', 'v'],
        offsets = ['null'],
        color = 'yellow'
        ):

    """
    :param joints: list (str), list of joints to add controls
    :param rigScale: float, scale factor for size of controls
    :param fkParenting: bool, parent each control to the previous one to make an FK chain. Default True
    :param parent: str, name of object to parent controls or control chain to
    :param shape: str, shape of controls
    :param smallestScalePercent: float, set to the smallest control size if you want controls that get smaller
    :param shape: list (str), channels to lock on controls
    :param offsets: str, offset groups on controls
    :return: list of FK controls created
    """

    chainControls = []
    controlScaleIncrement = (1.0 - smallestScalePercent) / len(joints)
    #mainCtrScaleFactor = 10
    jointConstraints = []

    for i in range(len(joints)):

        ctrScale = rigScale * (1.0 - (i * controlScaleIncrement))

        ctr = control.Control(prefix = name.removeSuffix(joints[i]), translateTo = joints[i], rotateTo = joints[i],
                              parent = parent, scale = ctrScale, shape = shape, lockChannels = lockChannels,
                              offsets = offsets, color = color)

        constraint = mc.parentConstraint(ctr.C, joints[i], mo = 1)[0]
        chainControls.append(ctr)
        jointConstraints.append(constraint)

    if fkParenting:

        for i in range(len(joints)):

            if i==0:
                continue

            mc.parent(chainControls[i].Off, chainControls[i-1].C )


    return { 'controls': chainControls, 'constraints': jointConstraints , 'topControl': chainControls[0] }
