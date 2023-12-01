"""
leg @ rig
"""

import maya.cmds as mc

from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name

def build(
        legJoints,
        topToeJoints,
        pvLocator,
        scapulaJnt = '',
        prefix = 'l_leg',
        rigScale = 1.0,
        baseRig = None
        ):
    """

    :param legJoints: list(str), shoulder - elbow - hand
    :param topToeJoints: list(str), top metacarpal toe joints
    :param pvLocator: str, reference locator for position of pole vector control
    :param scapulaJnt: str, optional, scapula joint
    :param prefix: str, prefix to name new objects
    :param rigScale: float, scale factor for size of controls
    :param baseRig: instance of base.module.Base class
    :return: dictionary with rig module objects
    """

    # make rig module

    rigmodule = module.Module(prefix=prefix, baseObj=baseRig)

    # make attach groups
    bodyAttachGrp = mc.group(n= '{}_BodyAttach_grp', em=1, p = rigmodule.partsGrp)
    baseAttachGrp = mc.group(n='{}_BaseAttach_grp', em=1, p=rigmodule.partsGrp)

    # make controls

    if scapulaJnt:
        scapulaCtr = control.Control(prefix = '{}_Scapula'.format(prefix), translateTo= scapulaJnt, rotateTo=scapulaJnt,
                                     scale = rigScale * 3, parent = rigmodule.controlsGrp, shape = 'sphere',
                                     lockChannels= [ 's', 'v'])
    footCtr = control.Control(prefix = '{}_Foot'.format(prefix), translateTo= legJoints[2],
                                     scale = rigScale * 3, parent = rigmodule.controlsGrp, shape = 'circleY')
    ballCtr = control.Control(prefix = '{}_Ball'.format(prefix), translateTo= legJoints[3],rotateTo = legJoints[3],
                                     scale = rigScale * 2, parent = footCtr.C, shape = 'circleZ')
    poleVectorCtr = control.Control(prefix = '{}_pv'.format(prefix), translateTo= pvLocator,
                                     scale = rigScale, parent = rigmodule.controlsGrp, shape = 'sphere')

    toeIKControls = []

    for topToeJnt in topToeJoints:
        toePrefix = name.removeSuffix(topToeJnt)[:-1]
        toeEndJnt = mc.listRelatives(topToeJnt, ad = 1, type = 'joint')[0]

        toeIKCtr = control.Control(prefix = toePrefix, translateTo= toeEndJnt,
                                     scale = rigScale, parent = footCtr.C, shape = 'circleY')
        toeIKControls.append(toeIKCtr)

    # make IK handles

    if scapulaJnt:

        scapulaIK = mc.ikHandle(n='{}_scapula_ik'.format(prefix), sol='ikSCsolver', sj=scapulaJnt, ee=legJoints[0])[0]

    legIK = mc.ikHandle(n = '{}_leg_ik'.format(prefix), sol= 'ikRPsolver', sj = legJoints[0], ee = legJoints[2])[0]
    ballIK = mc.ikHandle(n='{}_ball_ik'.format(prefix), sol='ikSCsolver', sj=legJoints[2], ee=legJoints[3])[0]
    mainToeIK = mc.ikHandle(n='{}_mainToe_ik'.format(prefix), sol='ikSCsolver', sj=legJoints[3], ee=legJoints[4])[0]

    mc.hide(legIK, ballIK, mainToeIK)

    for i, topToeJnt in enumerate(topToeJoints):
        toePrefix = name.removeSuffix(topToeJnt)[:-1]
        toeJoints = joint.listHierarchy(topToeJnt)

        toeIK = mc.ikHandle(n='{}_ik'.format(toePrefix), sol='ikSCsolver', sj=toeJoints[1], ee=toeJoints[-1])[0]
        mc.hide(toeIK)
        mc.parent(toeIK, toeIKControls[i].C)

    # attach controls

    mc.parentConstraint(bodyAttachGrp, poleVectorCtr.Off, mo = 1)

    if scapulaJnt:

        mc.parentConstraint(baseAttachGrp, scapulaCtr.Off, mo = 1)

    # attach objects to controls

    mc.parent(legIK, ballCtr.C)
    mc.parent(ballIK, mainToeIK, footCtr.C)

    mc.poleVectorConstraint(poleVectorCtr.C, legIK)

    if scapulaJnt:
        mc.parent(scapulaIK, scapulaCtr.C)
        mc.pointConstraint(scapulaCtr.C, scapulaJnt)

    # make pole vector connection line

    pvLinePos1 = mc.xform(legJoints[1], q=1, t=1, ws=1)
    pvLinePos2 = mc.xform(pvLocator, q=1, t=1, ws=1)
    poleVectorCurve = mc.curve(n = '{}_pv_curve'.format(prefix), d = 1, p = [pvLinePos1, pvLinePos2] )
    mc.cluster('{}.cv[0]'.format(poleVectorCurve), n = '{}_pv1_cls'.format(prefix), wn = [ legJoints[1], legJoints[1] ], bs = True )
    mc.cluster('{}.cv[1]'.format(poleVectorCurve), n='{}_pv2_cls'.format(prefix), wn=[poleVectorCtr.C, poleVectorCtr.C ], bs=True)
    mc.parent(poleVectorCurve, rigmodule.controlsGrp )
    mc.setAttr('{}.template'.format(poleVectorCurve), 1)

    return {'module': rigmodule, 'baseAttachGrp': baseAttachGrp, 'bodyAttachGrp': bodyAttachGrp}