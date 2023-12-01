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
    
    # duplicate spine joints to make IK spine joints
    ikJoints = []

    for jnt in neckJoints:
        ikJoints.append(jnt.replace('jnt', 'IK_jnt'))

    zippedList = list(zip(neckJoints, ikJoints))

    # make IK joint chain

    for defJnt, ikJnt in zippedList:
        mc.joint(n=ikJnt)
        mc.matchTransform(ikJnt, defJnt)

    mc.parent(ikJoints[0], rigmodule.jointsGrp)

    # make controls
    headMainCtr = control.Control(prefix='Head', translateTo=headJnt, scale=rigScale * 2,
                                  parent=rigmodule.controlsGrp, shape='circleY', offsets=['null', 'auto', 'zero'])

    neckBaseCtr = control.Control(prefix='Neck', translateTo=neckJoints[0], scale=rigScale * 1,
                                  parent=rigmodule.controlsGrp, shape='circleY', offsets=['null', 'auto', 'zero'],
                                  lockChannels = ['s', 'v', 't'])

    # make locators to drive spine curve clusters

    neckCurveCVs = mc.ls('{}.cv[*]'.format(neckCurve), fl=1)
    numNeckCVs = len(neckCurveCVs)
    neckCurveShape = mc.listRelatives(neckCurve, ad=True, shapes=True)[0]
    driverLocators = []
    driverLocatorOffsets = []
    locGrp = mc.group(n='{}_cv_drivers'.format(prefix), em=True)

    for i in range(numNeckCVs):
        cvXform = mc.xform(neckCurveCVs[i], q=True, t=True)
        loc = mc.spaceLocator(n='{}_cv{}_loc'.format(prefix, i + 1))[0]
        driverLocators.append(loc)
        # get locator shape node
        locShape = mc.listRelatives(loc, ad=1, shapes=True)[0]
        offset = mc.group(n='{}_cv{}_loc_offset'.format(prefix, i + 1))
        driverLocatorOffsets.append(offset)
        mc.parent(offset, locGrp)
        mc.xform(offset, t=cvXform)
        mc.connectAttr('{}.worldPosition[0]'.format(locShape), neckCurveShape + '.controlPoints[%d]' % (i))

    mc.parent(locGrp, rigmodule.partsGrp)
    mc.select(cl=1)

    # make locators to follow 2 IK controls

    neckLocators = ['{}_start_loc'.format(prefix), '{}_end_loc'.format(prefix)]
    neckLocatorOffsets = []
    neckLocatorsGrp = mc.group(n='{}_position_drivers'.format(prefix), em=True)

    for i in range(len(neckLocators)):
        mc.spaceLocator(n=neckLocators[i])
        offset = mc.group(n=neckLocators[i] + '_offset')
        neckLocatorOffsets.append(offset)
        mc.parent(offset, neckLocatorsGrp)

    mc.parent(neckLocatorsGrp, rigmodule.partsGrp)
    mc.parentConstraint(neckBaseCtr.C, neckLocatorOffsets[0], mo=False)
    mc.parentConstraint(headMainCtr.C, neckLocatorOffsets[1], mo=False)


    # connect spine CV drivers to locators at the neckBase and head
    # cv 0
    mc.parentConstraint(neckLocators[0],driverLocatorOffsets[0], mo=True )
    # cv 4
    mc.parentConstraint(neckLocators[1], driverLocatorOffsets[4], mo=True)

    # cv 1
    mc.parentConstraint(neckLocators[0], neckLocators[1], driverLocatorOffsets[1], mo=True)
    mc.parentConstraint(neckLocators[0], driverLocatorOffsets[1], e=True, w = 0.75)
    mc.parentConstraint(neckLocators[1], driverLocatorOffsets[1], e=True, w=0.25)
    # cv 2
    mc.parentConstraint(neckLocators[0], neckLocators[1], driverLocatorOffsets[2], mo=True)
    mc.parentConstraint(neckLocators[0], driverLocatorOffsets[2], e=True, w=0.5)
    mc.parentConstraint(neckLocators[1], driverLocatorOffsets[2], e=True, w=0.5)
    # cv 4
    mc.parentConstraint(neckLocators[0], neckLocators[1], driverLocatorOffsets[3], mo=True)
    mc.parentConstraint(neckLocators[0], driverLocatorOffsets[3], e=True, w=0.25)
    mc.parentConstraint(neckLocators[1], driverLocatorOffsets[3], e=True, w=0.75)


    # make attach groups
    neckBaseAttachGrp = mc.group(n='neckBaseAttach_grp', em=1)
    mc.delete(mc.parentConstraint(neckBaseCtr.C, neckBaseAttachGrp, mo=False))
    mc.parentConstraint(neckBaseAttachGrp, neckBaseCtr.Off, mo=True)

    # Parent head control to neckBase control
    mc.parent(headMainCtr.Off, neckBaseCtr.C)

    # make IK handle

    neckIk = mc.ikHandle(n='{}_ikHandle'.format(prefix), sol='ikSplineSolver', sj=ikJoints[0], ee=ikJoints[-1],
                          c=neckCurve, ccv=0, parentCurve=0)[0]
    mc.hide(neckIk)
    mc.parent(neckIk, rigmodule.noXformGrp)
    mc.parent(neckCurve, rigmodule.noXformGrp)

    # set up IK twist

    mc.setAttr('{}.dTwistControlEnable'.format(neckIk), 1)
    mc.setAttr('{}.dWorldUpType'.format(neckIk), 4)
    mc.setAttr('{}.dForwardAxis'.format(neckIk), 2)
    mc.setAttr('{}.dWorldUpAxis'.format(neckIk), 4)
    mc.setAttr('{}.dWorldUpVector'.format(neckIk), 0.0, 0.0, -1.0, type="double3")
    mc.setAttr('{}.dWorldUpVectorEnd'.format(neckIk), 0.0, 0.0, -1.0, type="double3")

    mc.connectAttr('{}.worldMatrix[0]'.format(headMainCtr.C), '{}.dWorldUpMatrixEnd'.format(neckIk))
    mc.connectAttr('{}.worldMatrix[0]'.format(neckBaseCtr.C), '{}.dWorldUpMatrix'.format(neckIk))


    # connect IK spine joints to deformation skeleton

    constraints = []
    for defJnt, ikJnt in zippedList[:-1]:
        constraints.append(mc.parentConstraint(ikJnt, defJnt, mo=0)[0])
        mc.connectAttr('{}.scale'.format(ikJnt), '{}.scale'.format(defJnt))

    # connect head joint to head control
    mc.parentConstraint(headMainCtr.C, headJnt, mo=1)

    # make head orient groups
    worldGrp = mc.group(n='globalOrient', em=True)

    globalOrientGrp = mc.group(n='head_globalOrient', em=True)
    chestOrientGrp = mc.group(n='head_chestOrient', em=True)
    neckOrientGrp = mc.group(n='head_neckOrient', em=True)

    mc.delete(mc.parentConstraint(headMainCtr.C, globalOrientGrp, mo=0))
    mc.delete(mc.parentConstraint(headMainCtr.C, chestOrientGrp, mo=0))
    mc.delete(mc.parentConstraint(headMainCtr.C, neckOrientGrp, mo=0))
    mc.parent(chestOrientGrp, neckBaseAttachGrp)
    mc.parent(globalOrientGrp, neckBaseAttachGrp)
    mc.parent(neckOrientGrp, neckBaseAttachGrp)

    mc.orientConstraint(worldGrp, globalOrientGrp, mo=1)
    mc.orientConstraint(neckBaseCtr.C, neckOrientGrp, mo=1)


    # setup head orientation switch
    attr = "Head_Orient"
    mc.addAttr(headMainCtr.C, ln=attr, at='enum', enumName='neck:chest:world', k=1)
    mc.setAttr('{}.{}'.format(headMainCtr.C, attr), cb=1)
    mc.setAttr('{}.{}'.format(headMainCtr.C, attr), 2)
    # orient constraint head control
    headOrientConstraint = mc.orientConstraint(neckOrientGrp, chestOrientGrp, globalOrientGrp, headMainCtr.Off, mo=1)[0]
    weights = mc.orientConstraint(headOrientConstraint, q=1, weightAliasList = 1)

    # set driven key for neck orient
    mc.setAttr('{}.{}'.format(headMainCtr.C, attr), 0)
    mc.setAttr( '{}.{}'.format(headOrientConstraint, weights[0]), 1)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 0)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 0)
    for i in range(len(weights)):
        mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd='{}.{}'.format(headMainCtr.C, attr))

    # set driven key for chest orient
    mc.setAttr('{}.{}'.format(headMainCtr.C, attr), 1)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[0]), 0)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 1)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 0)
    for i in range(len(weights)):
        mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd = '{}.{}'.format(headMainCtr.C, attr))

    # set driven key for global orient
    mc.setAttr('{}.{}'.format(headMainCtr.C, attr), 2)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[0]), 0)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 0)
    mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 1)
    for i in range(len(weights)):
        mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd='{}.{}'.format(headMainCtr.C, attr))

    # set default to follow chest
    mc.setAttr('{}.{}'.format(headMainCtr.C, attr), 1)

    # set up stretchy neck

    # Make curve info node
    curveInfoNode = mc.arclen(neckCurve, ch=True)
    # Connect neck curve to curveInfo
    neck_init_length = mc.getAttr('{}.arcLength'.format(curveInfoNode))
    # Make multiplyDivide node to scale initial amount by global scale
    neckGlobalScale = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_global_scale'.format(prefix))
    mc.setAttr('{}.operation'.format(neckGlobalScale), 1)
    mc.setAttr('{}.input1X'.format(neckGlobalScale), neck_init_length)
    mc.connectAttr('{}.scaleX'.format(baseRig.global1Ctrl.C), '{}.input2X'.format(neckGlobalScale))

    # Make multiplyDivide node to get stretch amount
    stretchRatio = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_stretch_ratio'.format(prefix))
    mc.setAttr('{}.operation'.format(stretchRatio), 2)
    mc.connectAttr('{}.arcLength'.format(curveInfoNode), '{}.input1X'.format(stretchRatio))
    mc.connectAttr('{}.outputX'.format(neckGlobalScale), '{}.input2X'.format(stretchRatio))

    # Connect output of stretchRatio with all spine joints scaleY except last
    for jnt in ikJoints[:-1]:
        mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.scaleY'.format(jnt))

    # set up neck squash

    # Make multiplyDivide node to get square root of neck
    sqrRootNeckStretch = mc.shadingNode('multiplyDivide', asUtility=True, n='sqrRoot_{}_stretch'.format(prefix))
    mc.setAttr('{}.operation'.format(sqrRootNeckStretch), 3)
    mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.input1X'.format(sqrRootNeckStretch))
    mc.setAttr('{}.input2X'.format(sqrRootNeckStretch), 0.5)

    # Make multiplyDivide node to divide 1 by square root of spine
    divideSqrRoot = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_divide_sqr_root'.format(prefix))
    mc.setAttr('{}.operation'.format(divideSqrRoot), 2)
    mc.setAttr('{}.input1X'.format(divideSqrRoot), 1.0)
    mc.connectAttr('{}.outputX'.format(sqrRootNeckStretch), '{}.input2X'.format(divideSqrRoot))

    # Connect output X to scale X and Z of spine joints except first and last
    for jnt in ikJoints[1:-1]:
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.scaleX'.format(jnt))
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.scaleZ'.format(jnt))

    # organize
    constraintGrp = mc.group(constraints, n='defSkeleton_{}_constraints'.format(prefix))
    mc.parent(constraintGrp, baseRig.noXformGrp)
    mc.parent(neckBaseAttachGrp, rigmodule.partsGrp)
    mc.parent(worldGrp, rigmodule.partsGrp)


    return {'module': rigmodule, 'baseAttachGrp': neckBaseAttachGrp}