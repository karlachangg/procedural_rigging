"""
spine @ rig
"""
import maya.cmds as mc

from ..base import module
from ..base import control

def build(
        spineJoints,
        rootJnt,
        chestJnt,
        spineCurve,
        prefix = 'spine',
        rigScale = 1.0,
        baseRig = None
        ):
    """

    :param spineJoints: list(str) list of 7 spine joints
    :param rootJnt: str, root joint
    :param spineCurve: str, name of spine cubic curve with 7 cvs matching 7 spine joints
    :param bodyLocator: str, reference transform for position of body control
    :param chestLocator: str, reference transform for position of chest control
    :param pelvisLocator: str, reference transform for position of pelvis control
    :param prefix: str, prefix to name new objects
    :param rigScale: float, scale factor for size of controls
    :param baseRig: instance of base.module.Base class
    :return: dictionary with rig module objects
    """

    # make rig module

    rigmodule = module.Module(prefix = prefix, baseObj= baseRig)


    # duplicate spine joints to make IK spine joints
    ikJoints = []

    for jnt in spineJoints:
        ikJoints.append(jnt.replace('jnt', 'IK_jnt'))

    zippedList = list(zip(spineJoints, ikJoints))

    # Duplicate root joint

    ik_root = rootJnt.replace('jnt', 'IK_jnt')
    ik_root_jnt = mc.duplicate(rootJnt, n=ik_root, parentOnly=True)[0]

    mc.select(ik_root_jnt)

    # make IK joint chain

    for defJnt, ikJnt in zippedList:
        mc.joint(n=ikJnt)
        mc.matchTransform(ikJnt, defJnt)

    mc.parent(ik_root_jnt, rigmodule.jointsGrp)


    # make IK/FK controls

    startJnt = spineJoints[0]
    endJnt = spineJoints[-1]

    bodyCtrl = control.Control(prefix='{}_body'.format(prefix), translateTo=startJnt, scale=rigScale * 3,
                               shape = 'circleY', parent=rigmodule.controlsGrp)

    middleCtrlFK = control.Control(prefix='{}_mid_FK'.format(prefix), scale=rigScale * 2.5,
                                 shape = 'circleY', parent=rigmodule.controlsGrp)

    chestCtrlIK = control.Control(prefix='{}_chest_IK'.format(prefix), translateTo=endJnt, scale=rigScale * 1.5,
                               shape = 'square', parent=rigmodule.controlsGrp)

    chestCtrlFK = control.Control(prefix='{}_chest_FK'.format(prefix), translateTo=endJnt, scale=rigScale * 2.5,
                                  shape = 'circleY', parent=rigmodule.controlsGrp)

    hipsCtrlIK = control.Control(prefix='{}_hips_IK'.format(prefix), translateTo=startJnt, scale=rigScale * 2,
                                shape = 'square', parent= bodyCtrl.C)
    middleCtrlIK = control.Control(prefix='{}_mid_IK'.format(prefix), scale=rigScale * 1.5,
                               shape = 'square', parent=rigmodule.controlsGrp,lockChannels = ['s', 'v'])

    # attach controls (parent constraint middleCtrl to chest and hips)
    mc.delete(mc.parentConstraint(chestCtrlIK.C, hipsCtrlIK.C, middleCtrlFK.Off, sr=['x', 'y', 'z'], mo=0))
    mc.delete(mc.parentConstraint(chestCtrlIK.C, hipsCtrlIK.C, middleCtrlIK.Off, sr=['x', 'y', 'z'], mo=0))

    mc.parent(middleCtrlFK.Off, bodyCtrl.C)
    mc.parent(middleCtrlIK.Off, bodyCtrl.C)
    mc.parent(chestCtrlIK.Off, middleCtrlFK.C)
    mc.parent(chestCtrlFK.Off, chestCtrlIK.C)

    mc.parentConstraint(chestCtrlIK.C, hipsCtrlIK.C, middleCtrlIK.Off, mo=1)

    # make locators to drive spine curve clusters

    spineCurveCVs = mc.ls('{}.cv[*]'.format(spineCurve), fl=1)
    numSpineCVs = len(spineCurveCVs)
    spineCurveShape = mc.listRelatives(spineCurve, ad=True, shapes=True)[0]
    driverLocators = []
    driverLocatorOffsets = []
    locGrp = mc.group(n='{}_cv_drivers'.format(prefix), em=True)

    for i in range(numSpineCVs):
        cvXform = mc.xform(spineCurveCVs[i], q=True, t=True)
        loc = mc.spaceLocator(n='{}_cv{}_loc'.format(prefix, i+1))[0]
        driverLocators.append(loc)
        # get locator shape node
        locShape = mc.listRelatives(loc, ad=1, shapes=True)[0]
        offset = mc.group(n='{}_cv{}_loc_offset'.format(prefix, i + 1) )
        driverLocatorOffsets.append(offset)
        mc.parent(offset, locGrp)
        mc.xform(offset, t=cvXform)
        mc.connectAttr('{}.worldPosition[0]'.format(locShape), spineCurveShape + '.controlPoints[%d]' % (i))

    mc.parent(locGrp, rigmodule.partsGrp)
    mc.select(cl=1)

    # make locators to follow 3 IK controls

    spineLocators = ['{}_start_loc'.format(prefix), '{}_mid_loc'.format(prefix), '{}_end_loc'.format(prefix)]
    spineLocatorOffsets = []
    spineLocatorsGrp =mc.group(n='{}_position_drivers'.format(prefix), em=True)
    for i in range(3):
        mc.spaceLocator(n=spineLocators[i])
        offset = mc.group(n = spineLocators[i] + '_offset')
        spineLocatorOffsets.append(offset)
        mc.parent(offset, spineLocatorsGrp)

    mc.parent(spineLocatorsGrp, rigmodule.partsGrp)
    mc.parentConstraint( hipsCtrlIK.C, spineLocatorOffsets[0], mo=False)
    mc.parentConstraint( middleCtrlIK.C, spineLocatorOffsets[1], mo=False)
    mc.parentConstraint( chestCtrlIK.C, spineLocatorOffsets[2], mo=False)


    # connect spine CV drivers to locators at hips, chest, and middle
    mc.parentConstraint(spineLocators[0],driverLocatorOffsets[0], mo=True )
    mc.parentConstraint(spineLocators[1], driverLocatorOffsets[3], mo=True)
    mc.parentConstraint(spineLocators[2], driverLocatorOffsets[6], mo=True)
    # cv 1
    mc.parentConstraint(spineLocators[0], spineLocators[1], driverLocatorOffsets[1], mo=True)
    mc.parentConstraint(spineLocators[0], driverLocatorOffsets[1], e=True, w = 0.66)
    mc.parentConstraint(spineLocators[1], driverLocatorOffsets[1], e=True, w=0.34)
    # cv 2
    mc.parentConstraint(spineLocators[0], spineLocators[1], driverLocatorOffsets[2], mo=True)
    mc.parentConstraint(spineLocators[0], driverLocatorOffsets[2], e=True, w=0.34)
    mc.parentConstraint(spineLocators[1], driverLocatorOffsets[2], e=True, w=0.66)
    # cv 4
    mc.parentConstraint(spineLocators[1], spineLocators[2], driverLocatorOffsets[4], mo=True)
    mc.parentConstraint(spineLocators[1], driverLocatorOffsets[4], e=True, w=0.66)
    mc.parentConstraint(spineLocators[2], driverLocatorOffsets[4], e=True, w=0.34)
    # cv 5
    mc.parentConstraint(spineLocators[1], spineLocators[2], driverLocatorOffsets[5], mo=True)
    mc.parentConstraint(spineLocators[1], driverLocatorOffsets[5], e=True, w=0.34)
    mc.parentConstraint(spineLocators[2], driverLocatorOffsets[5], e=True, w=0.66)


    # make IK handle

    spineIk = mc.ikHandle(n = '{}_ikHandle'.format(prefix), sol = 'ikSplineSolver', sj = ikJoints[0], ee = ikJoints[-1],
                          c = spineCurve, ccv = 0, parentCurve = 0 )[0]
    mc.hide(spineIk)
    mc.parent(spineIk, rigmodule.noXformGrp)
    mc.parent(spineCurve, rigmodule.noXformGrp)



    # set up IK twist

    mc.setAttr('{}.dTwistControlEnable'.format(spineIk), 1 )
    mc.setAttr('{}.dWorldUpType'.format(spineIk), 4 )
    mc.setAttr('{}.dForwardAxis'.format(spineIk), 2)
    mc.setAttr('{}.dWorldUpAxis'.format(spineIk), 4)
    mc.setAttr('{}.dWorldUpVector'.format(spineIk), 0.0, 0.0, -1.0, type="double3")
    mc.setAttr('{}.dWorldUpVectorEnd'.format(spineIk), 0.0, 0.0, -1.0, type="double3")

    mc.connectAttr( '{}.worldMatrix[0]'.format(chestCtrlIK.C), '{}.dWorldUpMatrixEnd'.format(spineIk) )
    mc.connectAttr('{}.worldMatrix[0]'.format(hipsCtrlIK.C), '{}.dWorldUpMatrix'.format(spineIk))

    # set up stretchy spine

    # Make curve info node

    curveInfoNode = mc.arclen(spineCurve, ch=True)

    # Connect spine curve to curveInfo
    spine_init_length = mc.getAttr('{}.arcLength'.format(curveInfoNode))

    # Make multiplyDivide node to scale initial amount by global scale
    spineGlobalScale = mc.shadingNode('multiplyDivide', asUtility=True, n='spine_global_scale')

    mc.setAttr('{}.operation'.format(spineGlobalScale), 1)
    mc.setAttr('{}.input1X'.format(spineGlobalScale), spine_init_length)
    mc.connectAttr('{}.scaleX'.format(baseRig.global1Ctrl.C), '{}.input2X'.format(spineGlobalScale))

    # Make multiplyDivide node to get stretch amount
    stretchRatio = mc.shadingNode('multiplyDivide', asUtility=True, n='spine_stretch_ratio')

    mc.setAttr('{}.operation'.format(stretchRatio), 2)
    mc.connectAttr('{}.arcLength'.format(curveInfoNode), '{}.input1X'.format(stretchRatio))
    mc.connectAttr('{}.outputX'.format(spineGlobalScale), '{}.input2X'.format(stretchRatio))

    # Connect output of stretchRatio with all spine joints scaleY except last
    for jnt in ikJoints[:-1]:
        mc.connectAttr('{}.outputX'.format(stretchRatio),'{}.scaleY'.format(jnt))

    # set up spine squash

    # Make multiplyDivide node to get square root of spine
    sqrRootSpineStretch = mc.shadingNode('multiplyDivide', asUtility=True, n='sqrRoot_spine_stretch')
    mc.setAttr('{}.operation'.format(sqrRootSpineStretch), 3)
    mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.input1X'.format(sqrRootSpineStretch))
    mc.setAttr('{}.input2X'.format(sqrRootSpineStretch), 0.5)

    # Make multiplyDivide node to divide 1 by square root of spine
    divideSqrRoot = mc.shadingNode('multiplyDivide', asUtility=True, n='divide_sqr_root')
    mc.setAttr('{}.operation'.format(divideSqrRoot), 2)
    mc.setAttr('{}.input1X'.format(divideSqrRoot), 1.0)
    mc.connectAttr('{}.outputX'.format(sqrRootSpineStretch), '{}.input2X'.format(divideSqrRoot))

    # Connect output X to scale X and Z of spine joints except first and last
    for jnt in ikJoints[1:-1]:
        mc.connectAttr('{}.outputX'.format(divideSqrRoot),'{}.scaleX'.format(jnt))
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.scaleZ'.format(jnt))

    # connect IK spine joints to deformation skeleton

    # parent constraint deformation skeleton to IK spine skeleton

    # direct connect scale values
    constraints = []
    for defJnt, ikJnt in zippedList[1:-1]:
        constraints.append(mc.parentConstraint(ikJnt, defJnt, mo=0)[0])
        mc.connectAttr('{}.scale'.format(ikJnt), '{}.scale'.format(defJnt))

    # attach chest and root joints

    constraints.append(mc.parentConstraint(chestCtrlFK.C, chestJnt, mo = 1)[0])
    constraints.append(mc.parentConstraint(hipsCtrlIK.C, rootJnt, mo=1)[0])

    # organize
    constraintGrp = mc.group(constraints, n = 'defSkeleton_spine_constraints')
    mc.parent(constraintGrp, baseRig.noXformGrp)

    return{'module':rigmodule}

