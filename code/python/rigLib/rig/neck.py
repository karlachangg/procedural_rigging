"""
neck @ rig
"""

import maya.cmds as mc

from ..base import module
from ..base import control
from ..utils import joint

class Neck():

    def __init__(self,
                 neckJoints,
                 headJnt,
                 neckCurve,
                 prefix = 'neck',
                 forwardAxis = 'x',
                 upAxis = 'y',
                 middleControl = False,
                 headParentToNeckBase = True,
                 rigScale = 1.0,
                 baseRig = None
                 ):

        """
       :param neckJoints: list(str) list of 5 neck joints
       :param headJnt: str, head joint
       :param neckCurve: str, name of spine cubic curve with 5 cvs matching 5 neck joints
       :param prefix: str, prefix to name new objects
       :param forwardAxis: str, axis pointing down the joint chain. Default 'x'
       :param upAxis: str, axis defining object up direction to be used in IK spline twist setup. Default 'y'
       :param forwardAxis: str, axis pointing down the joint chain. Default 'x'
       :param middleControl: bool, option to make a middle control. Default False
       :param middleControl: bool, option to make a middle control. Default False
       :param headParentToNeckBase: bool, option to parent head end control to starting IK neck control. Default True
       :param baseRig: instance of base.module.Base class
       """

        self.neckJoints = neckJoints
        self.headJnt = headJnt
        self.neckCurve = neckCurve
        self.forwardAxis = forwardAxis
        self.upAxis = upAxis
        self.middleControl = middleControl
        self.headParentToNeckBase = headParentToNeckBase
        self.prefix = prefix
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        # rigParts dictionary
        self.rigParts = {
            'module': self.rigmodule,
            'baseAttachGrp': '',
            'headCtr': '',
            'controls': '',
            'headAttachGrp': ''
        }



    def build(self):

        """
        :param neckJoints: list(str) list of 5 neck joints, last one located at head joint
        :param headJnt: str, head joint at the end of neck joint chain
        :param neckCurve: str, name of spine cubic curve with 5 cvs matching neck joints
        :param prefix:  str, prefix to name new objects
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """


        # duplicate spine joints to make IK spine joints
        ikJoints = joint.duplicateChain(self.neckJoints, oldSuffix='jnt', newSuffix='IK_jnt')


        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)
        controls = []

        # make controls
        headMainCtr = control.Control(prefix='Head', translateTo=self.headJnt,  rotateTo = self.headJnt, scale=self.rigScale,
                                      parent=self.rigmodule.controlsGrp, shape='cube', offsets=['null', 'auto', 'zero'])

        neckBaseCtr = control.Control(prefix='Neck', translateTo=self.neckJoints[0], rotateTo=self.neckJoints[0], scale=self.rigScale * 2,
                                      parent=self.rigmodule.controlsGrp, shape='circleX', offsets=['null', 'auto', 'zero'],
                                      lockChannels = ['s', 'v', 't'], color = 'yellow')


        controls = [neckBaseCtr, headMainCtr]




        if self.middleControl:

            middleCtrl = control.Control(prefix = '{}_Middle'.format(self.prefix), scale = self.rigScale * 1.5,
                                           shape='squareY', color = 'yellow', parent = self.rigmodule.controlsGrp )

            # position middle control
            mc.delete(mc.pointConstraint(headMainCtr.C, neckBaseCtr.C, middleCtrl.Off, mo=0))
            mc.delete(mc.orientConstraint(self.neckJoints[2], middleCtrl.Off, mo=0))

            # Create empty groups to drive middle control
            followHead = mc.group(em=1, n='{}_chestFollow'.format(self.prefix))
            followNeckBase = mc.group(em=1, n='{}_hipsFollow'.format(self.prefix))
            mc.delete(mc.parentConstraint(middleCtrl.C, followHead))
            mc.delete(mc.parentConstraint(middleCtrl.C, followNeckBase))
            mc.parent(followHead, headMainCtr.C)
            mc.parent(followNeckBase, neckBaseCtr.C)

            mc.pointConstraint(followHead, followNeckBase, middleCtrl.Off, mo=1)

            controls.append( middleCtrl)





        # make locators to drive spine curve clusters

        neckCurveCVs = mc.ls('{}.cv[*]'.format(self.neckCurve), fl=1)
        numNeckCVs = len(neckCurveCVs)
        neckCurveShape = mc.listRelatives(self.neckCurve, ad=True, shapes=True)[0]
        driverLocators = []
        driverLocatorOffsets = []
        locGrp = mc.group(n='{}_cv_drivers'.format(self.prefix), em=True)

        for i in range(numNeckCVs):
            cvXform = mc.xform(neckCurveCVs[i], q=True, t=True)
            loc = mc.spaceLocator(n='{}_cv{}_loc'.format(self.prefix, i + 1))[0]
            driverLocators.append(loc)
            # get locator shape node
            locShape = mc.listRelatives(loc, ad=1, shapes=True)[0]
            offset = mc.group(n='{}_cv{}_loc_offset'.format(self.prefix, i + 1))
            driverLocatorOffsets.append(offset)
            mc.parent(offset, locGrp)
            mc.xform(offset, t=cvXform)
            # orient the cv offset group to the neck base joint
            mc.delete(mc.orientConstraint(self.neckJoints[0], offset, mo = 0))
            mc.connectAttr('{}.worldPosition[0]'.format(locShape), neckCurveShape + '.controlPoints[%d]' % (i))

        mc.parent(locGrp, self.rigmodule.partsGrp)
        mc.select(cl=1)

        # make locators to follow 2 IK controls

        neckLocators = ['{}_start_loc'.format(self.prefix)]

        if self.middleControl:
            neckLocators.append('{}_mid_loc'.format(self.prefix))

        neckLocators.append('{}_end_loc'.format(self.prefix))

        neckLocatorOffsets = []
        neckLocatorsGrp = mc.group(n='{}_position_drivers'.format(self.prefix), em=True)

        for i in range(len(neckLocators)):
            mc.spaceLocator(n=neckLocators[i])
            offset = mc.group(n=neckLocators[i] + '_offset')
            neckLocatorOffsets.append(offset)
            mc.parent(offset, neckLocatorsGrp)

        mc.parent(neckLocatorsGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(neckBaseCtr.C, neckLocatorOffsets[0], mo=False)

        if self.middleControl:
            mc.parentConstraint(middleCtrl.C, neckLocatorOffsets[1], mo=False)

        mc.parentConstraint(headMainCtr.C, neckLocatorOffsets[-1], mo=False)


        # connect spine CV drivers to locators at the neckBase and head
        # cv 0
        mc.parentConstraint(neckLocators[0],driverLocatorOffsets[0], mo=True )
        # cv 4
        mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[4], mo=True)

        if self.middleControl:
            # cv 2
            mc.parentConstraint( neckLocators[1], driverLocatorOffsets[2], mo = True)

            # cv 1 values
            cv1SecondDriver = neckLocators[1]
            cv1weight1Value = 0.5
            cv1weight2Value = 0.5

            # cv 3 values
            cv3FirstDriver = neckLocators[1]
            cv3weight1Value = 0.5
            cv3weight2Value = 0.5


        else:
            # cv 2
            mc.parentConstraint(neckLocators[0], neckLocators[-1], driverLocatorOffsets[2], mo=True)
            mc.parentConstraint(neckLocators[0], driverLocatorOffsets[2], e=True, w=0.5)
            mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[2], e=True, w=0.5)

            # cv 1 values
            cv1SecondDriver = neckLocators[-1]
            cv1weight1Value = 0.75
            cv1weight2Value = 0.25

            # cv 3 values
            cv3FirstDriver = neckLocators[0]
            cv3weight1Value = 0.25
            cv3weight2Value = 0.75




        # cv 1
        mc.parentConstraint(neckLocators[0], cv1SecondDriver, driverLocatorOffsets[1], mo=True)
        mc.parentConstraint(neckLocators[0], driverLocatorOffsets[1], e=True, w = cv1weight1Value)
        mc.parentConstraint(cv1SecondDriver, driverLocatorOffsets[1], e=True, w = cv1weight2Value)

        # cv 3
        mc.parentConstraint(cv3FirstDriver, neckLocators[-1], driverLocatorOffsets[3], mo=True)
        mc.parentConstraint(cv3FirstDriver, driverLocatorOffsets[3], e=True, w = cv3weight1Value)
        mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[3], e=True, w = cv3weight2Value)


        # make attach groups
        neckBaseAttachGrp = mc.group(n='{}BaseAttachGrp'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(neckBaseCtr.C, neckBaseAttachGrp, mo=False))
        mc.parentConstraint(neckBaseAttachGrp, neckBaseCtr.Off, mo=True)

        headAttachGrp = mc.group(n='{}_HeadAttachGrp'.format(self.prefix), em=1)
        mc.parent(headAttachGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(self.headJnt, headAttachGrp, mo = False)

        # Parent head control to neckBase control
        if self.headParentToNeckBase:
            mc.parent(headMainCtr.Off, neckBaseCtr.C)
        else:
            mc.parentConstraint( neckBaseAttachGrp, headMainCtr.Off, mo = 1)

        # make IK handle

        neckIk = mc.ikHandle(n='{}_ikHandle'.format(self.prefix), sol='ikSplineSolver', sj=ikJoints[0], ee=ikJoints[-1],
                              c=self.neckCurve, ccv=0, parentCurve=0)[0]
        mc.hide(neckIk)
        mc.parent(neckIk, self.rigmodule.noXformGrp)
        mc.parent(self.neckCurve, self.rigmodule.noXformGrp)

        # set up IK twist

        # Forward Axis setting

        if self.forwardAxis == 'x':
            fAxisAttr = 0
            stretchAxis, squashAxis1, squashAxis2  = 'scaleX', 'scaleY', 'scaleZ'

        elif self.forwardAxis == 'y':
            fAxisAttr = 2
            stretchAxis, squashAxis1, squashAxis2  = 'scaleY', 'scaleX', 'scaleZ'

        elif self.forwardAxis == 'z':
            fAxisAttr = 4
            stretchAxis, squashAxis1, squashAxis2  = 'scaleZ', 'scaleY', 'scaleX'

        elif self.forwardAxis == '-x':
            fAxisAttr = 1
            stretchAxis, squashAxis1, squashAxis2  = 'scaleX', 'scaleY', 'scaleZ'

        elif self.forwardAxis == '-y':
            fAxisAttr = 3
            stretchAxis, squashAxis1, squashAxis2  = 'scaleY', 'scaleX', 'scaleZ'

        elif self.forwardAxis == '-z':
            fAxisAttr = 5
            stretchAxis, squashAxis1, squashAxis2  = 'scaleZ', 'scaleY', 'scaleX'

        # Up Axis setting

        if self.upAxis == 'x':
            upAxisAttr = 6
            x, y, z = 1, 0, 0

        elif self.upAxis == 'y':
            upAxisAttr = 0
            x, y, z = 0, 1, 0

        elif self.upAxis == 'z':
            upAxisAttr = 3
            x, y, z = 0, 0, 1

        elif self.upAxis == '-x':
            upAxisAttr = 7
            x, y, z = -1, 0, 0

        elif self.upAxis == '-y':
            upAxisAttr = 1
            x, y, z = 0, -1, 0

        elif self.upAxis == '-z':
            upAxisAttr = 4
            x, y, z = 0, 0, -1

        mc.setAttr('{}.dTwistControlEnable'.format(neckIk), 1)
        mc.setAttr('{}.dWorldUpType'.format(neckIk), 4)
        mc.setAttr('{}.dForwardAxis'.format(neckIk), fAxisAttr)
        mc.setAttr('{}.dWorldUpAxis'.format(neckIk), upAxisAttr)
        mc.setAttr('{}.dWorldUpVector'.format(neckIk), x, y, z, type="double3")
        mc.setAttr('{}.dWorldUpVectorEnd'.format(neckIk), x, y, z, type="double3")

        mc.connectAttr('{}.worldMatrix[0]'.format(headMainCtr.C), '{}.dWorldUpMatrixEnd'.format(neckIk))
        mc.connectAttr('{}.worldMatrix[0]'.format(neckBaseCtr.C), '{}.dWorldUpMatrix'.format(neckIk))


        # connect IK spine joints to deformation skeleton
        # Connect deformation joints to ik joints

        constraints = []

        for i in range(len(self.neckJoints)):
            constraints.append(mc.parentConstraint(ikJoints[i], self.neckJoints[i], mo=0)[0])
            mc.connectAttr('{}.scale'.format(ikJoints[i]), '{}.scale'.format(self.neckJoints[i]))

        # connect head joint to head control
        headConstraint = mc.orientConstraint(headMainCtr.C, self.headJnt, mo=1)[0]
        constraints.append(headConstraint)

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
        self.HeadOrient = '{}.{}'.format(headMainCtr.C, attr)

        # orient constraint head control

        neckSpaceDriver = neckOrientGrp

        if self.headParentToNeckBase == False:

            duplicate = mc.duplicate(chestOrientGrp, n = '{}_double'.format(chestOrientGrp))
            neckSpaceDriver = duplicate

        chestSpaceDriver = chestOrientGrp
        globalSpaceDriver = globalOrientGrp

        headOrientConstraint = mc.orientConstraint(neckSpaceDriver, chestSpaceDriver, globalSpaceDriver, headMainCtr.Offsets[1], mo=1)[0]
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
        curveInfoNode = mc.arclen(self.neckCurve, ch=True)
        # Connect neck curve to curveInfo
        neck_init_length = mc.getAttr('{}.arcLength'.format(curveInfoNode))
        # Make multiplyDivide node to scale initial amount by global scale
        neckGlobalScale = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_global_scale'.format(self.prefix))
        mc.setAttr('{}.operation'.format(neckGlobalScale), 1)
        mc.setAttr('{}.input1X'.format(neckGlobalScale), neck_init_length)
        mc.connectAttr('{}.scaleX'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(neckGlobalScale))

        # Make multiplyDivide node to get stretch amount
        stretchRatio = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_stretch_ratio'.format(self.prefix))
        mc.setAttr('{}.operation'.format(stretchRatio), 2)
        mc.connectAttr('{}.arcLength'.format(curveInfoNode), '{}.input1X'.format(stretchRatio))
        mc.connectAttr('{}.outputX'.format(neckGlobalScale), '{}.input2X'.format(stretchRatio))

        # Make blender to turn off Stretchy setup

        stretch_attr = 'Stretchy_Neck'
        mc.addAttr(headMainCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)
        self.StretchyAttr = '{}.{}'.format( headMainCtr.C, stretch_attr)

        # make blender node for stretchy spine
        blenderStretch = mc.shadingNode('blendColors', asUtility=True,
                                        n='{}_blenderNeckStretch'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(headMainCtr.C, stretch_attr), '{}.blender'.format(blenderStretch))
        mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.color1.color1R'.format(blenderStretch))
        mc.setAttr('{}.color2.color2R'.format(blenderStretch), 1)

        # Connect output of stretchRatio with all spine joints scaleY except last
        for jnt in ikJoints[:-1]:
            mc.connectAttr('{}.outputR'.format(blenderStretch), '{}.{}'.format(jnt, stretchAxis))

        # set up neck squash

        # Make multiplyDivide node to get square root of neck
        sqrRootNeckStretch = mc.shadingNode('multiplyDivide', asUtility=True, n='sqrRoot_{}_stretch'.format(self.prefix))
        mc.setAttr('{}.operation'.format(sqrRootNeckStretch), 3)
        mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.input1X'.format(sqrRootNeckStretch))
        mc.setAttr('{}.input2X'.format(sqrRootNeckStretch), 0.5)

        # Make multiplyDivide node to divide 1 by square root of spine
        divideSqrRoot = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_divide_sqr_root'.format(self.prefix))
        mc.setAttr('{}.operation'.format(divideSqrRoot), 2)
        mc.setAttr('{}.input1X'.format(divideSqrRoot), 1.0)
        mc.connectAttr('{}.outputX'.format(sqrRootNeckStretch), '{}.input2X'.format(divideSqrRoot))

        # Feed squash values through the blendColors node
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.color1.color1G'.format(blenderStretch))
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.color1.color1B'.format(blenderStretch))
        mc.setAttr('{}.color2.color2G'.format(blenderStretch), 1)
        mc.setAttr('{}.color2.color2B'.format(blenderStretch), 1)

        # Connect output X to scale X and Z of spine joints except first and last
        for jnt in ikJoints[1:-1]:
            mc.connectAttr('{}.outputG'.format(blenderStretch), '{}.{}'.format(jnt, squashAxis1))
            mc.connectAttr('{}.outputB'.format(blenderStretch), '{}.{}'.format(jnt, squashAxis2))

        # organize
        constraintGrp = mc.group(constraints, n='defSkeleton_{}_constraints'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)
        mc.parent(neckBaseAttachGrp, self.rigmodule.partsGrp)
        mc.parent(worldGrp, self.rigmodule.partsGrp)

        # rigParts dictionary
        self.rigParts['baseAttachGrp'] = neckBaseAttachGrp
        self.rigParts['headCtr'] = headMainCtr
        self.rigParts['controls'] = controls
        self.rigParts['headAttachGrp'] = headAttachGrp


    def setInitialValues(self,
                         Stretchy = 1,
                         HeadOrient = 1 ,
                         ):

        mc.setAttr(self.StretchyAttr, Stretchy)
        mc.setAttr(self.HeadOrient, HeadOrient)
