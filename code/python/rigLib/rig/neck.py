"""
neck @ rig
"""

import maya.cmds as mc

from . import fkChain

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
        self.prefix = prefix
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        # rigParts dictionary
        self.rigParts = {
            'module': self.rigmodule,
            'baseAttachGrp': '',
            'fkControls': '',
            'ikControls': '',
            'headAttachGrp': '',
            'headCtr': ''

        }

    def build(self):

        # Construct a list of all neck joints: neck joints + head
        self.neckHeadJoints = []
        self.neckHeadJoints.extend(self.neckJoints)
        self.neckHeadJoints.append(self.headJnt)

        # Make body attach group
        self.neckBaseAttachGrp = mc.group(n='{}_BaseAttachGrp'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(self.neckJoints[0], self.neckBaseAttachGrp, mo=False))
        mc.parent(self.neckBaseAttachGrp, self.rigmodule.partsGrp)

        # Make FK rig
        fkRig = self.buildFK()

        # Define rigParts properties
        self.rigParts['fkControls'] = fkRig['controls']


        # Make IK rig
        ikRig = self.buildIK()
        self.rigParts['ikControls'] = ikRig['controls']

        # Hake FK base control and IK Neck Base control follow neckBaseAttachGrp
        mc.parentConstraint(self.neckBaseAttachGrp, fkRig['controls'][0].Off, mo=True)
        mc.parentConstraint(self.neckBaseAttachGrp, ikRig['controls'][1].Off, mo=True)

        # Connect deformation joints to fk and ik joints
        constraints = []
        for i in range(len(self.neckHeadJoints)):
            constraints.append(mc.parentConstraint(fkRig['joints'][i], ikRig['joints'][i], self.neckHeadJoints[i], mo=0)[0])

        # Make switch control

        switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo=self.neckHeadJoints[0],
                                    scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='plus',
                                    color='yellow')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(switchCtr.C, ln=switch_attr, at='double', min=0, max=1, dv=0, k=1)

        control._rotateCtrlShape(switchCtr, axis='x', value=90)

        # Define rigParts properties
        self.rigParts['switchControl'] = switchCtr

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_neck_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in constraints:
            weights = mc.parentConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        # FK IK control visibility
        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        # Setup blend between joint scales

        for i in range(len(self.neckHeadJoints)):
            blendNode = mc.shadingNode('blendColors', asUtility=True, n='{}_jointScale_blend{}'.format(self.prefix, i))
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.blender'.format(blendNode))

            mc.connectAttr('{}.sx'.format(ikRig['joints'][i]), '{}.color1.color1R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(ikRig['joints'][i]), '{}.color1.color1G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(ikRig['joints'][i]), '{}.color1.color1B'.format(blendNode))

            mc.connectAttr('{}.sx'.format(fkRig['joints'][i]), '{}.color2.color2R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(fkRig['joints'][i]), '{}.color2.color2G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(fkRig['joints'][i]), '{}.color2.color2B'.format(blendNode))

            mc.connectAttr('{}.outputR'.format(blendNode), '{}.sx'.format(self.neckHeadJoints[i]))
            mc.connectAttr('{}.outputG'.format(blendNode), '{}.sy'.format(self.neckHeadJoints[i]))
            mc.connectAttr('{}.outputB'.format(blendNode), '{}.sz'.format(self.neckHeadJoints[i]))

        # organize
        constraintGrp = mc.group(constraints, n='defSkeleton_{}_constraints'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

        # move switch ctr
        mc.move(5, switchCtr.Off, x=True, os=1)

        # make attach groups
        headAttachGrp = mc.group(n='{}_headAttach_grp'.format(self.prefix), em=True)
        mc.parentConstraint(self.headJnt, headAttachGrp, mo=0)
        mc.parent(headAttachGrp, self.rigmodule.partsGrp)





        # rigParts dictionary
        self.rigParts['headAttachGrp'] = headAttachGrp
        self.rigParts['baseAttachGrp'] = self.neckBaseAttachGrp

    def buildFK(self):

        '''
        Create a simple FK rig. Duplicates deformation joints, creates simple FK rig on duplicate, and returns
        library of objects
        '''

        # duplicate neck joints to make FK neck joints
        fkNeckJoints = joint.duplicateChain(self.neckJoints, oldSuffix='jnt', newSuffix='FK_jnt')


        # Duplicate head joint
        fk_head_name = self.headJnt.replace('jnt', 'FK_jnt')
        fkHeadJoint = mc.duplicate(self.headJnt, n = fk_head_name, parentOnly=True)[0]

        mc.parent(fkNeckJoints[0], self.rigmodule.jointsGrp)
        mc.parent(fkHeadJoint, fkNeckJoints[-1])

        # Make fkJoints list to return
        fkJoints = []
        fkJoints.extend(fkNeckJoints)
        fkJoints.append(fkHeadJoint)

        fkNeckControlChain = fkChain.build(joints = fkNeckJoints[:-1], rigScale=self.rigScale, parent = self.rigmodule.controlsGrp,
                                       shape='circleX', lockChannels=['t'], color='cyan')

        headFKControl =  control.Control(prefix= 'HeadFK_{}'.format(self.prefix), translateTo=self.headJnt,
                                        rotateTo = self.headJnt, scale=self.rigScale * 2,
                                        parent= fkNeckControlChain['controls'][-1].C, shape='circleX',
                                       offsets=['null', 'auto', 'zero'], color = 'cyan')

        mc.parentConstraint(headFKControl.C, fkHeadJoint, mo = 1)[0]

        controls = []
        controls.extend(fkNeckControlChain['controls'])
        controls.append(headFKControl)

        # Add space switches on head control

        headOrientTargetsGrp = mc.group(n='{}_FKHeadOrientTargets'.format(self.prefix), em=1)
        mc.parent(headOrientTargetsGrp, self.rigmodule.partsGrp)

        globalOrientGrp = mc.group(n='{}_FKHead_globalFollow'.format(self.prefix), em=True)
        chestOrientGrp = mc.group(n='{}_FKHead_chestFollow'.format(self.prefix), em=True)
        neckOrientGrp = mc.group(n='{}_FKHead_neckFollow'.format(self.prefix), em=True)

        mc.parent(globalOrientGrp, headOrientTargetsGrp)
        mc.parent(chestOrientGrp, headOrientTargetsGrp)
        mc.parent(neckOrientGrp, headOrientTargetsGrp)

        mc.delete(mc.parentConstraint(headFKControl.C, headOrientTargetsGrp, mo=0))
        mc.parentConstraint(fkNeckControlChain['controls'][-1].C, headOrientTargetsGrp, mo=0)

        mc.orientConstraint(self.baseRig.global1Ctrl.C, globalOrientGrp, mo=1)
        mc.orientConstraint(self.neckBaseAttachGrp, chestOrientGrp, mo=1)

        # setup head orientation switch
        orient_attr = "Head_Orient"
        mc.addAttr(headFKControl.C, ln=orient_attr, at='enum', enumName='neck:chest:world', k=1)
        #mc.addAttr(headFKControl.C, ln=orient_attr, at='double', min=0, max=1, dv=0, k=1)
        mc.setAttr('{}.{}'.format(headFKControl.C, orient_attr), cb=1)
        #mc.setAttr('{}.{}'.format(headCtr.C, follow_attr), 1)
        self.HeadFKOrient = '{}.{}'.format(headFKControl.C, orient_attr)

        # orient constraint head control

        neckSpaceDriver = neckOrientGrp
        chestSpaceDriver = chestOrientGrp
        globalSpaceDriver = globalOrientGrp

        headOrientConstraint = mc.orientConstraint(neckSpaceDriver, chestSpaceDriver, globalSpaceDriver, headFKControl.Off, mo=1)[0]
        mc.setAttr('{}.interpType'.format(headOrientConstraint), 2)
        weights = mc.orientConstraint(headOrientConstraint, q=1, weightAliasList=1)

        #mc.connectAttr('{}.{}'.format(headFKControl.C, orient_attr), '{}.{}'.format(headOrientConstraint, weights[1]))


        # set driven key for neck orient
        mc.setAttr('{}.{}'.format(headFKControl.C, orient_attr), 0)
        mc.setAttr( '{}.{}'.format(headOrientConstraint, weights[0]), 1)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 0)
        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd='{}.{}'.format(headFKControl.C, orient_attr))

        # set driven key for chest orient
        mc.setAttr('{}.{}'.format(headFKControl.C, orient_attr), 1)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[0]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 1)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 0)
        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd = '{}.{}'.format(headFKControl.C, orient_attr))

        # set driven key for global orient
        mc.setAttr('{}.{}'.format(headFKControl.C, orient_attr), 2)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[0]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 1)
        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd='{}.{}'.format(headFKControl.C, orient_attr))

        # set default to follow neck
        mc.setAttr('{}.{}'.format(headFKControl.C, orient_attr), 0)




        return {'joints': fkJoints, 'controls': controls}



    def buildIK(self):

        ikNeckJoints = joint.duplicateChain(self.neckJoints, oldSuffix='jnt', newSuffix='IK_jnt')

        # Duplicate head joint
        ik_head_name = self.headJnt.replace('jnt', 'IK_jnt')
        ikHeadJoint = mc.duplicate(self.headJnt, n=ik_head_name, parentOnly=True)[0]

        mc.parent(ikNeckJoints[0], self.rigmodule.jointsGrp)
        mc.parent(ikHeadJoint, ikNeckJoints[-1])

        # Make ikJoints list to return
        ikJoints = []
        ikJoints.extend(ikNeckJoints)
        ikJoints.append(ikHeadJoint)


        controls = []


        # make controls
        headCtr = control.Control(prefix='Head', translateTo = ikHeadJoint,  rotateTo = ikHeadJoint, scale=self.rigScale,
                                    parent=self.rigmodule.controlsGrp, shape='cube', offsets=['null', 'auto', 'zero'],
                                    color = 'cyan')

        neckBaseCtr = control.Control(prefix='Neck', translateTo= ikNeckJoints[0], rotateTo= ikNeckJoints[0],
                                      scale=self.rigScale * 2, parent=self.rigmodule.controlsGrp,
                                      shape='cube', offsets=['null', 'auto', 'zero'],
                                      lockChannels = ['s', 'v', 't'], color = 'cyan')

        neckHybridFKCtr = control.Control(prefix='{}_BaseFK'.format(self.prefix), translateTo = ikNeckJoints[0],
                                    rotateTo = ikNeckJoints[0], offsets=['null', 'auto', 'zero'],
                                    scale=self.rigScale * 2, shape='circleX', parent=self.rigmodule.controlsGrp)

        mc.parent(headCtr.Off, neckHybridFKCtr.C)
        #mc.parent(neckBaseCtr.Off, neckHybridFKCtr.C)


        controls = [neckHybridFKCtr, neckBaseCtr, headCtr ]




        if self.middleControl:

            middleCtrl = control.Control(prefix = '{}_Middle'.format(self.prefix), scale = self.rigScale * 1.5,
                                           shape='squareY', color = 'cyan', parent = self.rigmodule.controlsGrp )

            # position middle control
            mc.delete(mc.pointConstraint(headCtr.C, neckBaseCtr.C, middleCtrl.Off, mo=0))
            mc.delete(mc.orientConstraint(ikNeckJoints[2], middleCtrl.Off, mo=0))

            # Create empty groups to drive middle control
            followHead = mc.group(em=1, n='{}_chestFollow'.format(self.prefix))
            followNeckBase = mc.group(em=1, n='{}_hipsFollow'.format(self.prefix))
            mc.delete(mc.parentConstraint(middleCtrl.C, followHead))
            mc.delete(mc.parentConstraint(middleCtrl.C, followNeckBase))
            mc.parent(followHead, headCtr.C)
            mc.parent(followNeckBase, neckBaseCtr.C)

            mc.pointConstraint(followHead, followNeckBase, middleCtrl.Off, mo=1)

            mc.parent(middleCtrl.Off, neckHybridFKCtr.C)

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
            mc.delete(mc.orientConstraint(ikNeckJoints[0], offset, mo = 0))
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

        mc.parentConstraint(headCtr.C, neckLocatorOffsets[-1], mo=False)


        # connect spine CV drivers to locators at the neckBase and head
        # cv 0
        mc.parentConstraint(neckLocators[0],driverLocatorOffsets[0], mo=True )
        # cv 1
        mc.parentConstraint(neckLocators[0], driverLocatorOffsets[1], mo=True)
        # cv 6
        mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[6], mo=True)
        # cv 5
        mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[5], mo=True)

        if self.middleControl:
            # cv 3
            mc.parentConstraint( neckLocators[1], driverLocatorOffsets[3], mo = True)

            # cv 2 values
            cv2SecondDriver = neckLocators[1]
            cv2weight1Value = 0.5
            cv2weight2Value = 0.5

            # cv 4 values
            cv4FirstDriver = neckLocators[1]
            cv4weight1Value = 0.5
            cv4weight2Value = 0.5


        else:
            # cv 3
            mc.parentConstraint(neckLocators[0], neckLocators[-1], driverLocatorOffsets[3], mo=True)
            mc.parentConstraint(neckLocators[0], driverLocatorOffsets[3], e=True, w=0.5)
            mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[3], e=True, w=0.5)

            # cv 2 values
            cv2SecondDriver = neckLocators[-1]
            cv2weight1Value = 0.75
            cv2weight2Value = 0.25

            # cv 4 values
            cv4FirstDriver = neckLocators[0]
            cv4weight1Value = 0.25
            cv4weight2Value = 0.75




        # cv 2
        mc.parentConstraint(neckLocators[0], cv2SecondDriver, driverLocatorOffsets[2], mo=True)
        mc.parentConstraint(neckLocators[0], driverLocatorOffsets[2], e=True, w = cv2weight1Value)
        mc.parentConstraint(cv2SecondDriver, driverLocatorOffsets[2], e=True, w = cv2weight2Value)

        # cv 4
        mc.parentConstraint(cv4FirstDriver, neckLocators[-1], driverLocatorOffsets[4], mo=True)
        mc.parentConstraint(cv4FirstDriver, driverLocatorOffsets[4], e=True, w = cv4weight1Value)
        mc.parentConstraint(neckLocators[-1], driverLocatorOffsets[4], e=True, w = cv4weight2Value)


        # make IK handle

        neckIk = mc.ikHandle(n='{}_ikHandle'.format(self.prefix), sol='ikSplineSolver', sj = ikNeckJoints[0], ee = ikNeckJoints[-1],
                              c = self.neckCurve, ccv=0, parentCurve=0)[0]
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

        mc.connectAttr('{}.worldMatrix[0]'.format(headCtr.C), '{}.dWorldUpMatrixEnd'.format(neckIk))
        mc.connectAttr('{}.worldMatrix[0]'.format(neckBaseCtr.C), '{}.dWorldUpMatrix'.format(neckIk))


        # connect IK spine joints to deformation skeleton
        # Connect deformation joints to ik joints


        # connect head joint to head control
        mc.orientConstraint(headCtr.C, ikHeadJoint, mo=1)


        # make head orient groups

        neckFollowTargetsGrp = mc.group(n = '{}_FollowTargets'.format(self.prefix), em = 1)
        mc.parent(neckFollowTargetsGrp, self.rigmodule.partsGrp)


        globalFollowGrp = mc.group(n='{}_head_globalFollow'.format(self.prefix), em=True)
        chestFollowGrp = mc.group(n='{}_head_chestFollow'.format(self.prefix), em=True)

        mc.parent(globalFollowGrp, neckFollowTargetsGrp)
        mc.parent(chestFollowGrp, neckFollowTargetsGrp)

        mc.delete(mc.parentConstraint(neckHybridFKCtr.C, neckFollowTargetsGrp, mo=0))

        mc.parentConstraint(self.baseRig.global1Ctrl.C, globalFollowGrp, mo=1)
        mc.parentConstraint(self.neckBaseAttachGrp, chestFollowGrp, mo=1)


        # setup head orientation switch
        follow_attr = "Follow_Chest"
        #mc.addAttr(headCtr.C, ln=attr, at='enum', enumName='neck:chest:world', k=1)
        mc.addAttr(headCtr.C, ln = follow_attr, at='double', min=0, max=1, dv=0, k=1)
        #mc.setAttr('{}.{}'.format(headCtr.C, follow_attr), cb=1)
        mc.setAttr('{}.{}'.format(headCtr.C, follow_attr), 1)
        self.HeadFollow = '{}.{}'.format(headCtr.C, follow_attr)

        # orient constraint head control

        #neckSpaceDriver = neckOrientGrp
        #chestSpaceDriver = chestOrientGrp
        #globalSpaceDriver = globalOrientGrp

        headFollowConstraint = mc.parentConstraint(globalFollowGrp, chestFollowGrp, neckHybridFKCtr.Off, mo=1)[0]
        mc.setAttr('{}.interpType'.format(headFollowConstraint), 2)
        weights = mc.parentConstraint(headFollowConstraint, q=1, weightAliasList = 1)

        mc.connectAttr('{}.{}'.format(headCtr.C , follow_attr), '{}.{}'.format(headFollowConstraint, weights[1]))

        # Make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_headFollow_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(headCtr.C , follow_attr), '{}.inputX'.format(reverse))
        mc.connectAttr('{}.outputX'.format(reverse),  '{}.{}'.format(headFollowConstraint, weights[0]))

        '''# set driven key for neck orient
        mc.setAttr('{}.{}'.format(headCtr.C, attr), 0)
        mc.setAttr( '{}.{}'.format(headOrientConstraint, weights[0]), 1)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 0)
        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd='{}.{}'.format(headCtr.C, attr))

        # set driven key for chest orient
        mc.setAttr('{}.{}'.format(headCtr.C, attr), 1)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[0]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 1)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 0)
        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd = '{}.{}'.format(headCtr.C, attr))

        # set driven key for global orient
        mc.setAttr('{}.{}'.format(headCtr.C, attr), 2)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[0]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[1]), 0)
        mc.setAttr('{}.{}'.format(headOrientConstraint, weights[2]), 1)
        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(headOrientConstraint, weights[i]), cd='{}.{}'.format(headCtr.C, attr))'''

        # set default to follow chest
        mc.setAttr('{}.{}'.format(headCtr.C, follow_attr),1)
        



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
        mc.addAttr(headCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)
        self.StretchyAttr = '{}.{}'.format( headCtr.C, stretch_attr)

        # make blender node for stretchy spine
        blenderStretch = mc.shadingNode('blendColors', asUtility=True,
                                        n='{}_blenderNeckStretch'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(headCtr.C, stretch_attr), '{}.blender'.format(blenderStretch))
        mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.color1.color1R'.format(blenderStretch))
        mc.setAttr('{}.color2.color2R'.format(blenderStretch), 1)

        # Connect output of stretchRatio with all spine joints scaleY except last
        for jnt in ikNeckJoints[:-1]:
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
        for jnt in ikNeckJoints[1:-1]:
            mc.connectAttr('{}.outputG'.format(blenderStretch), '{}.{}'.format(jnt, squashAxis1))
            mc.connectAttr('{}.outputB'.format(blenderStretch), '{}.{}'.format(jnt, squashAxis2))




        # rigParts dictionary
        self.rigParts['headCtr'] = headCtr


        return {'joints': ikJoints, 'controls': controls}

    def setInitialValues(self,
                         Stretchy = 1,
                         HeadFollow = 1 ,
                         HeadFKOrient = 0
                         ):

        mc.setAttr(self.StretchyAttr, Stretchy)
        mc.setAttr(self.HeadFollow, HeadFollow)
        mc.setAttr(self.HeadFKOrient, HeadFKOrient)
