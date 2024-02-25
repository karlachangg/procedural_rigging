"""
leg FK/IK @ rig
"""

import maya.cmds as mc

from . import bendyLimb

from ..base import module
from ..base import control

from ..utils import joint

class Leg():

    def __init__(self,
            type,
            legJoints,
            hipPivotJoint,
            prefix = 'leg',
            side = 'l',
            kneeDirection = 'z',
            forwardAxis = 'x',
            moveSwitchCtr = 'x, y',
            bendy = True,
            rigScale = 1.0,
            baseRig = None,

            ):
        """
        :param legJoints: list(str), hip - knee - ankle
        :param hipPivotJoint: str, hip position joint
        :param heelLoc: str, heel position locator
        :param innerLoc: str, inner rock position locator
        :param outerLoc: str, outer rock position locator
        :param prefix: str, prefix to name new objects
        :param side: str, left of right side indicator. Default 'l'
        :param kneeDirection: str, local axis of knee pole vector direction. Default 'y'
        :param forwardAxis: str, axis pointing down the joint chain. Default 'x'
        :param moveSwitchCtr: str, axes along which to translate the switch control. Default 'x, y'
        :param bendy: bool, option to build bendy limb controls
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """
        self.type = type
        self.legJoints = []

        for jnt in legJoints:
            newJnt = side + '_' + jnt
            self.legJoints.append(newJnt)


        self.hipPivotJoint = side + '_' + hipPivotJoint
        self.prefix = side + '_' + prefix
        self.side = side
        self.kneeDirection = kneeDirection
        self.forwardAxis = forwardAxis
        self.moveSwitchCtr = moveSwitchCtr
        self.rigScale = rigScale
        self.baseRig = baseRig
        self.bendy = bendy

        # make rig module

        self.rigmodule = module.Module(prefix=self.prefix, baseObj=self.baseRig)

        # make attach groups
        #bodyAttachGrp = mc.group(n = '{}_BodyAttach_grp'.format(self.prefix), em=1, p = self.rigmodule.partsGrp)

        self.rigParts = {'bodyAttachGrp': '',
                         'fkControls': '',
                         'switchControl': '',
                         'FKIKSwitchAttr': '',
                         'ikHandle': '',
                         'ikControl': '',
                         'ikJoints': '',
                         'fkJoints': '',
                         'pvControl': '',
                         'reverseFootDriven': '',
                         'bendControls': ''
                         }

    def build(self):


        # Make FK rig
        fkRig = self.buildFK()

        # Define rigParts properties
        self.rigParts['fkControls'] = fkRig['controls']
        self.rigParts['fkJoints'] = fkRig['joints']

        # Make IK rig
        if self.type == '2bones':
            ikRig = self.buildIK()
        elif self.type == '3bones':
            ikRig = self.buildQuadIK()

        self.rigParts['ikJoints'] = ikRig['joints']


        # Make switch control

        switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo= self.legJoints[1],
                                 scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='plus', color = 'green')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(switchCtr.C, ln = switch_attr, at= 'double', min = 0, max = 1, dv = 0, k = 1)

        control._rotateCtrlShape(switchCtr, axis='x', value=90)

        # Define rigParts properties
        self.rigParts['switchControl'] =switchCtr

        # Create class member so we can access later
        self.rigParts['FKIKSwitchAttr'] = '{}.{}'.format(switchCtr.C, switch_attr)


        # Connect deformation joints to fk and ik joints

        orientConstraints = []

        for i in range(len(self.legJoints)):
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], self.legJoints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            orientConstraints.append(oConstraint)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in orientConstraints:
            weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.{}'.format( constraint, weights[1] ) )
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))


        # Setup blend between joint scales

        for i in range(len(self.legJoints)):

            blendNode = mc.shadingNode('blendColors', asUtility=True,
                                       n='{}_jointScale_blend{}'.format(self.prefix, i))
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.blender'.format(blendNode))

            mc.connectAttr('{}.sx'.format(ikRig['joints'][i]), '{}.color1.color1R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(ikRig['joints'][i]), '{}.color1.color1G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(ikRig['joints'][i]), '{}.color1.color1B'.format(blendNode))

            mc.connectAttr('{}.sx'.format(fkRig['joints'][i]), '{}.color2.color2R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(fkRig['joints'][i]), '{}.color2.color2G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(fkRig['joints'][i]), '{}.color2.color2B'.format(blendNode))

            mc.connectAttr('{}.outputR'.format(blendNode), '{}.sx'.format(self.legJoints[i]))
            mc.connectAttr('{}.outputG'.format(blendNode), '{}.sy'.format(self.legJoints[i]))
            mc.connectAttr('{}.outputB'.format(blendNode), '{}.sz'.format(self.legJoints[i]))


        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format( ctrl.Off ) )
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
        mc.parent(orientconstraintGrp, self.baseRig.noXformGrp)

        # move switch ctr

        units = 3 * self.rigScale
        x, y, z = False, False, False

        if 'x' in self.moveSwitchCtr:
            x = True

        if 'y' in self.moveSwitchCtr:
            y = True

        if 'z' in self.moveSwitchCtr:
            z = True

        if '-' in self.moveSwitchCtr:
            units = units * -1


        mc.move(units, switchCtr.Off, x=x, y=y, z=z, os=1, r=1, wd=1)

        # Set initial state
        mc.setAttr('{}.{}'.format(switchCtr.C, switch_attr), 1 )

        if self.bendy:
            self.buildBendyLimbs()

        # Set some class properties we can call later to set a T pose
        self.fkControls = fkRig['controls']
        self.ikControls = ikRig['controls']
        self.pvControl = ikRig['controls'][1]


    def buildBendyLimbs(self):

        # Insert the bendy joints to our deformation skeleton
        '''upperLegJoints = joint.segmentJointchain(startJoint= self.legJoints[0],endJoint= self.legJoints[1], numberOfSegments = 4,
                                prefix= '{}Bend'.format(self.prefix), aimAxis='x' )'''

        if self.side == 'l':
            aimAxis = 'x'
            upAxis = 'z'
        elif self.side == 'r':
            aimAxis = '-x'
            upAxis = '-z'

        # Build the upper leg bendy rig
        upperLegBendy = bendyLimb.BendyLimb(
            startJoint=self.legJoints[0],
            endJoint=self.legJoints[1],
            numberOfBendyJoints=4,
            numberOfBendyControls=3,
            aimAxis= aimAxis,
            upAxis= upAxis,
            prefix= '{}_hip'.format(self.side) ,
            rigScale = self.rigScale,
            baseRig=self.baseRig)
        upperLegBendy.build()

        # Build the upper leg bendy rig
        lowerLegBendy = bendyLimb.BendyLimb(
            startJoint=self.legJoints[1],
            endJoint=self.legJoints[2],
            numberOfBendyJoints=4,
            numberOfBendyControls=3,
            aimAxis=aimAxis,
            upAxis=upAxis,
            prefix='{}_knee'.format(self.side),
            rigScale=self.rigScale,
            baseRig=self.baseRig)

        lowerLegBendy.build()



        # Get the parent of the top leg joint
        legParentJnt = mc.listRelatives(self.legJoints[0], p = 1)

        # Move the result joints out of the hierarchy
        mc.parent(self.legJoints[0], self.rigmodule.jointsGrp)

        # Move the bendyjoints into the hierarchy
        mc.parent(upperLegBendy.bendyJoints[0], legParentJnt)

        # remove last bendy joint of upper leg
        mc.delete(upperLegBendy.bendyJoints[-1])
        upperLegBendy.bendyJoints.pop()

        # Move the lower leg bendy joints under the upperleg bendy joints
        mc.parent(lowerLegBendy.bendyJoints[0], upperLegBendy.bendyJoints[-1] )

        # Get the child of the lower leg joint
        lowerlegChildJnt = mc.listRelatives(self.legJoints[2], c=1)

        # Move the foot joint back
        mc.parent(lowerlegChildJnt, lowerLegBendy.bendyJoints[-1])

        # Save rigparts dictionary
        self.rigParts['bendControls'] = upperLegBendy.controls + lowerLegBendy.controls



    def buildFK(self):

        # duplicate leg joints to make FK joints
        fkJoints = joint.duplicateChain(self.legJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)
        # make controls

        hipCtr = control.Control(prefix = '{}_hip'.format(self.prefix), translateTo = fkJoints[0], rotateTo = fkJoints[0],
                                  scale = self.rigScale * 1.5, parent = self.rigmodule.controlsGrp, shape = 'circleX',
                                 lockChannels= ['t', 's', 'v'])
        kneeCtr = control.Control(prefix = '{}_knee'.format(self.prefix), translateTo = fkJoints[1], rotateTo = fkJoints[1],
                                  scale = self.rigScale, parent = hipCtr.C, shape = 'circleX',
                                  lockChannels= ['t', 's', 'v'])
        footCtr = control.Control(prefix = '{}_foot'.format(self.prefix), translateTo = fkJoints[2], rotateTo = fkJoints[2],
                                        scale = self.rigScale, parent = kneeCtr.C, shape = 'circleX',
                                  lockChannels= ['t', 's', 'v'])

        controls = [hipCtr, kneeCtr, footCtr]

        # connect controls

        mc.parentConstraint(hipCtr.C, fkJoints[0], mo = 0 )
        mc.parentConstraint(kneeCtr.C, fkJoints[1], mo=0)
        mc.parentConstraint(footCtr.C, fkJoints[2], mo=0)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, hipCtr.Off, mo=1)[0]
        constraintGrp = mc.group(constraint, n= '{}_fk_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)



        return {'joints' : fkJoints, 'controls' : controls }






    def buildIK(self):

        # duplicate leg joints to make IK joints
        ikJoints = joint.duplicateChain(self.legJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)


        # Make controls
        controls = []
        legCtr = control.Control(prefix='{}_leg_ik'.format(self.prefix), translateTo=ikJoints[2],
                                  scale=self.rigScale, parent = self.rigmodule.controlsGrp, shape='cube')
        poleVectorCtr = control.Control(prefix='{}_leg_pv'.format(self.prefix), translateTo=ikJoints[1], rotateTo = ikJoints[2],
                                 scale=self.rigScale * 0.5, parent = self.rigmodule.controlsGrp, shape='locator')

        controls = [legCtr, poleVectorCtr]



        # ikEndGrp moves the IK handle and objects that are meant to be the "end" of the IK. Follows the IK control, but can be made to follow a reverse foot setup later.
        ikEndGrp = mc.group(n='{}_ikEndEffectors'.format(self.prefix), em=1)
        mc.parentConstraint(legCtr.C, ikEndGrp, mo=0)
        mc.parent(ikEndGrp, self.rigmodule.partsGrp)

        # Add followArmIK to rigParts dictionary bc we will need it in case of a reverse foot setup

        self.rigParts['reverseFootDriven'] = ikEndGrp
        self.rigParts['ikControl'] = legCtr
        self.rigParts['pvControl'] = poleVectorCtr

        # move pole vector ctr
        units = 5

        if 'x' in self.kneeDirection:
            x, y, z = True, False, False

        elif 'y' in self.kneeDirection:
            x, y, z = False, True, False

        elif 'z' in self.kneeDirection:
            x, y, z = False, False, True

        if '-' in self.kneeDirection:
            units = units * -1


        mc.move(units, poleVectorCtr.Off, x=x, y=y, z=z, os=1)

        # make IK handle
        legIK = mc.ikHandle(n='{}_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        mc.hide(legIK)
        mc.parent(legIK, ikEndGrp)


        # create no follow PV locator
        pvNoFollow = mc.spaceLocator(n = '{}_pv_noFollow'.format(self.prefix))
        mc.parent(pvNoFollow, self.rigmodule.partsGrp)
        mc.matchTransform(pvNoFollow, poleVectorCtr.C)

        # create PV follow object and setup
        poleAimLeg = mc.group(n = '{}_poleAim'.format(self.prefix), em = 1)
        upVector = mc.group(n = '{}_pv_upVec'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(legCtr.C, upVector, mo=0))
        mc.parent(upVector, self.rigmodule.partsGrp)
        #mc.parentConstraint(legCtr.C, upVector, sr = 'x', mo = 1)
        mc.parentConstraint(legCtr.C, upVector, mo=1)
        mc.pointConstraint(ikJoints[0], poleAimLeg, mo=0)

        # The pole aim axis will depend on the orientation of the foot control.. it should be the axis pointing out,
        # not in the direction of the IK bend. But it will change if we orient the end ctr to the bone or to world
        # TO DO: set up pole axis, and aim constraint up and aim vectors in a smarter way. Its hard coded right now

        poleAimAxis = 'y'

        poleAimConstraint = mc.aimConstraint(legCtr.C, poleAimLeg, aimVector = (0,1,0), upVector = (1,0,0),
                         worldUpType = 'objectRotation', worldUpVector = (1, 0, 0), worldUpObject = upVector,  mo=0)[0]

        poleOffsetFollow_noScale = mc.group(n = '{}_poleOffsetFollow_noScale'.format(self.prefix), em = 1)
        mc.pointConstraint(poleAimLeg, poleOffsetFollow_noScale , mo=0)
        mc.orientConstraint(poleAimLeg, poleOffsetFollow_noScale, mo=0)
        poleOffsetFollow = mc.group(n = '{}_poleOffsetFollow'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(poleVectorCtr.C, poleOffsetFollow, mo = 0))
        mc.parentConstraint( poleOffsetFollow_noScale, poleOffsetFollow, mo = 1 )
        mc.parent(poleAimLeg, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow_noScale, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow, self.rigmodule.partsGrp)

        # constrain pv
        pv_constraint = mc.parentConstraint(poleOffsetFollow, pvNoFollow, poleVectorCtr.Off, mo = 1)[0]
        weights = mc.parentConstraint(pv_constraint, q=1, weightAliasList=1)
        # setup pv follow switch
        pv_follow_attr = 'Follow'
        mc.addAttr(poleVectorCtr.C, ln = pv_follow_attr, at = 'double', min=0, max=1, dv=1, k=1)
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.{}'.format(pv_constraint, weights[0]))
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_pvFollow_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.inputX'.format(reverse))
        mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(pv_constraint, weights[1]))
        mc.parent(pv_constraint, self.rigmodule.noXformGrp)

        # attach objects to controls
        mc.poleVectorConstraint(poleVectorCtr.C, legIK)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, ikJoints[0], mo = 1 )[0]
        constraintGrp = mc.group(constraint, n='{}_ik_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)


        # Add Leg swivel attribute

        spin_attr = 'Spin'
        mc.addAttr(legCtr.C, ln = spin_attr, at='double', dv=0, k=1)

        self.elbowSpinAxis = self.forwardAxis

        if 'x' in poleAimAxis:
            spinAxisAttr = 'offsetX'
        elif 'y' in poleAimAxis:
            spinAxisAttr = 'offsetY'
        elif 'z' in poleAimAxis:
            spinAxisAttr = 'offsetZ'

        mc.connectAttr('{}.{}'.format(legCtr.C, spin_attr), '{}.{}'.format(poleAimConstraint, spinAxisAttr))



        # make pole vector connection line

        pvLinePos1 = mc.xform(ikJoints[1], q=1, t=1, ws=1)
        pvLinePos2 = mc.xform(poleVectorCtr.C, q=1, t=1, ws=1)
        poleVectorCurve = mc.curve(n='{}_pv_curve'.format(self.prefix), d=1, p=[pvLinePos1, pvLinePos2])

        mc.cluster('{}.cv[0]'.format(poleVectorCurve), n='{}_pv1_cls'.format(self.prefix), wn=[ikJoints[1], ikJoints[1]],
                   bs=True)
        mc.cluster('{}.cv[1]'.format(poleVectorCurve), n='{}_pv2_cls'.format(self.prefix),
                   wn=[poleVectorCtr.C, poleVectorCtr.C], bs=True)

        mc.parent(poleVectorCurve, self.rigmodule.controlsGrp)
        mc.setAttr('{}.template'.format(poleVectorCurve), 1)
        mc.setAttr('{}.it'.format(poleVectorCurve), 0)






        # make stretchy leg

        # Create group to follow hip location
        followHip = mc.group(n = '{}_IKHipFollow'.format(self.prefix), em = 1)
        mc.parentConstraint(self.hipPivotJoint,followHip, mo = 0 )
        mc.parent(followHip, self.rigmodule.partsGrp)

        # Create group to follow foot IK control
        followFoot = mc.group(n = '{}_IKFootFollow'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(legCtr.C, followFoot, mo=0))
        # create empty group under ik leg ctr
        followLegIK = mc.group(n='{}_IKLegFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(legCtr.C, followLegIK, mo=0))
        mc.parent(followLegIK, ikEndGrp)
        # parent constrain followFoot group to group under ik leg ctr
        mc.parentConstraint(followLegIK, followFoot, mo=0)
        mc.parent(followFoot, self.rigmodule.partsGrp)

        # create a distance node to get the length between the two groups
        leg_dist = mc.shadingNode('distanceBetween', asUtility=True, n='{}_leg_length'.format(self.prefix))
        # connect distance node to pv ctr world matrix and group positioned at wrist
        mc.connectAttr('{}.worldMatrix'.format(followHip), '{}.inMatrix1'.format(leg_dist))
        mc.connectAttr('{}.worldMatrix'.format(followFoot), '{}.inMatrix2'.format(leg_dist))
        # divide leg length by global scale
        leg_dist_global = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_leg_distance_global'.format(self.prefix))
        mc.setAttr('{}.operation'.format(leg_dist_global), 2)
        mc.connectAttr('{}.distance'.format(leg_dist), '{}.input1X'.format(leg_dist_global))
        mc.connectAttr('{}.sx'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(leg_dist_global))

        # Negate leg distance value for right side
        if '-' in self.forwardAxis:
            leg_dist_negative = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_leg_distance_negative'.format(self.prefix))
            mc.setAttr('{}.operation'.format(leg_dist_negative), 1)
            mc.connectAttr('{}.outputX'.format(leg_dist_global), '{}.input1X'.format(leg_dist_negative))
            mc.setAttr('{}.input2X'.format(leg_dist_negative), -1)
            sdkDriver = '{}.outputX'.format(leg_dist_negative)
        else:
            sdkDriver = '{}.outputX'.format(leg_dist_global)

        # Get the original length of the upper and lower leg joints

        if 'x' in self.forwardAxis:
            lengthAxisAttr = 'tx'
            scaleAxisAttr = 'sx'
        elif 'y' in self.forwardAxis:
            lengthAxisAttr = 'ty'
            scaleAxisAttr = 'sy'
        elif 'z' in self.forwardAxis:
            lengthAxisAttr = 'tz'
            scaleAxisAttr = 'sz'

        upperLeg_length = mc.getAttr('{}.{}'.format(ikJoints[1], lengthAxisAttr))
        lowerLeg_length = mc.getAttr('{}.{}'.format(ikJoints[2], lengthAxisAttr))

        # Calculate the length of fully extended leg
        legLength = upperLeg_length + lowerLeg_length

        # Create blender for stretchy leg setup
        stretch_attr = 'Stretchy'
        mc.addAttr(legCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # Create class member so we can access later
        self.StretchyAttr = '{}.{}'.format(legCtr.C, stretch_attr)

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        mc.addAttr(legCtr.C, ln=length1_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(legCtr.C, ln=length2_attr, at='double', min=1, dv=1, k=1)

        if '-' in self.forwardAxis:
            fAxisDirection = 0
        else:
            fAxisDirection = 1

        blenderStretchUpper = boneStretch(prefix='{}_bone1'.format(self.prefix),
                                             totalLimbLength = legLength,
                                             lengthAttr='{}.{}'.format(legCtr.C, length1_attr),
                                             stretchAttr=self.StretchyAttr,
                                             stretchDriver=sdkDriver,
                                             forwardAxisPositive=fAxisDirection
                                             )

        blenderStretchLower = boneStretch(prefix='{}_bone2'.format(self.prefix),
                                             totalLimbLength= legLength,
                                             lengthAttr='{}.{}'.format(legCtr.C, length2_attr),
                                             stretchAttr=self.StretchyAttr,
                                             stretchDriver=sdkDriver,
                                             forwardAxisPositive=fAxisDirection
                                             )



        # make knee pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        blenderPinUpper = poleVectorPin(prefix='{}_bone1'.format(self.prefix),
                                        pinAttr='{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl=poleVectorCtr.C,
                                        boneLocator = followHip,
                                        boneOrigLength= upperLeg_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive = fAxisDirection
                                        )

        blenderPinLower = poleVectorPin(prefix='{}_bone2'.format(self.prefix),
                                        pinAttr='{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl=poleVectorCtr.C,
                                        boneLocator = followFoot,
                                        boneOrigLength = lowerLeg_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive = fAxisDirection
                                        )

        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderStretchUpper), '{}.input[0]'.format(blenderPinUpper))
        mc.connectAttr('{}.output'.format(blenderStretchLower), '{}.input[0]'.format(blenderPinLower))


        # connect blender outputs to ik joints' translate X
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.{}'.format(ikJoints[0], scaleAxisAttr))
        mc.connectAttr('{}.output'.format(blenderPinLower), '{}.{}'.format(ikJoints[1], scaleAxisAttr))

        mc.setAttr('{}.{}'.format(legCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)


        return {'joints' : ikJoints, 'controls' : controls, 'poleVecLine': poleVectorCurve}





    def setInitialValues(self,
                         FKIKMode = 1,
                         Stretchy = 1
                         ):

        mc.setAttr(self.rigParts['FKIKSwitchAttr'], FKIKMode)
        mc.setAttr(self.StretchyAttr, Stretchy)



def boneStretch( prefix, totalLimbLength, lengthAttr, stretchAttr, stretchDriver, forwardAxisPositive):
    '''

    :param prefix: prefix to name nodes created
    :param totalLimbLength: sum of bone lengths making up the limb
    :param lengthAttr: attribute to increase bone length manually
    :param stretchBlender: attribute to blend between stretch and non-stretch bone length
    :param stretchDriver: will drive our SDK, its the distance between the shoulder and end control
    :param forwardAxisPositive: (bool) true if foward axis is positive, false if negative
    :return: stretch/no stretch blender node
    '''


    # make blender node to switch between stretch and non-stretched amount
    blenderStretch = mc.shadingNode('blendTwoAttr', asUtility=True,
                                            n='{}_blender_stretch'.format(prefix))
    mc.connectAttr(stretchAttr, '{}.attributesBlender'.format(blenderStretch))

    # Multiply stretch amount by length multiplier
    length_stretch_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length_mult'.format(prefix))
    mc.setAttr('{}.operation'.format(length_stretch_mult), 1)
    mc.connectAttr(lengthAttr, '{}.input1X'.format(length_stretch_mult))

    # Have leg stretchy SDK feed into this multDiv node
    sdkDriven = '{}.input2X'.format(length_stretch_mult)


    # Feed result of multDiv node to blender stretch value
    mc.connectAttr('{}.outputX'.format(length_stretch_mult), '{}.input[1]'.format(blenderStretch))

    # Feed result of multDiv node to blender non-stretch value
    mc.connectAttr(lengthAttr, '{}.input[0]'.format(blenderStretch))


    # Set up SDK to stretch arm after it's extended beyond its length

    # upper leg
    mc.setDrivenKeyframe(sdkDriven,
                         currentDriver = stretchDriver,
                         driverValue = totalLimbLength,
                         value = 1.0,
                         inTangentType = 'linear',
                         outTangentType = 'linear')

    mc.setDrivenKeyframe(sdkDriven,
                         currentDriver = stretchDriver,
                         driverValue = totalLimbLength * 2,
                         value = 1.0 * 2,
                         inTangentType='spline',
                         outTangentType='spline')

    animCurve = mc.keyframe(sdkDriven, query=True, name=True)[0]

    if forwardAxisPositive:
        mc.setAttr('{}.postInfinity'.format(animCurve), 1)

    else:
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)

    return blenderStretch


def poleVectorPin( prefix, pinAttr, pvCtrl, boneLocator, boneOrigLength, globalCtrl, forwardAxisPositive):

    """

    :param prefix: prefix to name nodes created
    :param pinAttr: (str) object.attribute of Pin switch
    :param pvCtrl: (str) name of pole vector control
    :param boneLocator: (str) name of locator positioned at the bone to pin
    :param boneOrigLength: (float) value of bone length before stretching
    :param globalCtrl: (str) name.attr of global scaling control
    :param forwardAxisPositive: (bool) true if foward axis is positive, false if negative
    :return: pin/no pin blender node
    """

    # make blender node which will blend between pin and no pin values
    blenderPin = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPin'.format(prefix))
    mc.connectAttr(pinAttr, '{}.attributesBlender'.format(blenderPin))

    # get distance between boneLocator and PV control
    bone_pv_dist = mc.shadingNode('distanceBetween', asUtility=True,
                                      n='{}_bone_2_pv_distance'.format(prefix))
    # connect distance node to shoulder location and pv ctr world matrix
    mc.connectAttr('{}.worldMatrix'.format(boneLocator), '{}.inMatrix1'.format(bone_pv_dist))
    mc.connectAttr('{}.worldMatrix'.format(pvCtrl), '{}.inMatrix2'.format(bone_pv_dist))

    # make multiply divide node to take into account global scale
    boneGlobalLength = mc.shadingNode('multiplyDivide', asUtility=True,
                                       n='{}_boneGlobalLength'.format(prefix))
    mc.setAttr('{}.operation'.format(boneGlobalLength), 2)
    mc.connectAttr('{}.distance'.format(bone_pv_dist), '{}.input1X'.format(boneGlobalLength))
    mc.connectAttr(globalCtrl, '{}.input2X'.format(boneGlobalLength))


    # divide new limb length by original limb length
    boneLengthScale = mc.shadingNode('multiplyDivide', asUtility=True,
                                      n='{}_boneLengthScale'.format(prefix))
    mc.setAttr('{}.operation'.format(boneLengthScale), 2)

    if not forwardAxisPositive:
        boneOrigLength = boneOrigLength * (-1)

    mc.setAttr('{}.input2X'.format(boneLengthScale), boneOrigLength)

    # feed new limb length value to boneLengthScale node
    mc.connectAttr('{}.outputX'.format(boneGlobalLength), '{}.input1X'.format(boneLengthScale))
    # connect distance from shoulder to pv ctr to blender input
    mc.connectAttr('{}.outputX'.format(boneLengthScale), '{}.input[1]'.format(blenderPin))

    return blenderPin