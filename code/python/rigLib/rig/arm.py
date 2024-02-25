"""
arm FK/IK @ rig
"""

import maya.cmds as mc
from . import bendyLimb

from ..base import module
from ..base import control

from ..utils import joint

class Arm():

    def __init__(self,
            armJoints,
            scapulaJoint,
            prefix = 'arm',
            side = 'l',
            bendy = False,
            ikCtrOrient = 'bone',
            elbowDirection = '-z',
            forwardAxis = 'x',
            elbowSpinAxis = 'x',
            moveSwitchCtr = 'x, y',

            rigScale = 1.0,
            baseRig = None,
            ):
        """
        :param armJoints: list(str), shoulder - elbow - wrist
        :param scapulaJoint: str, scapula position joint
        :param prefix: str, prefix to name new objects. Default "arm"
        :param side: str, left of right side indicator. Default 'l'
        :param bendy: bool, option to build bendy limb controls. Default False
        :param ikCtrOrient: bool, option to orient the ik limb control to the bone or world. Default "bone"
        :param elbowDirection: str, local axis of elbow pole vector direction. Default '-z'
        :param forwardAxis: str, axis pointing down the joint chain. Default 'x'
        :param elbowSpinAxis: str, axis along which to spin the pole vector. Default 'x'
        :param moveSwitchCtr: str, axes along which to translate the switch control. Default 'x, y'

        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        """
        self.armJoints = []

        for jnt in armJoints:
            newJnt = side + '_' + jnt
            self.armJoints.append(newJnt)

        self.scapulaJoint = side + '_' + scapulaJoint
        self.prefix = side + '_' + prefix
        self.side = side
        self.bendy = bendy
        self.elbowDirection = elbowDirection
        self.forwardAxis = forwardAxis
        self.elbowSpinAxis = elbowSpinAxis
        self.moveSwitchCtr = moveSwitchCtr

        self.ikCtrOrient = ikCtrOrient
        self.rigScale = rigScale
        self.baseRig = baseRig


        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        self.rigParts = {'bodyAttachGrp': '',
                         'handAttachGrp': '',
                         'fkControls': '',
                         'switchControl': '',
                         'FKIKSwitchAttr': '',
                         'ikHandle': '',
                         'ikControl': '',
                         'ikGimbalControl': '',
                         'reverseFootDriven': '',
                         'ikJoints': '',
                         'fkJoints': '',
                         'scapulaControls': ''
                         }


    def build(self):

        # Make FK rig
        fkRig = self.buildFK()

        # Make IK rig
        ikRig = self.buildIK()

        # Define rigParts properties
        self.rigParts['fkControls'] = fkRig['controls']
        self.rigParts['fkJoints'] = fkRig['joints']
        self.rigParts['ikJoints'] = ikRig['joints']



        # Make switch control

        self.switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo=self.armJoints[1], color = 'green',
                                         scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='plus')
        switch_attr = 'FKIK_Switch'
        control._rotateCtrlShape(self.switchCtr, axis = 'x', value = 90)

        mc.addAttr(self.switchCtr.C, ln=switch_attr, at='double', min=0, max=1, dv=0, k=1)

        self.rigParts['switchControl'] = self.switchCtr
        # Create class member so we can access later
        self.rigParts['FKIKSwitchAttr'] = '{}.{}'.format(self.switchCtr.C, switch_attr)
        # Connect deformation joints to fk and ik joints


        orientConstraints = []

        for i in range(len(self.armJoints)):
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], self.armJoints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            orientConstraints.append(oConstraint)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in orientConstraints:
            weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))


        # Setup blend between joint scales

        for i in range(len(self.armJoints)):
            blendNode = mc.shadingNode('blendColors', asUtility=True,
                                       n='{}_jointScale_blend{}'.format(self.prefix, i))
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.blender'.format(blendNode))

            mc.connectAttr('{}.sx'.format(ikRig['joints'][i]), '{}.color1.color1R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(ikRig['joints'][i]), '{}.color1.color1G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(ikRig['joints'][i]), '{}.color1.color1B'.format(blendNode))

            mc.connectAttr('{}.sx'.format(fkRig['joints'][i]), '{}.color2.color2R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(fkRig['joints'][i]), '{}.color2.color2G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(fkRig['joints'][i]), '{}.color2.color2B'.format(blendNode))

            mc.connectAttr('{}.outputR'.format(blendNode), '{}.sx'.format(self.armJoints[i]))
            mc.connectAttr('{}.outputG'.format(blendNode), '{}.sy'.format(self.armJoints[i]))
            mc.connectAttr('{}.outputB'.format(blendNode), '{}.sz'.format(self.armJoints[i]))


        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
        mc.parent(orientconstraintGrp, self.baseRig.noXformGrp)

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


        # move switch ctr
        mc.move(units, self.switchCtr.Off, x = x, y = y, z = z, os = 1, r = 1, wd = 1)

        # build scapula rig
        scapRig = self.buildScapula()

        # connect FK and IK arm rigs to scapula
        mc.pointConstraint(scapRig['armAttach'], fkRig['controls'][0].Off, mo = 1)
        mc.orientConstraint(scapRig['armAttach'], fkRig['controls'][0].Off, mo = 1)
        mc.pointConstraint(scapRig['armAttach'], ikRig['baseAttachGrp'], mo = 1 )
        mc.orientConstraint(scapRig['armAttach'], ikRig['baseAttachGrp'], mo=1)

        # Create hand attach group
        handAttachGrp = mc.group(em=1, n = '{}_handAttachGrp'.format(self.prefix) )
        mc.parent(handAttachGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(self.armJoints[2], handAttachGrp, mo = 0)

        # return set group
        self.rigParts['bodyAttachGrp'] = scapRig['bodyAttach']
        self.rigParts['handAttachGrp'] = handAttachGrp
        
        # Set some class properties we can call later to set a T pose
        self.fkControls = fkRig['controls']
        self.pvControl = ikRig['controls'][1]
        self.ikRotateGrp = ikRig['rotateGrp']

        if self.bendy:
            self.buildBendyLimbs()

        # Create tpose/ a pose switch
        self.tPose(self.fkControls, self.pvControl, self.ikRotateGrp)

    def buildBendyLimbs(self):

        # Insert the bendy joints to our deformation skeleton

        if self.side == 'l':
            aimAxis = 'x'
            upAxis = 'y'
        elif self.side == 'r':
            aimAxis = '-x'
            upAxis = '-y'

        # Build the upper leg bendy rig
        upperArmBendy = bendyLimb.BendyLimb(
            startJoint=self.armJoints[0],
            endJoint=self.armJoints[1],
            numberOfBendyJoints=4,
            numberOfBendyControls=3,
            aimAxis= aimAxis,
            upAxis= upAxis,
            prefix= '{}_shoulder'.format(self.side) ,
            rigScale = self.rigScale,
            baseRig=self.baseRig)
        upperArmBendy.build()

        # Build the upper leg bendy rig
        lowerArmBendy = bendyLimb.BendyLimb(
            startJoint=self.armJoints[1],
            endJoint=self.armJoints[2],
            numberOfBendyJoints=4,
            numberOfBendyControls=3,
            aimAxis=aimAxis,
            upAxis=upAxis,
            prefix='{}_elbow'.format(self.side),
            rigScale=self.rigScale,
            baseRig=self.baseRig)

        lowerArmBendy.build()



        # Get the parent of the top leg joint
        armParentJnt = mc.listRelatives(self.armJoints[0], p = 1)

        # Get the children of the wrist joint
        wristChildren = mc.listRelatives(self.armJoints[2], c=1)

        # Move the result joints out of the hierarchy
        mc.parent(self.armJoints[0], self.rigmodule.jointsGrp)

        # Move the bendyjoints into the hierarchy
        mc.parent(upperArmBendy.bendyJoints[0], armParentJnt)

        # remove last bendy joint of upper leg
        mc.delete(upperArmBendy.bendyJoints[-1])
        upperArmBendy.bendyJoints.pop()

        # Move the lower arm bendy joints under the upper arm bendy joints
        mc.parent(lowerArmBendy.bendyJoints[0], upperArmBendy.bendyJoints[-1] )

        # Move the hand joints back
        for jnt in wristChildren:
            mc.parent(jnt, lowerArmBendy.bendyJoints[-1])

    def buildFK(self):

        # duplicate arm joints to make FK joints
        fkJoints = joint.duplicateChain(self.armJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)


        # make controls

        shoulderCtr = control.Control(prefix ='{}_shoulder'.format(self.prefix), translateTo = fkJoints[0], rotateTo = fkJoints[0],
                                scale=self.rigScale, parent = self.rigmodule.controlsGrp, shape = 'circleX',
                                lockChannels= ['t', 's', 'v'], offsets= ['null', 'zero', 'auto'])
        elbowCtr = control.Control(prefix ='{}_elbow'.format(self.prefix), translateTo = fkJoints[1], rotateTo = fkJoints[1],
                                  scale=self.rigScale, parent = shoulderCtr.C, shape='circleX', lockChannels= ['t', 's', 'v'])
        wristCtr = control.Control(prefix = '{}_wrist'.format(self.prefix), translateTo = fkJoints[2], rotateTo = fkJoints[2],
                                  scale = self.rigScale, parent = elbowCtr.C, shape = 'circleX', lockChannels= ['t', 's', 'v'])
        controls = [shoulderCtr, elbowCtr, wristCtr]


        # connect controls

        mc.parentConstraint(shoulderCtr.C, fkJoints[0], mo=0)
        mc.parentConstraint(elbowCtr.C, fkJoints[1], mo=0)
        mc.parentConstraint(wristCtr.C, fkJoints[2], mo=0)


        return {'joints': fkJoints, 'controls': controls}

    def buildIK(self):

        # duplicate arm joints to make IK joints
        ikJoints = joint.duplicateChain(self.armJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)


        # Make controls
        if self.ikCtrOrient == 'bone':
            orientation = ikJoints[2]

        elif self.ikCtrOrient == 'world':
            orientation = self.baseRig.global1Ctrl

        armCtr = control.Control(prefix='{}_arm_ik'.format(self.prefix), translateTo=ikJoints[2], rotateTo= orientation,
                                 offsets= ['null', 'zero', 'auto'],
                                 scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape='cube')

        poleVectorCtr = control.Control(prefix='{}_arm_pv'.format(self.prefix), translateTo = ikJoints[1],
                                        rotateTo = ikJoints[1], offsets= ['null', 'zero', 'auto'],
                                        scale=self.rigScale * 0.25, parent=self.rigmodule.controlsGrp, shape='locator')

        wristGimbalCtr = control.Control(prefix='{}_hand_gimbal'.format(self.prefix), translateTo = ikJoints[2], rotateTo= ikJoints[2],
                                        scale =self.rigScale , lockChannels= ['t', 's'], shape='circle')

        controls = [armCtr, poleVectorCtr, wristGimbalCtr]

        self.rigParts['ikControl'] = armCtr
        self.rigParts['ikControls'] = controls
        self.rigParts['ikGimbalControl'] = wristGimbalCtr

        shoulderAttachGrp = mc.group( n = '{}_ikShoulderJnt_driver'.format(self.prefix), em=1 , p = self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], shoulderAttachGrp, mo = 0))

        # ikEndGrp moves the IK handle and objects that are meant to be the "end" of the IK. Follows the IK control, but can be used to follow a reverse foot setup later.
        ikEndGrp = mc.group(n='{}_ikEndEffectors'.format(self.prefix), em=1)
        mc.parentConstraint(armCtr.C, ikEndGrp, mo=0)
        mc.parent(ikEndGrp, self.rigmodule.partsGrp)

        # Add followArmIK to rigParts dictionary bc we will need it in case of a reverse foot setup
        self.rigParts['reverseFootDriven'] = ikEndGrp


        # move pole vector ctr
        units = 5 * self.rigScale

        if self.elbowDirection == 'x' or self.elbowDirection == '-x':
            x, y, z = True, False, False

        elif self.elbowDirection == 'y' or self.elbowDirection == '-y':
            x, y, z = False, True, False

        elif self.elbowDirection == 'z' or self.elbowDirection == '-z':
            x, y, z = False, False, True

        if '-' in self.elbowDirection:
            units = units * -1

        mc.move(units, poleVectorCtr.Off, x = x, y = y, z = z, os=1, r = 1, wd = 1)

        # make IK handle
        armIK = mc.ikHandle(n='{}_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        self.rigParts['ikHandle'] = armIK
        mc.hide(armIK)

        # Parent ikhandle to follow End group
        mc.parent(armIK, ikEndGrp)
        
        # make pole vector connection line
        pvLinePos1 = mc.xform(ikJoints[1], q=1, t=1, ws=1)
        pvLinePos2 = mc.xform(poleVectorCtr.C, q=1, t=1, ws=1)
        poleVectorCurve = mc.curve(n='{}_pv_curve'.format(self.prefix), d=1, p=[pvLinePos1, pvLinePos2])

        mc.cluster('{}.cv[0]'.format(poleVectorCurve), n='{}_pv1_cls'.format(self.prefix),
                   wn=[ikJoints[1], ikJoints[1]], bs=True)

        mc.cluster('{}.cv[1]'.format(poleVectorCurve), n='{}_pv2_cls'.format(self.prefix),
                   wn=[poleVectorCtr.C, poleVectorCtr.C], bs=True)

        mc.parent(poleVectorCurve, self.rigmodule.controlsGrp)
        mc.setAttr('{}.template'.format(poleVectorCurve), 1)
        mc.setAttr('{}.it'.format(poleVectorCurve), 0)


        # create no follow PV locator
        pvNoFollow = mc.spaceLocator(n = '{}_pv_noFollow'.format(self.prefix))
        mc.parent(pvNoFollow, self.rigmodule.partsGrp)
        mc.matchTransform(pvNoFollow, poleVectorCtr.C)

        # create follow pole vector  setup
        poleOffsetFollow_noScale_offset = mc.group(n='{}_poleOffsetFollow_noScale_offset'.format(self.prefix), em=1)
        poleOffsetFollow_noScale = mc.group(n='{}_poleOffsetFollow_noScale'.format(self.prefix), em=1)
        mc.parent(poleOffsetFollow_noScale, poleOffsetFollow_noScale_offset)
        mc.delete(mc.parentConstraint(armCtr.C, poleOffsetFollow_noScale_offset, mo = 0))
        poleFollowOrientConstraint = mc.orientConstraint(armCtr.C, poleOffsetFollow_noScale, mo = 1 )[0]
        poleFollowPointConstraint = mc.pointConstraint(armCtr.C, poleOffsetFollow_noScale, mo = 1 )[0]
        pvFollow = mc.group(n='{}_pv_Follow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(poleVectorCtr.C, pvFollow, mo=0))
        mc.parentConstraint(poleOffsetFollow_noScale, pvFollow, mo=1)
        mc.parent(poleOffsetFollow_noScale_offset, self.rigmodule.partsGrp)
        mc.parent(pvFollow, self.rigmodule.partsGrp)

        # constrain pv
        pv_constraint = mc.parentConstraint(pvFollow, pvNoFollow, poleVectorCtr.Offsets[0], mo = 1)[0]
        weights = mc.parentConstraint(pv_constraint, q=1, weightAliasList=1)
        mc.setAttr('{}.interpType'.format(pv_constraint), 2 )

        # setup pv follow switch
        pv_follow_attr = 'Follow'
        mc.addAttr(poleVectorCtr.C, ln = pv_follow_attr, at = 'double', min=0, max=1, dv=0, k=1)
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.{}'.format(pv_constraint, weights[0]))
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_pvFollow_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.inputX'.format(reverse))
        mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(pv_constraint, weights[1]))
        mc.parent(pv_constraint, self.rigmodule.noXformGrp)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), 1)

        # Add spin attribute

        spin_attr = 'Spin'
        mc.addAttr(armCtr.C, ln=spin_attr, at='double', dv=0, k=1)


        if self.elbowSpinAxis == 'x' or self.elbowSpinAxis == '-x':
            spinAxisAttr = 'offsetX'
        elif self.elbowSpinAxis == 'y' or self.elbowSpinAxis == '-y':
            spinAxisAttr = 'offsetY'
        elif self.elbowSpinAxis == 'z' or self.elbowSpinAxis == '-z':
            spinAxisAttr = 'offsetZ'

        mc.connectAttr('{}.{}'.format(armCtr.C, spin_attr), '{}.{}'.format(poleFollowOrientConstraint, spinAxisAttr))


        # attach objects to controls
        mc.poleVectorConstraint(poleVectorCtr.C, armIK)
        mc.orientConstraint(wristGimbalCtr.C, ikJoints[2], mo=1)
        mc.parent(wristGimbalCtr.Off, armCtr.C)

        # attach to shoulderAttachGrp
        mc.pointConstraint(shoulderAttachGrp, ikJoints[0], mo = 1 )

        # make group to rotate whole IK setup, we'll use this to pose our arm in T Pose
        rotateAllGrp = mc.group(n = '{}_ik_rotateAll'.format(self.prefix), em = 1)
        mc.parent(rotateAllGrp, self.rigmodule.partsGrp)
        mc.matchTransform(rotateAllGrp, ikJoints[0])
        mc.parentConstraint(rotateAllGrp, armCtr.Off, mo = 1)
        mc.parentConstraint(rotateAllGrp, pvNoFollow, mo=1)
        mc.parentConstraint(rotateAllGrp, poleOffsetFollow_noScale_offset, mo=1)




        # make stretchy arm

        # Create group to follow shoulder location
        followClavGrp = mc.group(n='{}_IKClavicleFollow'.format(self.prefix), em=1)
        shoulderLoc = mc.group(n='{}_IKShoulderLoc'.format(self.prefix), em=1)
        mc.parent(shoulderLoc, followClavGrp)
        mc.parentConstraint(self.scapulaJoint, followClavGrp, mo=0)
        mc.delete(mc.parentConstraint(ikJoints[0], shoulderLoc, mo=0))
        mc.parent(followClavGrp, self.rigmodule.partsGrp)

        # Create group to follow hand IK control
        followWrist = mc.group(n='{}_IKWristFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, followWrist, mo=0))
        # create empty group under ik arm ctr
        followArmIK = mc.group(n='{}_IKArmFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, followArmIK, mo=0))
        mc.parent(followArmIK, ikEndGrp)
        # point constrain wrist group to group under ik arm ctr
        mc.parentConstraint(followArmIK, followWrist, mo=0)
        mc.parent(followWrist, self.rigmodule.partsGrp)


        # create a distance node to get the length between the two groups
        arm_dist = mc.shadingNode('distanceBetween', asUtility=True, n='{}_arm_length'.format(self.prefix))
        # connect distance node to pv ctr world matrix and group positioned at wrist
        mc.connectAttr('{}.worldMatrix'.format(shoulderLoc), '{}.inMatrix1'.format(arm_dist))
        mc.connectAttr('{}.worldMatrix'.format(followWrist), '{}.inMatrix2'.format(arm_dist))
        # divide arm length by global scale
        arm_dist_global = mc.shadingNode('multiplyDivide', asUtility=True,
                                         n='{}_arm_distance_global'.format(self.prefix))
        mc.setAttr('{}.operation'.format(arm_dist_global), 2)
        mc.connectAttr('{}.distance'.format(arm_dist), '{}.input1X'.format(arm_dist_global))
        mc.connectAttr('{}.sx'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(arm_dist_global))


        # Negate leg distance value for right side
        if '-' in self.forwardAxis:
            leg_dist_negative = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_leg_distance_negative'.format(self.prefix))
            mc.setAttr('{}.operation'.format(leg_dist_negative), 1)
            mc.connectAttr('{}.outputX'.format(arm_dist_global), '{}.input1X'.format(leg_dist_negative))
            mc.setAttr('{}.input2X'.format(leg_dist_negative), -1)
            sdkDriver = '{}.outputX'.format(leg_dist_negative)
        else:
            sdkDriver = '{}.outputX'.format(arm_dist_global)


        # Get the original length of the upper and lower arm joints

        if 'x' in self.forwardAxis:
            lengthAxisAttr = 'tx'
            scaleAxisAttr = 'sx'
        elif 'y' in self.forwardAxis:
            lengthAxisAttr = 'ty'
            scaleAxisAttr = 'sy'
        elif 'z' in self.forwardAxis:
            lengthAxisAttr = 'tz'
            scaleAxisAttr = 'sz'

        upperArm_length = mc.getAttr('{}.{}'.format(ikJoints[1], lengthAxisAttr))
        lowerArm_length = mc.getAttr('{}.{}'.format(ikJoints[2], lengthAxisAttr))

        # Calculate the length of fully extended arm
        armLength = upperArm_length + lowerArm_length

        # Create blender for stretchy arm setup
        stretch_attr = 'Stretchy'
        mc.addAttr(armCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # Create class member so we can access later
        self.StretchyAttr = '{}.{}'.format(armCtr.C, stretch_attr)

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        mc.addAttr(armCtr.C, ln=length1_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(armCtr.C, ln=length2_attr, at='double', min=1, dv=1, k=1)
        if '-' in self.forwardAxis:
            fAxisDirection = 0
        else:
            fAxisDirection = 1

        blenderStretchUpper = boneStretch(prefix='{}_bone1'.format(self.prefix),
                                          totalLimbLength=armLength,
                                          lengthAttr='{}.{}'.format(armCtr.C, length1_attr),
                                          stretchAttr=self.StretchyAttr,
                                          stretchDriver=sdkDriver,
                                          forwardAxisPositive=fAxisDirection
                                          )

        blenderStretchLower = boneStretch(prefix='{}_bone2'.format(self.prefix),
                                          totalLimbLength=armLength,
                                          lengthAttr='{}.{}'.format(armCtr.C, length2_attr),
                                          stretchAttr=self.StretchyAttr,
                                          stretchDriver=sdkDriver,
                                          forwardAxisPositive=fAxisDirection
                                          )


        # make elbow pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        blenderPinUpper = poleVectorPin(prefix='{}_bone1'.format(self.prefix),
                                        pinAttr='{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl=poleVectorCtr.C,
                                        boneLocator = shoulderLoc,
                                        boneOrigLength = upperArm_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive=fAxisDirection
                                        )

        blenderPinLower = poleVectorPin(prefix='{}_bone2'.format(self.prefix),
                                        pinAttr='{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl=poleVectorCtr.C,
                                        boneLocator = followWrist,
                                        boneOrigLength = lowerArm_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive=fAxisDirection
                                        )

        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderStretchUpper), '{}.input[0]'.format(blenderPinUpper))
        mc.connectAttr('{}.output'.format(blenderStretchLower), '{}.input[0]'.format(blenderPinLower))

        # connect blender outputs to ik joints' translate X
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.{}'.format(ikJoints[0], scaleAxisAttr))
        mc.connectAttr('{}.output'.format(blenderPinLower), '{}.{}'.format(ikJoints[1], scaleAxisAttr))

        mc.setAttr('{}.{}'.format(armCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)



        return {'joints': ikJoints, 'controls': controls, 'poleVecLine': poleVectorCurve, 'baseAttachGrp': shoulderAttachGrp, 'rotateGrp': rotateAllGrp}



    def buildScapula(self):

        # Make scapula control

        scapCtr = control.Control(prefix='{}_scapula'.format(self.prefix), translateTo= self.scapulaJoint,
                                  rotateTo = self.scapulaJoint, scale=self.rigScale, parent=self.rigmodule.controlsGrp,
                                  shape = 'quadArrow', offsets= ['null', 'zero', 'auto'])

        scapAimCtr = control.Control(prefix='{}_scapula_translate'.format(self.prefix), translateTo= self.armJoints[0], rotateTo= self.armJoints[0],
                                  scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape = 'squareY',
                                     lockChannels= ['r', 's'])

        self.rigParts['scapulaControls'] = [scapCtr, scapAimCtr]

        # setup aim constraint on scapCtr

        upVector = mc.group(n = '{}_aim_upVec'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(scapAimCtr.C, upVector, mo=0))
        mc.parent(upVector, scapAimCtr.C)

        mc.aimConstraint( scapAimCtr.C, scapCtr.Offsets[2], aimVector=(1, 0, 0), upVector=(0, 1, 0),
                         worldUpType='objectRotation', worldUpVector=(0, 1, 0), worldUpObject=upVector, mo = 1)

        # connect scapula joint to control
        constraint = mc.parentConstraint(scapCtr.C, self.scapulaJoint, mo = 1 )[0]
        mc.parent(constraint, self.rigmodule.noXformGrp)

        # make locator to follow end of clavicle
        endPos = mc.spaceLocator( n = '{}_scapEnd_pos'.format(self.prefix) )
        mc.parent(endPos, self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(self.armJoints[0], endPos))
        mc.parent(endPos, scapCtr.C)
        mc.hide(endPos)

        # make body attach group
        bodyAttachGrp = mc.group(n= '{}_scap_bodyAttachGrp'.format(self.prefix),em = 1, p = self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(self.scapulaJoint, bodyAttachGrp, mo = 0) )

        mc.parentConstraint( bodyAttachGrp, scapCtr.Off, mo = 1)
        mc.parentConstraint(bodyAttachGrp, scapAimCtr.Off, mo=1)

        return {'armAttach': endPos, 'bodyAttach': bodyAttachGrp}


    def tPose(self, fkControls, pvControl, ikRotateGrp):

        '''
        Set default animation pose to T pose
        '''

        mc.setAttr('{}.{}'.format(self.switchCtr.C, 'FKIK_Switch'), 0)



        # Set up locators to straighten shoulder

        # create locator to drive rotation, positioned at shoulder location & orientation
        loc = mc.spaceLocator(n='{}shoulder_Straighten'.format(self.prefix))[0]
        mc.delete(mc.parentConstraint(fkControls[0].C, loc, mo=0))

        # create locator at shoulder with world orientation
        shoulderWorld = mc.spaceLocator(n='{}_shoulder_world'.format(self.prefix))[0]
        mc.delete(mc.pointConstraint(fkControls[0].C, shoulderWorld, mo=0))

        # if right side, rotate world orientation holders so it mirrors left
        if self.side == 'r':
            mc.rotate('180deg', shoulderWorld, r=1, os=1, x=1)

        # store IK rotation values for A pose
        ik_aposeRx = mc.getAttr('{}.rx'.format(ikRotateGrp))
        ik_aposeRy = mc.getAttr('{}.ry'.format(ikRotateGrp))
        ik_aposeRz = mc.getAttr('{}.rz'.format(ikRotateGrp))

        # orient constrain shoulder control to locator
        shoulderCnstr = mc.orientConstraint(loc, fkControls[0].Offsets[1], mo=1)[0]
        ikCnstr = mc.orientConstraint(loc, ikRotateGrp, mo=1)[0]

        # orient pv control offset so it stays aligned to character global
        globalOrient = mc.group(n='{}_global_orient'.format(self.prefix), em=1)
        pvCnstr = mc.orientConstraint(globalOrient, pvControl.Offsets[1], mo=1)[0]

        # rotate locator to world orientation
        mc.orientConstraint(shoulderWorld, loc, mo=0)
        mc.delete([shoulderCnstr, ikCnstr])

        # store IK rotation values for T pose
        ik_tposeRx = mc.getAttr('{}.rx'.format(ikRotateGrp))
        ik_tposeRy = mc.getAttr('{}.ry'.format(ikRotateGrp))
        ik_tposeRz = mc.getAttr('{}.rz'.format(ikRotateGrp))

        pv_tposeRx = mc.getAttr('{}.rx'.format(pvControl.Offsets[1]))
        pv_tposeRy = mc.getAttr('{}.ry'.format(pvControl.Offsets[1]))
        pv_tposeRz = mc.getAttr('{}.rz'.format(pvControl.Offsets[1]))

        # setup arm orientation switch

        poseAttr = 'Default_Pose'
        mc.addAttr(self.switchCtr.C, ln=poseAttr, at='enum', enumName = 'T-Pose:A-Pose', k = 1)
        mc.setAttr('{}.{}'.format(self.switchCtr.C, poseAttr), cb=1)
        mc.setAttr('{}.{}'.format(self.switchCtr.C, poseAttr), 0)

        # Create class member so we can access later
        self.APose = '{}.{}'.format(self.switchCtr.C, poseAttr)

        fk_tposeRx = mc.getAttr('{}.rx'.format(fkControls[0].Offsets[1]))
        fk_tposeRy = mc.getAttr('{}.ry'.format(fkControls[0].Offsets[1]))
        fk_tposeRz = mc.getAttr('{}.rz'.format(fkControls[0].Offsets[1]))
        mc.delete([pvCnstr, globalOrient])

        # set driven key for t-pose
        mc.setAttr('{}.{}'.format(self.switchCtr.C, poseAttr), 0)

        mc.setAttr('{}.rx'.format(fkControls[0].Offsets[1]), fk_tposeRx)
        mc.setAttr('{}.ry'.format(fkControls[0].Offsets[1]), fk_tposeRy)
        mc.setAttr('{}.rz'.format(fkControls[0].Offsets[1]), fk_tposeRz)

        mc.setAttr('{}.rx'.format(pvControl.Offsets[1]), pv_tposeRx)
        mc.setAttr('{}.ry'.format(pvControl.Offsets[1]), pv_tposeRy )
        mc.setAttr('{}.rz'.format(pvControl.Offsets[1]), pv_tposeRz )

        mc.setAttr('{}.rx'.format(ikRotateGrp), ik_tposeRx)
        mc.setAttr('{}.ry'.format(ikRotateGrp), ik_tposeRy)
        mc.setAttr('{}.rz'.format(ikRotateGrp), ik_tposeRz)

        mc.setDrivenKeyframe('{}.rx'.format(fkControls[0].Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.ry'.format(fkControls[0].Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.rz'.format(fkControls[0].Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))

        mc.setDrivenKeyframe('{}.rx'.format(ikRotateGrp), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.ry'.format(ikRotateGrp), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.rz'.format(ikRotateGrp), cd='{}.{}'.format(self.switchCtr.C, poseAttr))

        mc.setDrivenKeyframe('{}.rx'.format(pvControl.Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.ry'.format(pvControl.Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.rz'.format(pvControl.Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))

        # set driven key for a-pose
        mc.setAttr('{}.{}'.format(self.switchCtr.C, poseAttr), 1)

        mc.setAttr('{}.rx'.format(fkControls[0].Offsets[1]), 0)
        mc.setAttr('{}.ry'.format(fkControls[0].Offsets[1]), 0)
        mc.setAttr('{}.rz'.format(fkControls[0].Offsets[1]), 0)

        mc.setAttr('{}.rx'.format(pvControl.Offsets[1]), 0)
        mc.setAttr('{}.ry'.format(pvControl.Offsets[1]), 0)
        mc.setAttr('{}.rz'.format(pvControl.Offsets[1]), 0)

        mc.setAttr('{}.rx'.format(ikRotateGrp), ik_aposeRx)
        mc.setAttr('{}.ry'.format(ikRotateGrp), ik_aposeRy)
        mc.setAttr('{}.rz'.format(ikRotateGrp), ik_aposeRz)


        mc.setDrivenKeyframe('{}.rx'.format(fkControls[0].Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.ry'.format(fkControls[0].Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.rz'.format(fkControls[0].Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))

        mc.setDrivenKeyframe('{}.rx'.format(ikRotateGrp), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.ry'.format(ikRotateGrp), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.rz'.format(ikRotateGrp), cd='{}.{}'.format(self.switchCtr.C, poseAttr))

        mc.setDrivenKeyframe('{}.rx'.format(pvControl.Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.ry'.format(pvControl.Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))
        mc.setDrivenKeyframe('{}.rz'.format(pvControl.Offsets[1]), cd='{}.{}'.format(self.switchCtr.C, poseAttr))

        mc.setAttr('{}.{}'.format(self.switchCtr.C, poseAttr), 1)


        # delete stuff
        mc.delete([loc, shoulderWorld])



    def setInitialValues(self,
                         FKIKMode = 1,
                         Stretchy = 1,
                         Apose = 1,
                         ):

        mc.setAttr(self.rigParts['FKIKSwitchAttr'], FKIKMode)
        mc.setAttr(self.StretchyAttr, Stretchy)
        mc.setAttr(self.APose, Apose)




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





