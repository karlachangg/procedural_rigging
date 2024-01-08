"""
arm FK/IK @ rig
"""

import maya.cmds as mc

from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name

class Arm():

    def __init__(self,
            armJoints,
            scapulaJoint,
            prefix = 'arm',
            side = 'l',
            rigScale = 1.0,
            baseRig = None
            ):
        """
        :param armJoints: list(str), shoulder - elbow - wrist
        :param scapulaJoint: str, scapula position joint
        :param prefix: str, prefix to name new objects
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """
        self.armJoints = []

        for jnt in armJoints:
            newJnt = side + '_' + jnt
            self.armJoints.append(newJnt)


        self.scapulaJoint = side + '_' + scapulaJoint
        self.prefix = side + '_' + prefix
        self.side = side
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        self.rigParts = {'bodyAttachGrp': '',
                         'handAttachGrp': ''
                         }


    def build(self):

        # Add twist joints
        elbowTwistJnt = joint.duplicateChain(self.armJoints[1], oldSuffix='jnt', newSuffix='twist_jnt')
        wristTwistJnt = joint.duplicateChain(self.armJoints[2], oldSuffix='jnt', newSuffix='twist_jnt')
        mc.parent(elbowTwistJnt, self.armJoints[0])
        mc.parent(wristTwistJnt, self.armJoints[1])



        # Make FK rig
        fkRig = self.buildFK()

        # Make IK rig
        ikRig = self.buildIK()

        # Connect deformation joints to fk and ik joints

        pointConstraints = []
        orientConstraints = []

        for i in range(len(self.armJoints)):
            pConstraint = mc.pointConstraint(fkRig['joints'][i], ikRig['joints'][i], self.armJoints[i], mo=0)[0]
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], self.armJoints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            pointConstraints.append(pConstraint)
            orientConstraints.append(oConstraint)

        # orient constrain twist joints
        elbowTwistConstr = mc.orientConstraint(fkRig['joints'][1], ikRig['joints'][1], elbowTwistJnt, mo=1, skip=['y', 'z'])[0]
        wristTwistConstr = mc.orientConstraint(fkRig['joints'][2], ikRig['joints'][2], wristTwistJnt, mo=1, skip=['y', 'z'])[0]
        mc.setAttr('{}.interpType'.format(elbowTwistConstr), 2)
        mc.setAttr('{}.interpType'.format(wristTwistConstr), 2)
        orientConstraints.append(elbowTwistConstr)
        orientConstraints.append(wristTwistConstr)

        # Make switch control

        self.switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo=self.armJoints[1],
                                         scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='sphere')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(self.switchCtr.C, ln=switch_attr, at='double', min=0, max=1, dv=0, k=1)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in pointConstraints:
            weights = mc.pointConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for constraint in orientConstraints:
            weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        pointconstraintGrp = mc.group(pointConstraints, n='defSkeleton_{}_pconstraints'.format(self.prefix))
        orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
        mc.parent(pointconstraintGrp, self.baseRig.noXformGrp)
        mc.parent(orientconstraintGrp, self.baseRig.noXformGrp)

        # move switch ctr
        if self.side == 'l':
            mc.move(3, self.switchCtr.Off, x = True, os = 1, r = 1, wd = 1)
            mc.move(3, self.switchCtr.Off, y = True, os = 1, r = 1, wd = 1)
        elif self.side == 'r':
            mc.move(-3, self.switchCtr.Off, x = True, os = 1, r = 1, wd = 1)
            mc.move(3, self.switchCtr.Off, y = True, os = 1, r = 1, wd = 1)

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
        
        # Set default animation pose to T pose

        self.tPose(fkControls = fkRig['controls'],
                   pvControl = ikRig['controls'][1],
                   ikRotateGrp= ikRig['rotateGrp']
                   )





    def buildFK(self):

        # duplicate arm joints to make FK joints
        fkJoints = joint.duplicateChain(self.armJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)

        # make controls

        shoulderCtr = control.Control(prefix ='{}_shoulder'.format(self.prefix), translateTo = fkJoints[0], rotateTo = fkJoints[0],
                                 scale=self.rigScale, parent = self.rigmodule.controlsGrp, shape = 'circleX',  offsets= ['null', 'zero', 'auto'])
        elbowCtr = control.Control(prefix ='{}_elbow'.format(self.prefix), translateTo = fkJoints[1], rotateTo = fkJoints[1],
                                  scale=self.rigScale, parent = shoulderCtr.C, shape='circleX')
        wristCtr = control.Control(prefix = '{}_wrist'.format(self.prefix), translateTo = fkJoints[2], rotateTo = fkJoints[2],
                                  scale = self.rigScale, parent = elbowCtr.C, shape = 'circleX')
        controls = [shoulderCtr, elbowCtr, wristCtr]

        # connect controls

        mc.parentConstraint(shoulderCtr.C, fkJoints[0], mo=0)
        mc.parentConstraint(elbowCtr.C, fkJoints[1], mo=0)
        mc.parentConstraint(wristCtr.C, fkJoints[2], mo=0)

        # attach to scapula

        return {'joints': fkJoints, 'controls': controls}

    def buildIK(self):

        # duplicate arm joints to make IK joints
        ikJoints = joint.duplicateChain(self.armJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)

        # Make controls

        armCtr = control.Control(prefix='{}_arm_ik'.format(self.prefix), translateTo=ikJoints[2], rotateTo= ikJoints[2],
                                 offsets= ['null', 'zero', 'auto'],
                                 scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape='square')
        poleVectorCtr = control.Control(prefix='{}_arm_pv'.format(self.prefix), translateTo = ikJoints[1], offsets= ['null', 'zero', 'auto'],
                                        scale=self.rigScale * 0.25, parent=self.rigmodule.controlsGrp, shape='orb')
        wristGimbalCtr = control.Control(prefix='{}_hand_gimbal'.format(self.prefix), translateTo = ikJoints[2], rotateTo= ikJoints[2],
                                        scale =self.rigScale * 0.5 , lockChannels= ['t', 's'], shape='circle')
        controls = [armCtr, poleVectorCtr, wristGimbalCtr]

        shoulderAttachGrp = mc.group( n = '{}_ikShoulderJnt_driver'.format(self.prefix), em=1 , p = self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], shoulderAttachGrp, mo = 0))

        # move pole vector ctr
        mc.move(-5, poleVectorCtr.Off, z=True, os=1)

        # make IK handle
        armIK = mc.ikHandle(n='{}_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        mc.hide(armIK)
        mc.parent(armIK, self.rigmodule.noXformGrp)
        
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
        mc.connectAttr('{}.{}'.format(armCtr.C, spin_attr), '{}.offsetX'.format(poleFollowOrientConstraint))


        # attach objects to controls
        mc.parentConstraint(armCtr.C, armIK, mo = 1)
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
        mc.parent(followArmIK, armCtr.C)
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
        if self.side == 'r':
            arm_dist_negative = mc.shadingNode('multiplyDivide', asUtility=True,
                                               n='{}_arm_distance_negative'.format(self.prefix))
            mc.setAttr('{}.operation'.format(arm_dist_negative), 1)
            mc.connectAttr('{}.outputX'.format(arm_dist_global), '{}.input1X'.format(arm_dist_negative))
            mc.setAttr('{}.input2X'.format(arm_dist_negative), -1)
            sdkDriver = '{}.outputX'.format(arm_dist_negative)

        if self.side == 'l':
            sdkDriver = '{}.outputX'.format(arm_dist_global)

        # Get the original length of the upper and lower arm joints
        upperArm_length = mc.getAttr('{}.tx'.format(ikJoints[1]))
        lowerArm_length = mc.getAttr('{}.tx'.format(ikJoints[2]))

        # Calculate the length of fully extended arm
        armLength = upperArm_length + lowerArm_length

        # Create blender for stretchy arm setup
        stretch_attr = 'Stretchy'
        mc.addAttr(armCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # make blender node for upper arm
        blenderStretchUpperArm = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderArmUpper_stretch'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(armCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderStretchUpperArm))

        # make blender node for lower arm
        blenderStretchLowerArm = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderArmLower_stretch'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(armCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderStretchLowerArm))

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        mc.addAttr(armCtr.C, ln=length1_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(armCtr.C, ln=length2_attr, at='double', min=1, dv=1, k=1)

        # Multiply stretchy leg by length
        length1_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length1_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(length1_mult), 1)
        mc.connectAttr('{}.{}'.format(armCtr.C, length1_attr), '{}.input1X'.format(length1_mult))

        length2_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length2_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(length2_mult), 1)
        mc.connectAttr('{}.{}'.format(armCtr.C, length2_attr), '{}.input1X'.format(length2_mult))

        # Have leg stretchy SDK feed into this multDiv node
        sdkDrivenUpper = '{}.input2X'.format(length1_mult)
        sdkDrivenLower = '{}.input2X'.format(length2_mult)

        # connect multDiv node to blender input
        mc.connectAttr('{}.outputX'.format(length1_mult), '{}.input[1]'.format(blenderStretchUpperArm))
        mc.connectAttr('{}.outputX'.format(length2_mult), '{}.input[1]'.format(blenderStretchLowerArm))

        # Multiply non stretchy leg by length attribute
        length1_noStretch_mult = mc.shadingNode('multiplyDivide', asUtility=True,
                                                    n='{}_length1_noStretch_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(length1_noStretch_mult), 1)
        mc.connectAttr('{}.{}'.format(armCtr.C, length1_attr), '{}.input1X'.format(length1_noStretch_mult))
        mc.setAttr('{}.input2X'.format(length1_noStretch_mult), upperArm_length)
        # connect output to blender input
        mc.connectAttr('{}.outputX'.format(length1_noStretch_mult), '{}.input[0]'.format(blenderStretchUpperArm))

        length2_noStretch_mult = mc.shadingNode('multiplyDivide', asUtility=True,
                                                    n='{}_length2_noStretch_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(length2_noStretch_mult), 1)
        mc.connectAttr('{}.{}'.format(armCtr.C, length2_attr), '{}.input1X'.format(length2_noStretch_mult))
        mc.setAttr('{}.input2X'.format(length2_noStretch_mult), lowerArm_length)
        # connect upper leg bone original length to blender input
        mc.connectAttr('{}.outputX'.format(length2_noStretch_mult), '{}.input[0]'.format(blenderStretchLowerArm))

        # Set up SDK to stretch arm after it's extended beyond its length

        # upper leg
        mc.setDrivenKeyframe(sdkDrivenUpper,
                             currentDriver = sdkDriver,
                             driverValue = armLength,
                             value = upperArm_length,
                             inTangentType = 'linear',
                             outTangentType = 'linear')

        mc.setDrivenKeyframe(sdkDrivenUpper,
                             currentDriver=sdkDriver,
                             driverValue = armLength * 2,
                             value = upperArm_length * 2,
                             inTangentType = 'spline',
                             outTangentType = 'spline')

        animCurveUpperArm = mc.keyframe(sdkDrivenUpper, query=True, name=True)[0]

        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveUpperArm), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveUpperArm), 1)

        # lower leg
        mc.setDrivenKeyframe(sdkDrivenLower,
                             currentDriver = sdkDriver,
                             driverValue = armLength,
                             value = lowerArm_length,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(sdkDrivenLower,
                             currentDriver=sdkDriver,
                             driverValue=armLength * 2,
                             value=lowerArm_length * 2,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurveLowerArm = mc.keyframe(sdkDrivenLower, query=True, name=True)[0]

        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveLowerArm), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveLowerArm), 1)


        # make elbow pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        # make blender node for elbow
        blenderPinUpper = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPinUpper'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinUpper))

        # make blender node for wrist
        blenderPinLower = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPinLower'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinLower))

        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderStretchUpperArm), '{}.input[0]'.format(blenderPinUpper))
        mc.connectAttr('{}.output'.format(blenderStretchLowerArm), '{}.input[0]'.format(blenderPinLower))

        # get distance between shoulderLoc and PV control
        shoulder_pv_dist = mc.shadingNode('distanceBetween', asUtility=True,
                                          n='{}_shoulder_2_pv_distance'.format(self.prefix))
        # connect distance node to shoulder location and pv ctr world matrix
        mc.connectAttr('{}.worldMatrix'.format(shoulderLoc), '{}.inMatrix1'.format(shoulder_pv_dist))
        mc.connectAttr('{}.worldMatrix'.format(poleVectorCtr.C), '{}.inMatrix2'.format(shoulder_pv_dist))


        # make multiply divide node to take into account global scale
        elbowGlobalLength = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_elbowGlobalLength'.format(self.prefix))
        mc.setAttr('{}.operation'.format(elbowGlobalLength), 2)
        mc.connectAttr('{}.distance'.format(shoulder_pv_dist), '{}.input1X'.format(elbowGlobalLength))
        mc.connectAttr('{}.scaleX'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(elbowGlobalLength))

        # Now do lower leg
        # get distance between followWrist and PV control
        wrist_pv_dist = mc.shadingNode('distanceBetween', asUtility=True,
                                       n='{}_pv_2_wrist_distance'.format(self.prefix))
        # connect distance node to pv ctr world matrix and group positioned at wrist
        mc.connectAttr('{}.worldMatrix'.format(followWrist), '{}.inMatrix1'.format(wrist_pv_dist))
        mc.connectAttr('{}.worldMatrix'.format(poleVectorCtr.C), '{}.inMatrix2'.format(wrist_pv_dist))


        # make multiply divide node to take into account global scale
        wristGlobalLength = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_wristGlobalLength'.format(self.prefix))
        mc.setAttr('{}.operation'.format(wristGlobalLength), 2)
        mc.connectAttr('{}.distance'.format(wrist_pv_dist), '{}.input1X'.format(wristGlobalLength))
        mc.connectAttr('{}.scaleX'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(wristGlobalLength))



        if self.side == 'l':

            # connect distance from shoulder to pv ctr to blender input
            mc.connectAttr('{}.outputX'.format(elbowGlobalLength), '{}.input[1]'.format(blenderPinUpper))
            mc.connectAttr('{}.outputX'.format(wristGlobalLength), '{}.input[1]'.format(blenderPinLower))

        elif self.side == 'r':
            # upper
            elbow_negateOutput = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_elbowNegate'.format(self.prefix))
            mc.setAttr('{}.operation'.format(elbow_negateOutput), 1)
            mc.connectAttr('{}.outputX'.format(elbowGlobalLength), '{}.input1X'.format(elbow_negateOutput))
            mc.setAttr('{}.input2X'.format(elbow_negateOutput), -1)
            mc.connectAttr('{}.outputX'.format(elbow_negateOutput),'{}.input[1]'.format(blenderPinUpper))

            # lower
            wrist_negateOutput = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_wristNegate'.format(self.prefix))
            mc.setAttr('{}.operation'.format(wrist_negateOutput), 1)
            mc.connectAttr('{}.outputX'.format(wristGlobalLength), '{}.input1X'.format(wrist_negateOutput))
            mc.setAttr('{}.input2X'.format(wrist_negateOutput), -1)
            mc.connectAttr('{}.outputX'.format(wrist_negateOutput), '{}.input[1]'.format(blenderPinLower))

        # connect blender output to ik elbow joint translate X
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.tx'.format(ikJoints[1]))
        mc.connectAttr('{}.output'.format(blenderPinLower), '{}.tx'.format(ikJoints[2]))

        mc.setAttr('{}.{}'.format(armCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)


        return {'joints': ikJoints, 'controls': controls, 'poleVecLine': poleVectorCurve, 'baseAttachGrp': shoulderAttachGrp, 'rotateGrp': rotateAllGrp}



    def buildScapula(self):

        # Make scapula control

        scapCtr = control.Control(prefix='{}_scapula'.format(self.prefix), translateTo= self.scapulaJoint,
                                  rotateTo = self.scapulaJoint, scale=self.rigScale, parent=self.rigmodule.controlsGrp,
                                  shape = 'circle', offsets= ['null', 'zero', 'auto'])

        scapAimCtr = control.Control(prefix='{}_scapula_translate'.format(self.prefix), translateTo= self.armJoints[0],
                                  scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape = 'square',
                                     lockChannels= ['r', 's'])

        # setup aim constraint on scapCtr

        upVector = mc.group(n = '{}_aim_upVec'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(scapAimCtr.C, upVector, mo=0))
        mc.parent(upVector, scapAimCtr.C)

        mc.aimConstraint( scapAimCtr.C, scapCtr.Offsets[2], aimVector=(1, 0, 0), upVector=(0, 1, 0),
                         worldUpType='objectRotation', worldUpVector=(0, 1, 0), worldUpObject=upVector, mo = 1)

        # connect scapula joint to control
        mc.parentConstraint(scapCtr.C, self.scapulaJoint, mo = 1 )

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

        mc.setAttr('{}.{}'.format(self.switchCtr.C, poseAttr), 0)


        # delete stuff
        mc.delete([loc, shoulderWorld])









