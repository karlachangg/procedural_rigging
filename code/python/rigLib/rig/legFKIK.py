"""
leg FK/IK @ rig
"""

import maya.cmds as mc

from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name

class Leg():

    def __init__(self,
            legJoints,
            toeJoint,
            hipPivotJoint,
            prefix = 'leg',
            side = 'l',
            rigScale = 1.0,
            baseRig = None
            ):
        """
        :param legJoints: list(str), hip - knee - ankle
        :param toeJoint: str, toe joint
        :param hipPivotJoint: str, hip position joint
        :param prefix: str, prefix to name new objects
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """
        self.legJoints = []

        for jnt in legJoints:
            newJnt = side + '_' + jnt
            self.legJoints.append(newJnt)


        self.toeJoint = side + '_' + toeJoint
        self.hipPivotJoint = side + '_' + hipPivotJoint
        self.prefix = side + '_' + prefix
        self.side = side
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix=self.prefix, baseObj=self.baseRig)

        # make attach groups
        #bodyAttachGrp = mc.group(n = '{}_BodyAttach_grp'.format(self.prefix), em=1, p = self.rigmodule.partsGrp)


    def build(self):

        # Add twist joints
        kneeTwistJnt = joint.duplicateChain(self.legJoints[1], oldSuffix= 'jnt', newSuffix= 'twist_jnt')
        ankleTwistJnt = joint.duplicateChain(self.legJoints[2], oldSuffix='jnt', newSuffix='twist_jnt')
        mc.parent(kneeTwistJnt, self.legJoints[0] )
        mc.parent(ankleTwistJnt, self.legJoints[1])

        # Make FK rig
        fkRig = self.buildFK()

        # Make IK rig
        ikRig = self.buildIK()

        # Connect deformation joints to fk and ik joints

        pointConstraints = []
        orientConstraints = []

        for i in range(len(self.legJoints)):
            pConstraint = mc.pointConstraint( fkRig['joints'][i], ikRig['joints'][i], self.legJoints[i], mo=0)[0]
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], self.legJoints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            pointConstraints.append(pConstraint)
            orientConstraints.append(oConstraint)

        # orient constrain twist joints
        kneeTwistConstr = mc.orientConstraint(fkRig['joints'][1], ikRig['joints'][1], kneeTwistJnt,  mo = 1, skip = ['y', 'z'] )[0]
        ankleTwistConstr = mc.orientConstraint(fkRig['joints'][2], ikRig['joints'][2], ankleTwistJnt, mo=1, skip=['y', 'z'])[0]
        mc.setAttr('{}.interpType'.format(kneeTwistConstr), 2)
        mc.setAttr('{}.interpType'.format(ankleTwistConstr), 2)
        orientConstraints.append(kneeTwistConstr)
        orientConstraints.append(ankleTwistConstr)


        # Make switch control

        switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo= self.legJoints[1],
                                 scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='sphere')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(switchCtr.C, ln = switch_attr, at= 'double', min = 0, max = 1, dv = 0, k = 1)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.inputX'.format(reverse) )

        for constraint in pointConstraints:
            weights = mc.pointConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.{}'.format( constraint, weights[1] ) )
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for constraint in orientConstraints:
            weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.{}'.format( constraint, weights[1] ) )
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format( ctrl.Off ) )
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        pointconstraintGrp = mc.group( pointConstraints , n='defSkeleton_{}_pconstraints'.format(self.prefix))
        orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
        mc.parent(pointconstraintGrp, self.baseRig.noXformGrp)
        mc.parent(orientconstraintGrp, self.baseRig.noXformGrp)

        # move switch ctr
        if self.side == 'l':
            mc.move(5, switchCtr.Off, x=True, os=1)
        elif self.side == 'r':
            mc.move(-5, switchCtr.Off, x=True, os=1)











    def buildFK(self):

        # duplicate leg joints to make FK joints
        fkJoints = joint.duplicateChain(self.legJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)

        # make controls

        hipCtr = control.Control(prefix = '{}_hip'.format(self.prefix), translateTo = fkJoints[0], rotateTo = fkJoints[0],
                                  scale = self.rigScale * 1.5, parent = self.rigmodule.controlsGrp, shape = 'circleX')
        kneeCtr = control.Control(prefix = '{}_knee'.format(self.prefix), translateTo = fkJoints[1], rotateTo = fkJoints[1],
                                  scale = self.rigScale, parent = hipCtr.C, shape = 'circleX')
        footCtr = control.Control(prefix = '{}_foot'.format(self.prefix), translateTo = fkJoints[2], rotateTo = fkJoints[2],
                                        scale = self.rigScale, parent = kneeCtr.C, shape = 'circleY')
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
                                  scale=self.rigScale, parent = self.rigmodule.controlsGrp, shape='square')
        poleVectorCtr = control.Control(prefix='{}_leg_pv'.format(self.prefix), translateTo=ikJoints[1],
                                 scale=self.rigScale * 0.5, parent = self.rigmodule.controlsGrp, shape='orb')

        controls = [legCtr, poleVectorCtr]

        # move pole vector ctr
        mc.move( 5, poleVectorCtr.Off, z = True, os = 1)

        # make IK handle
        legIK = mc.ikHandle(n='{}_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        mc.hide(legIK)
        mc.parent(legIK, self.rigmodule.noXformGrp)

        # create no follow PV locator
        pvNoFollow = mc.spaceLocator(n = '{}_pv_noFollow'.format(self.prefix))
        mc.parent(pvNoFollow, self.rigmodule.partsGrp)
        mc.matchTransform(pvNoFollow, poleVectorCtr.C)

        # pole vector position setup
        poleAimLeg = mc.group(n = '{}_poleAim'.format(self.prefix), em = 1)
        upVector = mc.group(n = '{}_pv_upVec'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(legCtr.C, upVector, mo=0))
        mc.parent(upVector, self.rigmodule.partsGrp)
        mc.parentConstraint(legCtr.C, upVector, sr = 'x', mo = 1)


        mc.pointConstraint(ikJoints[0], poleAimLeg, mo = 0)
        mc.aimConstraint(legCtr.C, poleAimLeg, aimVector = (1,0,0), upVector = (0,0,1),
                         worldUpType = 'objectRotation', worldUpVector = (0, 0, 1), worldUpObject = upVector,  mo=0)

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
        mc.parentConstraint(legCtr.C, legIK)
        mc.poleVectorConstraint(poleVectorCtr.C, legIK)
        mc.orientConstraint(legCtr.C, ikJoints[2], mo = 1)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, ikJoints[0], mo = 1 )[0]
        constraintGrp = mc.group(constraint, n='{}_ik_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

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
        mc.parent(followLegIK, legCtr.C)
        # point constrain wrist group to group under ik arm ctr
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
        if self.side == 'r':
            leg_dist_negative = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_leg_distance_negative'.format(self.prefix))
            mc.setAttr('{}.operation'.format(leg_dist_negative), 1)
            mc.connectAttr('{}.outputX'.format(leg_dist_global), '{}.input1X'.format(leg_dist_negative))
            mc.setAttr('{}.input2X'.format(leg_dist_negative), -1)
            sdkDriver = '{}.outputX'.format(leg_dist_negative)
        if self.side == 'l':
            sdkDriver = '{}.outputX'.format(leg_dist_global)

        # Get the original length of the upper and lower leg joints
        upperLeg_length = mc.getAttr('{}.tx'.format(ikJoints[1]))
        lowerLeg_length = mc.getAttr('{}.tx'.format(ikJoints[2]))

        # Calculate the length of fully extended leg
        legLength = upperLeg_length + lowerLeg_length

        # Create blender for stretchy leg setup
        stretch_attr = 'Stretchy'
        mc.addAttr(legCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # make blender node for upper leg
        blenderUpperleg = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderLegUpper'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(legCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderUpperleg))

        # make blender node for lower leg
        blenderLowerleg = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderLegLower'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(legCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderLowerleg))

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        mc.addAttr(legCtr.C, ln = length1_attr, at = 'double', min = 1, dv=1, k=1)
        mc.addAttr(legCtr.C, ln = length2_attr, at='double', min=1, dv=1, k=1)

        # Multiply stretchy leg by length
        leg_length1_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length1_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(leg_length1_mult), 1)
        mc.connectAttr('{}.{}'.format(legCtr.C, length1_attr), '{}.input1X'.format(leg_length1_mult))

        leg_length2_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length2_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(leg_length2_mult), 1)
        mc.connectAttr('{}.{}'.format(legCtr.C, length2_attr), '{}.input1X'.format(leg_length2_mult))

        # Have leg stretchy SDK feed into this multDiv node
        sdkDrivenUpper = '{}.input2X'.format(leg_length1_mult)
        sdkDrivenLower = '{}.input2X'.format(leg_length2_mult)

        # connect multDiv node to blender input
        mc.connectAttr('{}.outputX'.format(leg_length1_mult), '{}.input[1]'.format(blenderUpperleg))
        mc.connectAttr('{}.outputX'.format(leg_length2_mult), '{}.input[1]'.format(blenderLowerleg))

        # Multiply non stretchy leg by length
        leg_length1_noStretch_mult = mc.shadingNode('multiplyDivide', asUtility=True,
                                                    n='{}_length1_noStretch_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(leg_length1_noStretch_mult), 1)
        mc.connectAttr('{}.{}'.format(legCtr.C, length1_attr), '{}.input1X'.format(leg_length1_noStretch_mult))
        mc.setAttr('{}.input2X'.format(leg_length1_noStretch_mult), upperLeg_length)
        # connect output to blender input
        mc.connectAttr('{}.outputX'.format(leg_length1_noStretch_mult), '{}.input[0]'.format(blenderUpperleg))


        leg_length2_noStretch_mult = mc.shadingNode('multiplyDivide', asUtility=True,
                                                    n='{}_length2_noStretch_mult'.format(self.prefix))
        mc.setAttr('{}.operation'.format(leg_length2_noStretch_mult), 1)
        mc.connectAttr('{}.{}'.format(legCtr.C, length2_attr), '{}.input1X'.format(leg_length2_noStretch_mult))
        mc.setAttr('{}.input2X'.format(leg_length2_noStretch_mult), lowerLeg_length)
        # connect upper leg bone original length to blender input
        mc.connectAttr('{}.outputX'.format(leg_length2_noStretch_mult), '{}.input[0]'.format(blenderLowerleg))

        # Set up SDK to stretch leg after it's extended beyond its length
        # upper leg
        mc.setDrivenKeyframe( sdkDrivenUpper,
                              currentDriver = sdkDriver ,
                              driverValue = legLength,
                              value = upperLeg_length,
                              inTangentType = 'linear',
                              outTangentType = 'linear')

        mc.setDrivenKeyframe(sdkDrivenUpper,
                             currentDriver = sdkDriver,
                             driverValue = legLength * 2,
                             value = upperLeg_length * 2,
                             inTangentType = 'spline',
                             outTangentType = 'spline')

        animCurveUpperLeg = mc.keyframe(sdkDrivenUpper, query=True, name=True)[0]

        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveUpperLeg), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveUpperLeg), 1)


        # lower leg
        mc.setDrivenKeyframe(sdkDrivenLower,
                             currentDriver = sdkDriver,
                             driverValue = legLength,
                             value = lowerLeg_length,
                             inTangentType = 'linear',
                             outTangentType = 'linear')

        mc.setDrivenKeyframe(sdkDrivenLower,
                             currentDriver = sdkDriver,
                             driverValue = legLength * 2,
                             value = lowerLeg_length * 2,
                             inTangentType = 'spline',
                             outTangentType = 'spline')

        animCurveLowerLeg = mc.keyframe(sdkDrivenLower, query=True, name=True)[0]
        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveLowerLeg), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveLowerLeg), 1)



        # make elbow pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        # make blender node for upper leg
        blenderPinUpper = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPinUpper'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinUpper))
        # make blender node for lower leg
        blenderPinLower = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPinLower'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinLower))

        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderUpperleg), '{}.input[0]'.format(blenderPinUpper) )
        mc.connectAttr('{}.output'.format(blenderLowerleg), '{}.input[0]'.format(blenderPinLower))

        # get distance between hipFollow and PV control
        hip_pv_dist = mc.shadingNode('distanceBetween', asUtility=True,
                                          n='{}_hip_2_pv_distance'.format(self.prefix))

        # connect distance node to shoulder location and pv ctr world matrix
        mc.connectAttr('{}.worldMatrix'.format(followHip), '{}.inMatrix1'.format(hip_pv_dist))
        mc.connectAttr('{}.worldMatrix'.format(poleVectorCtr.C), '{}.inMatrix2'.format(hip_pv_dist))

        # make multiply divide node to take into account global scale
        upperLegGlobalLength = mc.shadingNode('multiplyDivide', asUtility=True,
                                           n='{}_upperLegGlobalLength'.format(self.prefix))
        mc.setAttr('{}.operation'.format(upperLegGlobalLength), 2)
        mc.connectAttr('{}.distance'.format(hip_pv_dist), '{}.input1X'.format(upperLegGlobalLength))
        mc.connectAttr('{}.scaleX'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(upperLegGlobalLength))

        # Now do lower leg
        # get distance between followFoot and PV control
        pv_foot_dist = mc.shadingNode('distanceBetween', asUtility=True,
                                      n='{}_pv_2_foot_distance'.format(self.prefix))

        # connect distance node to shoulder location and pv ctr world matrix
        mc.connectAttr('{}.worldMatrix'.format(followFoot), '{}.inMatrix1'.format(pv_foot_dist))
        mc.connectAttr('{}.worldMatrix'.format(poleVectorCtr.C), '{}.inMatrix2'.format(pv_foot_dist))

        # make multiply divide node to take into account global scale
        lowerLegGlobalLength = mc.shadingNode('multiplyDivide', asUtility=True,
                                              n='{}_lowerLegGlobalLength'.format(self.prefix))
        mc.setAttr('{}.operation'.format(lowerLegGlobalLength), 2)
        mc.connectAttr('{}.distance'.format(pv_foot_dist), '{}.input1X'.format(lowerLegGlobalLength))
        mc.connectAttr('{}.scaleX'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(lowerLegGlobalLength))

        # connect distance from shoulder to pv ctr to blender input
        if self.side == 'l':
            mc.connectAttr('{}.outputX'.format(upperLegGlobalLength), '{}.input[1]'.format(blenderPinUpper))
            mc.connectAttr('{}.outputX'.format(lowerLegGlobalLength), '{}.input[1]'.format(blenderPinLower))

        elif self.side == 'r':
            # upper leg
            upperLeg_negateOutput = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_upperLegNegate'.format(self.prefix))
            mc.setAttr('{}.operation'.format(upperLeg_negateOutput), 1)
            mc.connectAttr('{}.outputX'.format(upperLegGlobalLength), '{}.input1X'.format(upperLeg_negateOutput))
            mc.setAttr('{}.input2X'.format(upperLeg_negateOutput), -1)
            mc.connectAttr('{}.outputX'.format(upperLeg_negateOutput),'{}.input[1]'.format(blenderPinUpper))

            # lower leg
            lowerLeg_negateOutput = mc.shadingNode('multiplyDivide', asUtility=True,
                                                   n='{}_lowerLegNegate'.format(self.prefix))
            mc.setAttr('{}.operation'.format(lowerLeg_negateOutput), 1)
            mc.connectAttr('{}.outputX'.format(lowerLegGlobalLength), '{}.input1X'.format(lowerLeg_negateOutput))
            mc.setAttr('{}.input2X'.format(lowerLeg_negateOutput), -1)
            mc.connectAttr('{}.outputX'.format(lowerLeg_negateOutput), '{}.input[1]'.format(blenderPinLower))









        # connect blender outputs to ik joints' translate X
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.tx'.format(ikJoints[1]))
        mc.connectAttr('{}.output'.format(blenderPinLower), '{}.tx'.format(ikJoints[2]))

        mc.setAttr('{}.{}'.format(legCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)





        return {'joints' : ikJoints, 'controls' : controls, 'poleVecLine': poleVectorCurve}





