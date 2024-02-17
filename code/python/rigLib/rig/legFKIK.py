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
            toeJoints,
            hipPivotJoint,
            heelLoc,
            innerLoc,
            outerLoc,
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
        :param toeJoints: list(str), toe - toeEnd
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
        self.toeJoints = []

        for jnt in legJoints:
            newJnt = side + '_' + jnt
            self.legJoints.append(newJnt)

        for jnt in toeJoints:
            newJnt = side + '_' + jnt
            self.toeJoints.append(newJnt)


        self.hipPivotJoint = side + '_' + hipPivotJoint
        self.heelLoc = side + '_' + heelLoc
        self.innerLoc = side + '_' + innerLoc
        self.outerLoc = side + '_' + outerLoc
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
                         'ikHandle': '',
                         'ikControl': '',
                         'pvControl': '',
                         'limbMeasureEndNode': '',
                         'bendControls': ''
                         }

    def build(self):


        # Make FK rig
        fkRig = self.buildFK()

        # Define rigParts properties
        self.rigParts['fkControls'] = fkRig['controls']

        # Make IK rig
        if self.type == '2bones':
            ikRig = self.buildIK()
        elif self.type == '3bones':
            ikRig = self.buildQuadIK()


        # Connect deformation joints to fk and ik joints

        pointConstraints = []
        orientConstraints = []
        wholeLeg_Joints = []
        wholeLeg_Joints.extend(self.legJoints)
        wholeLeg_Joints.extend(self.toeJoints)

        for i in range(len(wholeLeg_Joints)):
            pConstraint = mc.pointConstraint( fkRig['joints'][i], ikRig['joints'][i], wholeLeg_Joints[i], mo=0)[0]
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i],wholeLeg_Joints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            pointConstraints.append(pConstraint)
            orientConstraints.append(oConstraint)


        # Make switch control

        switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo= wholeLeg_Joints[1],
                                 scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='plus', color = 'green')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(switchCtr.C, ln = switch_attr, at= 'double', min = 0, max = 1, dv = 0, k = 1)

        control._rotateCtrlShape(switchCtr, axis='x', value=90)

        # Define rigParts properties
        self.rigParts['switchControl'] =switchCtr

        # Create class member so we can access later
        self.FKIKAttr = '{}.{}'.format(switchCtr.C, switch_attr)

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

        # Move the foot joint back
        mc.parent(self.toeJoints[0], lowerLegBendy.bendyJoints[-1])

        # Save rigparts dictionary
        self.rigParts['bendControls'] = upperLegBendy.controls + lowerLegBendy.controls



    def buildFK(self):

        # duplicate leg joints to make FK joints
        fkJoints = joint.duplicateChain(self.legJoints, 'jnt', 'FK_jnt')
        fk_toeJoints = joint.duplicateChain(self.toeJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)
        mc.parent(fk_toeJoints[0], fkJoints[-1] )
        fkJoints.extend(fk_toeJoints)

        # make controls

        hipCtr = control.Control(prefix = '{}_hip'.format(self.prefix), translateTo = fkJoints[0], rotateTo = fkJoints[0],
                                  scale = self.rigScale * 1.5, parent = self.rigmodule.controlsGrp, shape = 'circleX')
        kneeCtr = control.Control(prefix = '{}_knee'.format(self.prefix), translateTo = fkJoints[1], rotateTo = fkJoints[1],
                                  scale = self.rigScale, parent = hipCtr.C, shape = 'circleX')
        footCtr = control.Control(prefix = '{}_foot'.format(self.prefix), translateTo = fkJoints[2], rotateTo = fkJoints[2],
                                        scale = self.rigScale, parent = kneeCtr.C, shape = 'circleX')

        toeCtr = control.Control(prefix='{}_toeFK'.format(self.side), translateTo = fk_toeJoints[0], rotateTo=fk_toeJoints[0],
                                 scale=self.rigScale * 0.5, shape='circleX', parent = footCtr.C)

        controls = [hipCtr, kneeCtr, footCtr, toeCtr]

        # connect controls

        mc.parentConstraint(hipCtr.C, fkJoints[0], mo = 0 )
        mc.parentConstraint(kneeCtr.C, fkJoints[1], mo=0)
        mc.parentConstraint(footCtr.C, fkJoints[2], mo=0)
        mc.parentConstraint(toeCtr.C, fk_toeJoints[0], mo=0)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, hipCtr.Off, mo=1)[0]
        constraintGrp = mc.group(constraint, n= '{}_fk_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)



        return {'joints' : fkJoints, 'controls' : controls }






    def buildIK(self, buildFoot = True):

        # duplicate leg joints to make IK joints
        ikJoints = joint.duplicateChain(self.legJoints, 'jnt', 'IK_jnt')
        ik_toeJoints = joint.duplicateChain(self.toeJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)
        mc.parent(ik_toeJoints[0], ikJoints[-1])
        ikJoints.extend(ik_toeJoints)



        # Make controls
        controls = []
        legCtr = control.Control(prefix='{}_leg_ik'.format(self.prefix), translateTo=ikJoints[2],
                                  scale=self.rigScale, parent = self.rigmodule.controlsGrp, shape='cube')
        poleVectorCtr = control.Control(prefix='{}_leg_pv'.format(self.prefix), translateTo=ikJoints[1], rotateTo = ikJoints[2],
                                 scale=self.rigScale * 0.5, parent = self.rigmodule.controlsGrp, shape='locator')

        controls = [legCtr, poleVectorCtr]

        self.rigParts['ikControl'] = legCtr
        self.rigParts['pvControl'] = poleVectorCtr

        # move pole vector ctr
        units = 5

        if self.kneeDirection == 'x' or self.kneeDirection == '-x':
            x, y, z = True, False, False

        elif self.kneeDirection == 'y' or self.kneeDirection == '-y':
            x, y, z = False, True, False

        elif self.kneeDirection == 'z' or self.kneeDirection == '-z':
            x, y, z = False, False, True

        if '-' in self.kneeDirection:
            units = units * -1


        mc.move(units, poleVectorCtr.Off, x=x, y=y, z=z, os=1)

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

        poleAimConstraint = mc.aimConstraint(legCtr.C, poleAimLeg, aimVector = (1,0,0), upVector = (0,0,1),
                         worldUpType = 'objectRotation', worldUpVector = (0, 0, 1), worldUpObject = upVector,  mo=0)[0]

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
        if buildFoot == False:
            mc.parentConstraint(legCtr.C, legIK)

        mc.poleVectorConstraint(poleVectorCtr.C, legIK)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, ikJoints[0], mo = 1 )[0]
        constraintGrp = mc.group(constraint, n='{}_ik_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)


        # Add Leg swivel attribute

        spin_attr = 'Spin'
        mc.addAttr(legCtr.C, ln = spin_attr, at='double', dv=0, k=1)
        mc.connectAttr('{}.{}'.format(legCtr.C, spin_attr), '{}.offsetX'.format(poleAimConstraint))

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
        # parent constrain followFoot group to group under ik leg ctr
        mc.parentConstraint(followLegIK, followFoot, mo=0)
        mc.parent(followFoot, self.rigmodule.partsGrp)

        # Add followLegIK to rigParts dictionary bc we will need it in case of a reverse foot setup
        self.rigParts['limbMeasureEndNode'] = followLegIK

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

        if self.forwardAxis == 'x' or '-x':
            lengthAxisAttr = 'tx'
        elif self.forwardAxis == 'y' or '-y':
            lengthAxisAttr = 'ty'
        elif self.forwardAxis == 'z' or '-z':
            lengthAxisAttr = 'tz'

        upperLeg_length = mc.getAttr('{}.{}'.format(ikJoints[1], lengthAxisAttr))
        lowerLeg_length = mc.getAttr('{}.{}'.format(ikJoints[2], lengthAxisAttr))

        # Calculate the length of fully extended leg
        legLength = upperLeg_length + lowerLeg_length

        # Create blender for stretchy leg setup
        stretch_attr = 'Stretchy'
        mc.addAttr(legCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # Create class member so we can access later
        self.StretchyAttr = '{}.{}'.format(legCtr.C, stretch_attr)

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



        # make knee pin to pv setup
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
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.{}'.format(ikJoints[1], lengthAxisAttr))
        mc.connectAttr('{}.output'.format(blenderPinLower), '{}.{}'.format(ikJoints[2], lengthAxisAttr))

        mc.setAttr('{}.{}'.format(legCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)


        ik_footJoints = [ikJoints[2], ik_toeJoints[0], ik_toeJoints[1] ]

        if buildFoot == True:

            footRig = self.buildReverseFoot(heel_loc = self.heelLoc,
                                  inner_loc = self.innerLoc,
                                  outer_loc = self.outerLoc,
                                  footJoints = ik_footJoints,
                                  leg_IKH = [legIK],
                                  footCtr= legCtr,
                                  measureLeg_end_node = followLegIK
                                  )

            footControls = footRig['controls']
            controls.extend(footControls)



        return {'joints' : ikJoints, 'controls' : controls, 'poleVecLine': poleVectorCurve}






    def buildQuadIK(self, buildFoot=True):

        # duplicate leg joints to make IK joints
        ikJoints = joint.duplicateChain(self.legJoints, 'jnt', 'IK_jnt')
        ik_toeJoints = joint.duplicateChain(self.toeJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)
        mc.parent(ik_toeJoints[0], ikJoints[-1])
        ikJoints.extend(ik_toeJoints)

        # Make controls
        controls = []
        legCtr = control.Control(prefix='{}_leg_ik'.format(self.prefix), translateTo = self.toeJoints[0],
                                 scale= self.rigScale, parent=self.rigmodule.controlsGrp, shape='square')
        poleVectorCtr = control.Control(prefix='{}_leg_pv'.format(self.prefix), translateTo=ikJoints[1], rotateTo = ikJoints[1],
                                        scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='orb')

        controls = [legCtr, poleVectorCtr]

        # move pole vector ctr
        units = 2.5

        if self.kneeDirection == 'x' or self.kneeDirection == '-x':
            x, y, z = True, False, False

        elif self.kneeDirection == 'y' or self.kneeDirection == '-y':
            x, y, z = False, True, False

        elif self.kneeDirection == 'z' or self.kneeDirection == '-z':
            x, y, z = False, False, True

        if '-' in self.kneeDirection:
            units = units * -1

        mc.move(units, poleVectorCtr.Off, x=x, y=y, z=z, os=1)

        # make IK handles
        legIK = mc.ikHandle(n='{}_wholeLeg_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[3])[0]
        mc.hide(legIK)

        kneeIK = mc.ikHandle(n='{}_Knee_ikh'.format(self.prefix), sol='ikSCsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        mc.hide(kneeIK)

        ankleIK = mc.ikHandle(n='{}_Ankle_ikh'.format(self.prefix), sol='ikSCsolver', sj=ikJoints[2], ee=ikJoints[3])[0]
        mc.hide(ankleIK)




        # create no follow PV locator
        pvNoFollow = mc.spaceLocator(n='{}_pv_noFollow'.format(self.prefix))
        mc.parent(pvNoFollow, self.rigmodule.partsGrp)
        mc.matchTransform(pvNoFollow, poleVectorCtr.C)

        # pole vector position setup
        poleAimLeg = mc.group(n='{}_poleAim'.format(self.prefix), em=1)
        upVector = mc.group(n='{}_pv_upVec'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(legCtr.C, upVector, mo=0))
        mc.parent(upVector, self.rigmodule.partsGrp)
        mc.parentConstraint(legCtr.C, upVector, sr='x', mo=1)
        mc.pointConstraint(ikJoints[0], poleAimLeg, mo=0)

        poleAimConstraint = mc.aimConstraint(legCtr.C, poleAimLeg, aimVector=(1, 0, 0), upVector=(0, 0, 1),
                                             worldUpType='objectRotation', worldUpVector=(0, 0, 1),
                                             worldUpObject = upVector, mo=0)[0]

        poleOffsetFollow_noScale = mc.group(n='{}_poleOffsetFollow_noScale'.format(self.prefix), em=1)
        mc.pointConstraint(poleAimLeg, poleOffsetFollow_noScale, mo=0)
        mc.orientConstraint(poleAimLeg, poleOffsetFollow_noScale, mo=0)
        poleOffsetFollow = mc.group(n='{}_poleOffsetFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(poleVectorCtr.C, poleOffsetFollow, mo=0))
        mc.parentConstraint(poleOffsetFollow_noScale, poleOffsetFollow, mo=1)
        mc.parent(poleAimLeg, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow_noScale, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow, self.rigmodule.partsGrp)

        # constrain pv
        pv_constraint = mc.parentConstraint(poleOffsetFollow, pvNoFollow, poleVectorCtr.Off, mo=1)[0]
        weights = mc.parentConstraint(pv_constraint, q=1, weightAliasList=1)

        # setup pv follow switch
        pv_follow_attr = 'Follow'
        mc.addAttr(poleVectorCtr.C, ln=pv_follow_attr, at='double', min=0, max=1, dv=1, k=1)
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.{}'.format(pv_constraint, weights[0]))
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_pvFollow_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.inputX'.format(reverse))
        mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(pv_constraint, weights[1]))
        mc.parent(pv_constraint, self.rigmodule.noXformGrp)


        # Pole vector constraint
        mc.poleVectorConstraint(poleVectorCtr.C, legIK)

        # Connect major parts of the leg
        #mc.orientConstraint(legCtr.C, ikJoints[2], mo=1)


        if buildFoot == False:
            mc.parentConstraint(legCtr.C, legIK)

        # Drive IK handles
        mc.parent( ankleIK, legCtr.C,)
        kneeIKGrp = mc.group(n = '{}_kneeIKGrp'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(kneeIK, kneeIKGrp, mo = 0))
        mc.parent(kneeIK, kneeIKGrp)
        mc.parent(kneeIKGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(legCtr.C, kneeIKGrp, mo = 1)

        # Add Leg swivel attribute

        spin_attr = 'Spin'
        mc.addAttr(legCtr.C, ln=spin_attr, at='double', dv=0, k=1)
        mc.connectAttr('{}.{}'.format(legCtr.C, spin_attr), '{}.offsetX'.format(poleAimConstraint))


        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, ikJoints[0], mo=1)[0]
        constraintGrp = mc.group(constraint, n='{}_ik_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)



        # make pole vector connection line

        pvLinePos1 = mc.xform(ikJoints[1], q=1, t=1, ws=1)
        pvLinePos2 = mc.xform(poleVectorCtr.C, q=1, t=1, ws=1)
        poleVectorCurve = mc.curve(n='{}_pv_curve'.format(self.prefix), d=1, p=[pvLinePos1, pvLinePos2])

        mc.cluster('{}.cv[0]'.format(poleVectorCurve), n='{}_pv1_cls'.format(self.prefix),
                   wn=[ikJoints[1], ikJoints[1]],
                   bs=True)
        mc.cluster('{}.cv[1]'.format(poleVectorCurve), n='{}_pv2_cls'.format(self.prefix),
                   wn=[poleVectorCtr.C, poleVectorCtr.C], bs=True)

        mc.parent(poleVectorCurve, self.rigmodule.controlsGrp)
        mc.setAttr('{}.template'.format(poleVectorCurve), 1)
        mc.setAttr('{}.it'.format(poleVectorCurve), 0)






        # make stretchy leg

        # Create group to follow hip location
        followHip = mc.group(n='{}_IKHipFollow'.format(self.prefix), em=1)
        mc.parentConstraint(self.hipPivotJoint, followHip, mo=0)
        mc.parent(followHip, self.rigmodule.partsGrp)

        # Create group to follow foot IK control
        followFoot = mc.group(n='{}_IKFootFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(legCtr.C, followFoot, mo=0))
        # create empty group under ik leg ctr
        followLegIK = mc.group(n='{}_IKLegFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(legCtr.C, followLegIK, mo=0))
        mc.parent(followLegIK, legCtr.C)
        # parent constrain followFoot group to group under ik leg ctr
        mc.parentConstraint(followLegIK, followFoot, mo=0)
        mc.parent(followFoot, self.rigmodule.partsGrp)

        # create a distance node to get the length between the two groups
        leg_dist = mc.shadingNode('distanceBetween', asUtility=True, n='{}_leg_length'.format(self.prefix))
        # connect distance node to pv ctr world matrix and group positioned at wrist
        mc.connectAttr('{}.worldMatrix'.format(followHip), '{}.inMatrix1'.format(leg_dist))
        mc.connectAttr('{}.worldMatrix'.format(followFoot), '{}.inMatrix2'.format(leg_dist))
        # divide leg length by global scale
        leg_dist_global = mc.shadingNode('multiplyDivide', asUtility=True,
                                         n='{}_leg_distance_global'.format(self.prefix))
        mc.setAttr('{}.operation'.format(leg_dist_global), 2)
        mc.connectAttr('{}.distance'.format(leg_dist), '{}.input1X'.format(leg_dist_global))
        mc.connectAttr('{}.sx'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(leg_dist_global))

        # Negate leg distance value for right side

        if self.side == 'r':
            leg_dist_negative = mc.shadingNode('multiplyDivide', asUtility=True,
                                               n='{}_leg_distance_negative'.format(self.prefix))
            mc.setAttr('{}.operation'.format(leg_dist_negative), 1)
            mc.connectAttr('{}.outputX'.format(leg_dist_global), '{}.input1X'.format(leg_dist_negative))
            mc.setAttr('{}.input2X'.format(leg_dist_negative), -1)
            sdkDriver = '{}.outputX'.format(leg_dist_negative)
        if self.side == 'l':
            sdkDriver = '{}.outputX'.format(leg_dist_global)

        # Get the original length of the upper and lower leg joints
        upperLeg_length = mc.getAttr('{}.tx'.format(ikJoints[1]))
        lowerLeg_length = mc.getAttr('{}.tx'.format(ikJoints[2]))
        ankle_length = mc.getAttr('{}.tx'.format(ikJoints[3]))

        # Calculate the length of fully extended leg
        legLength = upperLeg_length + lowerLeg_length + ankle_length

        # Create blender for stretchy leg setup
        stretch_attr = 'Stretchy'
        mc.addAttr(legCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # Create class member so we can access later
        self.StretchyAttr = '{}.{}'.format(legCtr.C, stretch_attr)

        # make blender node for upper leg
        blenderUpperleg = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderLegUpper'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(legCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderUpperleg))

        # make blender node for lower leg
        blenderLowerleg = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderLegLower'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(legCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderLowerleg))

        # make blender node for ankle bone
        blenderAnkleBone = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderAnkleBone'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(legCtr.C, stretch_attr), '{}.attributesBlender'.format(blenderAnkleBone))

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        mc.addAttr(legCtr.C, ln=length1_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(legCtr.C, ln=length2_attr, at='double', min=1, dv=1, k=1)

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

        # Since we are not making a length attribute for the ankle bone, set the sdk Driven to be the blender
        sdkDrivenAnkleBone = '{}.input[1]'.format(blenderAnkleBone)

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

        # Since we are not adding a length attribute for the ankle bone, set the blender input 0 to the original leg length value
        mc.setAttr('{}.input[0]'.format(blenderAnkleBone), ankle_length)

        # Set up SDK to stretch leg after it's extended beyond its length

        # upper leg
        mc.setDrivenKeyframe(sdkDrivenUpper,
                             currentDriver=sdkDriver,
                             driverValue=legLength,
                             value=upperLeg_length,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(sdkDrivenUpper,
                             currentDriver=sdkDriver,
                             driverValue=legLength * 2,
                             value=upperLeg_length * 2,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurveUpperLeg = mc.keyframe(sdkDrivenUpper, query=True, name=True)[0]

        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveUpperLeg), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveUpperLeg), 1)

        # lower leg
        mc.setDrivenKeyframe(sdkDrivenLower,
                             currentDriver=sdkDriver,
                             driverValue=legLength,
                             value=lowerLeg_length,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(sdkDrivenLower,
                             currentDriver=sdkDriver,
                             driverValue=legLength * 2,
                             value=lowerLeg_length * 2,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurveLowerLeg = mc.keyframe(sdkDrivenLower, query=True, name=True)[0]
        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveLowerLeg), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveLowerLeg), 1)

        # ankle bone

        mc.setDrivenKeyframe(sdkDrivenAnkleBone,
                             currentDriver=sdkDriver,
                             driverValue=legLength,
                             value = ankle_length,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(sdkDrivenAnkleBone,
                             currentDriver=sdkDriver,
                             driverValue=legLength * 2,
                             value = ankle_length * 2,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurveAnkleBone = mc.keyframe(sdkDrivenAnkleBone, query=True, name=True)[0]

        if self.side == 'l':
            mc.setAttr('{}.postInfinity'.format(animCurveAnkleBone), 1)
        elif self.side == 'r':
            mc.setAttr('{}.preInfinity'.format(animCurveAnkleBone), 1)

        # make knee pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        # make blender node for upper leg
        blenderPinUpper = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPinUpper'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinUpper))
        # make blender node for lower leg
        blenderPinLower = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPinLower'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinLower))

        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderUpperleg), '{}.input[0]'.format(blenderPinUpper))
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
            upperLeg_negateOutput = mc.shadingNode('multiplyDivide', asUtility=True,
                                                   n='{}_upperLegNegate'.format(self.prefix))
            mc.setAttr('{}.operation'.format(upperLeg_negateOutput), 1)
            mc.connectAttr('{}.outputX'.format(upperLegGlobalLength), '{}.input1X'.format(upperLeg_negateOutput))
            mc.setAttr('{}.input2X'.format(upperLeg_negateOutput), -1)
            mc.connectAttr('{}.outputX'.format(upperLeg_negateOutput), '{}.input[1]'.format(blenderPinUpper))

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

        # for the ankle bone, connect it to the stretch/ no stretch blender
        mc.connectAttr('{}.output'.format(blenderAnkleBone), '{}.tx'.format(ikJoints[3]))
        mc.setAttr('{}.{}'.format(legCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)



        ik_footJoints = [ik_toeJoints[0], ik_toeJoints[1], ik_toeJoints[2]]

        if buildFoot == True:
            footRig = self.buildReverseFoot(heel_loc=self.heelLoc,
                                            inner_loc = self.innerLoc,
                                            outer_loc=self.outerLoc,
                                            footJoints=ik_footJoints,
                                            leg_IKH= [legIK, ankleIK],
                                            footCtr=legCtr,
                                            measureLeg_end_node = followLegIK,
                                            rollAxis= '-z',
                                            rockAxis= 'x'
                                            )

            footControls = footRig['controls']
            controls.extend(footControls)



        return {'joints': ikJoints, 'controls': controls, 'poleVecLine': poleVectorCurve}


















    def buildReverseFoot(self,
                         heel_loc,
                         inner_loc,
                         outer_loc,
                         footJoints,
                         leg_IKH,
                         footCtr,
                         measureLeg_end_node,
                         rollAxis = 'x',
                         rockAxis = 'z'
                         ):

        # Create control at the ball of the foot
        ballCtr = control.Control(prefix='{}_footRoll'.format(self.side), translateTo = footJoints[1],
                                  scale=self.rigScale * 0.5, shape='circleX')

        # Create control at the end of the foot (toe)

        toeEndCtr = control.Control(prefix='{}_toeRoll'.format(self.side), translateTo=footJoints[2],
                                  scale=self.rigScale * 0.25, shape='circleX')

        toeCtr = control.Control(prefix='{}_toe'.format(self.side), translateTo=footJoints[1], rotateTo= footJoints[1],
                                  scale=self.rigScale * 0.5, shape='circleX')

        # Create control at the heel of the foot

        heelCtr = control.Control(prefix='{}_heelRoll'.format(self.side), translateTo = heel_loc,
                                    scale=self.rigScale * 0.3, shape='circleX')

        controls = [ballCtr, toeEndCtr, toeCtr, heelCtr]

        mc.parent(heel_loc, heelCtr.C)
        mc.hide(heel_loc)

        innerPivot_Grp = inner_loc + '_grp'
        outerPivot_Grp = outer_loc + '_grp'

        mc.group(em = 1, n = innerPivot_Grp)
        mc.group(em=1, n = outerPivot_Grp)

        mc.hide(inner_loc)
        mc.hide(outer_loc)

        mc.delete(mc.parentConstraint(inner_loc, innerPivot_Grp, mo = 0))
        mc.delete(mc.parentConstraint(outer_loc, outerPivot_Grp, mo=0))

        mc.parent(inner_loc, innerPivot_Grp)
        mc.parent(outer_loc, outerPivot_Grp)

        # Create IK single chain solver from the ankle to the ball of the foot
        ball_IKH = mc.ikHandle(n='{}_ball_ikh'.format(self.prefix), sol='ikSCsolver', sj=footJoints[0], ee=footJoints[1])[0]
        mc.hide(ball_IKH)

        # Create IK single chain solver from the ball of the foot to the toe
        toe_IKH = mc.ikHandle(n='{}_toe_ikh'.format(self.prefix), sol='ikSCsolver', sj=footJoints[1], ee=footJoints[2])[0]
        mc.hide(toe_IKH)

        # Parent IK handles to controls
        mc.parent(ball_IKH, ballCtr.C)
        mc.parent( toe_IKH, toeCtr.C)
        for ikHandle in leg_IKH:
            mc.parent( ikHandle, ballCtr.C)

        # Set up parenting structure
        mc.parent(ballCtr.Off, toeEndCtr.C)
        mc.parent(toeCtr.Off, toeEndCtr.C)
        mc.parent(toeEndCtr.Off, heelCtr.C)
        mc.parent(heelCtr.Off, outerPivot_Grp)
        mc.parent(outerPivot_Grp, innerPivot_Grp)
        mc.parent(innerPivot_Grp, footCtr.C)
        # parent the node we use to measure the leg length to the ball control. Because this will give us a more accurate reading
        mc.parent(measureLeg_end_node, ballCtr.C)

        # Set up Roll
        roll_attr = 'Roll'
        mc.addAttr(footCtr.C, ln = roll_attr, at= 'double', dv = 0, k = 1)

        ball_angle_attr = 'Ball_Roll_Angle'
        mc.addAttr(footCtr.C, ln = ball_angle_attr, at='double', dv=0, k=1)
        mc.setAttr('{}.{}'.format(footCtr.C, ball_angle_attr), 45)

        # Create class member so we can access later
        self.BallRollAngleAttr = '{}.{}'.format(footCtr.C, ball_angle_attr)

        toe_angle_attr = 'Toe_Roll_Angle'
        mc.addAttr(footCtr.C, ln = toe_angle_attr, at='double', dv=0, k=1)
        mc.setAttr('{}.{}'.format(footCtr.C, toe_angle_attr), 60)

        # Create class member so we can access later
        self.ToeRollAngleAttr = '{}.{}'.format(footCtr.C, toe_angle_attr)

        # Connect heel pivot
        heelRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                               n='{}_heelRoll_clamp'.format(self.prefix))

        mc.connectAttr('{}.{}'.format(footCtr.C, roll_attr), '{}.inputR'.format(heelRoll_clamp))
        mc.setAttr( '{}.minR'.format(heelRoll_clamp), -90)
        mc.setAttr('{}.maxR'.format(heelRoll_clamp), 0)

        if rollAxis == 'x' or rollAxis == '-x':
            rollAxisAttr = 'rotateX'

        elif rollAxis == 'y' or rollAxis == '-y':
            rollAxisAttr = 'rotateY'

        elif rollAxis == 'z' or rollAxis == '-z':
            rollAxisAttr = 'rotateZ'

        rollUpNegative = False
        if '-' in rollAxis:
            rollUpNegative = True

        mc.connectAttr('{}.outputR'.format(heelRoll_clamp), '{}.{}'.format(heelCtr.Off, rollAxisAttr))


        # Connect toe pivot
        toeRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                         n='{}_toeRoll_clamp'.format(self.prefix))


        toeRoll_setRange = mc.shadingNode('setRange', asUtility=True,
                                         n='{}_toeRoll_setRange'.format(self.prefix))

        toeRoll_mult_percent = mc.shadingNode('multiplyDivide', asUtility=True,
                                         n='{}_toeRoll_mult_percent'.format(self.prefix))

        # Roll value goes into our clamp input
        mc.connectAttr('{}.{}'.format(footCtr.C, roll_attr), '{}.inputR'.format(toeRoll_clamp))
        # Ball angle attr goes into our clamp minimum
        mc.connectAttr('{}.{}'.format(footCtr.C, ball_angle_attr), '{}.minR'.format(toeRoll_clamp))
        # Toe angle attr goes into our clamp maximum
        mc.connectAttr('{}.{}'.format(footCtr.C, toe_angle_attr), '{}.maxR'.format(toeRoll_clamp))

        # Remap our clamped values from zero to one
        mc.connectAttr('{}.minR'.format(toeRoll_clamp), '{}.oldMinX'.format(toeRoll_setRange))
        mc.connectAttr('{}.maxR'.format(toeRoll_clamp), '{}.oldMaxX'.format(toeRoll_setRange))
        mc.setAttr('{}.minX'.format(toeRoll_setRange), 0)
        mc.setAttr('{}.maxX'.format(toeRoll_setRange), 1)
        mc.connectAttr('{}.inputR'.format(toeRoll_clamp), '{}.valueX'.format(toeRoll_setRange))

        # Multiply the remapped value (percentage of roll between our ball and toe angles) by Roll amount
        mc.connectAttr('{}.outValueX'.format(toeRoll_setRange), '{}.input1X'.format(toeRoll_mult_percent))
        mc.connectAttr('{}.inputR'.format(toeRoll_clamp), '{}.input2X'.format(toeRoll_mult_percent))
        mc.setAttr('{}.operation'.format(toeRoll_mult_percent), 1)



        if rollUpNegative:

            # Negate the roll value
            toeRoll_mult_negate = mc.shadingNode('multiplyDivide', asUtility=True,
                                                  n='{}_toeRoll_mult_negate'.format(self.prefix))

            mc.connectAttr('{}.outputX'.format(toeRoll_mult_percent), '{}.input1X'.format(toeRoll_mult_negate))
            mc.setAttr( '{}.input2X'.format(toeRoll_mult_negate), -1)
            mc.setAttr('{}.operation'.format(toeRoll_mult_negate), 1)

            toeRollDriver = '{}.outputX'.format(toeRoll_mult_negate)



        else:
            toeRollDriver = '{}.outputX'.format(toeRoll_mult_percent)

        # Feed the out value of our multiply node to toe_loc rotate axis
        mc.connectAttr(toeRollDriver, '{}.{}'.format(toeEndCtr.Off, rollAxisAttr))

        # Connect ball pivot
        ballRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                       n='{}_ballRoll_clamp'.format(self.prefix))

        ballRoll_setRange = mc.shadingNode('setRange', asUtility=True,
                                          n='{}_ballRoll_setRange'.format(self.prefix))

        toeRoll_reverse = mc.shadingNode('reverse', asUtility=True,
                                          n='{}_toeRoll_reverse'.format(self.prefix))

        ballRoll_combine_percentages = mc.shadingNode('multiplyDivide', asUtility=True,
                                               n='{}_ballRoll_combine_percentages'.format(self.prefix))

        ballRoll_mult_percent = mc.shadingNode('multiplyDivide', asUtility=True,
                                              n='{}_ballRoll_mult_percent'.format(self.prefix))

        # Roll value goes into our clamp input
        mc.connectAttr('{}.{}'.format(footCtr.C, roll_attr), '{}.inputR'.format(ballRoll_clamp))
        # Clamp minimum set to zero
        mc.setAttr('{}.minR'.format(ballRoll_clamp), 0)
        # Ball angle attr goes into our clamp maximum
        mc.connectAttr('{}.{}'.format(footCtr.C, ball_angle_attr), '{}.maxR'.format(ballRoll_clamp))

        # Remap our clamped values from zero to one
        mc.connectAttr('{}.minR'.format(ballRoll_clamp), '{}.oldMinX'.format(ballRoll_setRange))
        mc.connectAttr('{}.maxR'.format(ballRoll_clamp), '{}.oldMaxX'.format(ballRoll_setRange))
        mc.setAttr('{}.minX'.format(ballRoll_setRange), 0)
        mc.setAttr('{}.maxX'.format(ballRoll_setRange), 1)
        mc.connectAttr('{}.inputR'.format(ballRoll_clamp), '{}.valueX'.format(ballRoll_setRange))

        # Get the reciprocal of the toe range
        mc.connectAttr('{}.outValueX'.format(toeRoll_setRange), '{}.inputX'.format(toeRoll_reverse))

        # Multiply the range of the ball roll with the reciprocal range of the toe roll
        mc.connectAttr('{}.outputX'.format(toeRoll_reverse), '{}.input1X'.format(ballRoll_combine_percentages))
        mc.connectAttr('{}.outValueX'.format(ballRoll_setRange), '{}.input2X'.format(ballRoll_combine_percentages))
        mc.setAttr('{}.operation'.format(ballRoll_combine_percentages), 1)

        # Multiply the resulting percentage by the Roll amount
        mc.connectAttr('{}.outputX'.format(ballRoll_combine_percentages), '{}.input1X'.format(ballRoll_mult_percent))
        mc.connectAttr('{}.inputR'.format(ballRoll_clamp), '{}.input2X'.format(ballRoll_mult_percent))
        mc.setAttr('{}.operation'.format(ballRoll_mult_percent), 1)


        if rollUpNegative:

            # Negate the roll value
            ballRoll_mult_negate = mc.shadingNode('multiplyDivide', asUtility=True,
                                                  n='{}_ballRoll_mult_negate'.format(self.prefix))

            mc.connectAttr('{}.outputX'.format(ballRoll_mult_percent), '{}.input1X'.format(ballRoll_mult_negate))
            mc.setAttr( '{}.input2X'.format(ballRoll_mult_negate), -1)
            mc.setAttr('{}.operation'.format(ballRoll_mult_negate), 1)

            ballRollDriver = '{}.outputX'.format(ballRoll_mult_negate)



        else:
            ballRollDriver = '{}.outputX'.format(ballRoll_mult_percent)

        # Feed the out value of our multiply node to ballCtr rotation
        mc.connectAttr( ballRollDriver , '{}.{}'.format(ballCtr.Off, rollAxisAttr))

        # Set up rock
        rock_attr = 'Rock'
        mc.addAttr(footCtr.C, ln=rock_attr, at='double', dv=0, k=1)

        if rockAxis == 'x':
            rockAxisAttr = 'rotateX'

        elif rockAxis == 'y':
            rockAxisAttr = 'rotateY'

        elif rockAxis == 'z':
            rockAxisAttr = 'rotateZ'


        if self.side == 'l':

            # Set up SDK to rock outer pivot
            mc.setDrivenKeyframe( '{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                  currentDriver = '{}.{}'.format(footCtr.C, rock_attr) ,
                                  driverValue = 0,
                                  value = 0,
                                  inTangentType = 'linear',
                                  outTangentType = 'linear')

            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue = 90,
                                 value = -90,
                                 inTangentType='linear',
                                 outTangentType='linear')


            # Set up SDK to rock inner pivot
            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue = -90,
                                 value = 90,
                                 inTangentType='linear',
                                 outTangentType='linear')

        elif self.side == 'r':

            # Set up SDK to rock outer pivot
            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue=90,
                                 value = 90,
                                 inTangentType='linear',
                                 outTangentType='linear')

            # Set up SDK to rock inner pivot
            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                                 driverValue=-90,
                                 value = - 90,
                                 inTangentType='linear',
                                 outTangentType='linear')


        return {'controls': controls}

    def setInitialValues(self,
                         FKIKMode = 1,
                         Stretchy = 1,
                         BallRollAngle = 45 ,
                         ToeRollAngle = 60,
                         ):

        mc.setAttr(self.FKIKAttr, FKIKMode)
        mc.setAttr(self.StretchyAttr, Stretchy)
        mc.setAttr(self.BallRollAngleAttr, BallRollAngle)
        mc.setAttr(self.ToeRollAngleAttr, ToeRollAngle)
