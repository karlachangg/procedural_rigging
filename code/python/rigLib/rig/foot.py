"""
reverse foot rig
"""

import maya.cmds as mc

from ..base import control
from ..utils import joint

class Foot():

    def __init__(self,

            toeJoints,
            heelLoc,
            innerLoc,
            outerLoc,

            fkParentJoint = '',
            ikParentJoint = '',

            fkFootCtr = '',
            ikFootCtr = '',
            reverseFootParent = '',

            ikGroupToDrive = '',
            switchAttr = '',

            legType = 'planti',

            ballCtr = '',
            toeIKCtr = '',
            ankleRollDriver = '',

            rollAxis = 'x',
            rockAxis = 'z',
            prefix = 'foot',
            side = 'l',
            rigScale = 1.0,
            baseRig = None,

            ):

        """
        :param toeJoints: list(str), ballPivot - toeEnd
        :param heelLoc: str, locator at the heel pivot
        :param innerLoc: str, locator at the inner rocking pivot
        :param outerLoc: str, locator at the outer rocking pivot

        :param fkParentJointJoint: str, name of fk joint to parent the fk toe joint to
        :param ikParentJointJoint: str, name of ik joint to parent the ik toe joint to

        :param fkFootCtr: control object, control on which to parent the fk toe control
        :param ikFootCtr: control object, Limb ik control on which to add the rock and roll attributes
        :param reverseFootParent: str, Control to parent the reverse foot controls to
        :param ikGroupToDrive: str, Group of ik limb items which should move with the reverse foot roll
        :param switchAttr: str, "object.attr" name of fk to ik switcher

        :param rollAxis: str, axis to roll ball and toe joints UP. Default "x"
        :param rockAxis: str, axis to roll outer pivot outwards. Default "z"
        :param prefix: str, prefix to name new objects
        :param side: str, side notation to add to prefix
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class

        :return: dictionary with rig module objects
        """

        self.toeJoints = []

        for jnt in toeJoints:
            newJnt = side + '_' + jnt
            self.toeJoints.append(newJnt)

        self.heelLoc = side + '_' + heelLoc
        self.innerLoc = side + '_' + innerLoc
        self.outerLoc = side + '_' + outerLoc

        self.fkParentJoint = fkParentJoint
        self.ikParentJoint = ikParentJoint

        self.fkFootCtr = fkFootCtr
        self.ikFootCtr = ikFootCtr
        self.reverseFootParent = reverseFootParent
        self.ikGroupToDrive = ikGroupToDrive
        self.switchAttr = switchAttr

        self.legType = legType

        self.ballCtr = ballCtr
        self.toeIKCtr = toeIKCtr
        self.ankleRollDriver = ankleRollDriver

        self.rollAxis = rollAxis
        self.rockAxis = rockAxis
        self.prefix =  side + '_' + prefix
        self.side = side
        self.rigScale = rigScale
        self.baseRig = baseRig

        self.rigParts = {'fkControls': '',
                         'ikControls': '',
                         }


    def build(self):

        if self.legType == 'planti':

            # Build FK toe
            fkRig = self.buildFK()

            # Build IK rig
            ikRig = self.buildReverseFoot()

            self.rigParts['fkControls'] = fkRig['controls']
            self.rigParts['ikControls'] = ikRig['controls']

            # Connect deformation joints to fk and ik joints

            orientConstraints = []

            for i in range(len(self.toeJoints)):
                oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], self.toeJoints[i], mo=0)[0]
                mc.setAttr('{}.interpType'.format(oConstraint), 2)
                orientConstraints.append(oConstraint)

            # make reverse node
            reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
            mc.connectAttr(self.switchAttr, '{}.inputX'.format(reverse))

            for constraint in orientConstraints:
                weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
                mc.connectAttr(self.switchAttr, '{}.{}'.format(constraint, weights[1]))
                mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

            # Setup blend between joint scales

            for i in range(len(self.toeJoints)):
                blendNode = mc.shadingNode('blendColors', asUtility=True,
                                           n='{}_jointScale_blend{}'.format(self.prefix, i))
                mc.connectAttr(self.switchAttr, '{}.blender'.format(blendNode))

                mc.connectAttr('{}.sx'.format(ikRig['joints'][i]), '{}.color1.color1R'.format(blendNode))
                mc.connectAttr('{}.sy'.format(ikRig['joints'][i]), '{}.color1.color1G'.format(blendNode))
                mc.connectAttr('{}.sz'.format(ikRig['joints'][i]), '{}.color1.color1B'.format(blendNode))

                mc.connectAttr('{}.sx'.format(fkRig['joints'][i]), '{}.color2.color2R'.format(blendNode))
                mc.connectAttr('{}.sy'.format(fkRig['joints'][i]), '{}.color2.color2G'.format(blendNode))
                mc.connectAttr('{}.sz'.format(fkRig['joints'][i]), '{}.color2.color2B'.format(blendNode))

                mc.connectAttr('{}.outputR'.format(blendNode), '{}.sx'.format(self.toeJoints[i]))
                mc.connectAttr('{}.outputG'.format(blendNode), '{}.sy'.format(self.toeJoints[i]))
                mc.connectAttr('{}.outputB'.format(blendNode), '{}.sz'.format(self.toeJoints[i]))


            for ctrl in fkRig['controls']:
                mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
            for ctrl in ikRig['controls']:
                mc.connectAttr(self.switchAttr, '{}.v'.format(ctrl.Off))

            # organize
            orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
            mc.parent(orientconstraintGrp, self.baseRig.noXformGrp)

        elif self.legType == 'digi' or self.legType == 'ungu':

            # No need to make any FK controls because the limb module already makes it

            # The reverse foot setup just needs to add heel and toeEnd controls, and foot roll and rock functionality
            self.buildDigitigradeFoot()






    def buildFK(self):

        # duplicate toe joints to make FK joints
        fkJoints = joint.duplicateChain(self.toeJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.fkParentJoint)

        # make controls

        toeCtr = control.Control(prefix = '{}_toesFK'.format(self.prefix), translateTo = fkJoints[0], rotateTo=fkJoints[0],
                                scale = self.rigScale * 0.5, parent = self.fkFootCtr.C, shape='circleX',
                                 lockChannels= ['t', 's', 'v'])

        controls = [toeCtr]

        # connect controls

        mc.parentConstraint(toeCtr.C, fkJoints[0], mo=0)


        return {'joints': fkJoints, 'controls': controls}




    def buildReverseFoot(self):

        # duplicate toe joints to make IK joints
        ikJoints = joint.duplicateChain(self.toeJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.ikParentJoint)

        footJoints = [self.ikParentJoint]
        footJoints.extend(ikJoints)



        # Create control at the ball of the foot
        ballCtr = control.Control(prefix='{}_footRoll'.format(self.prefix), translateTo = footJoints[1],
                                  scale=self.rigScale * 0.5, shape='circleX', offsets=['null', 'zero', 'auto'])

        # Create control at the end of the foot (toe)

        toeEndCtr = control.Control(prefix='{}_toeRoll'.format(self.prefix), translateTo = footJoints[2],
                                    scale=self.rigScale * 0.25, shape='circleX')

        toeCtr = control.Control(prefix='{}_toe'.format(self.prefix), translateTo =footJoints[1],
                                 rotateTo= footJoints[1], scale=self.rigScale * 0.5, shape='circleX', offsets=['null', 'zero', 'auto'])

        # Create control at the heel of the foot

        heelCtr = control.Control(prefix='{}_heelRoll'.format(self.prefix), translateTo = self.heelLoc,
                                  scale=self.rigScale * 0.3, shape='circleX')

        controls = [ballCtr, toeCtr,  toeEndCtr, heelCtr]

        mc.parent(self.heelLoc, heelCtr.C)
        mc.hide(self.heelLoc)

        # Create groups for inner and outer locators

        innerPivot_Grp = self.innerLoc + '_grp'
        outerPivot_Grp = self.outerLoc + '_grp'

        mc.group(em=1, n=innerPivot_Grp)
        mc.group(em=1, n=outerPivot_Grp)

        mc.hide(self.innerLoc)
        mc.hide(self.outerLoc)

        mc.delete(mc.parentConstraint(self.innerLoc, innerPivot_Grp, mo=0))
        mc.delete(mc.parentConstraint(self.outerLoc, outerPivot_Grp, mo=0))

        mc.parent(self.innerLoc, innerPivot_Grp)
        mc.parent(self.outerLoc, outerPivot_Grp)

        # Create IK single chain solver from the ankle to the ball of the foot
        ball_IKH = mc.ikHandle(n='{}_ball_ikh'.format(self.prefix), sol='ikSCsolver', sj = footJoints[0], ee= footJoints[1])[0]
        mc.hide(ball_IKH)

        # Create IK single chain solver from the ball of the foot to the toe
        toe_IKH = mc.ikHandle(n='{}_toe_ikh'.format(self.prefix), sol='ikSCsolver',
                              sj= footJoints[1], ee= footJoints[2])[0]
        mc.hide(toe_IKH)

        # Parent IK handles to controls
        mc.parent(ball_IKH, ballCtr.C)
        mc.parent(toe_IKH, toeCtr.C)

        # Set up parenting structure
        mc.parent(ballCtr.Off, toeEndCtr.C)
        mc.parent(toeCtr.Off, toeEndCtr.C)
        mc.parent(toeEndCtr.Off, heelCtr.C)
        mc.parent(heelCtr.Off, outerPivot_Grp)
        mc.parent(outerPivot_Grp, innerPivot_Grp)

        # Create a group to hold our feet controls and pivots
        footRigGrp = mc.group( n = '{}_feetRigGrp'.format(self.prefix), em = 1)
        mc.delete(mc.pointConstraint(self.reverseFootParent, footRigGrp))
        mc.parent(innerPivot_Grp, footRigGrp)
        mc.parent(footRigGrp, self.reverseFootParent)



        # Delete the constraint on the ikGroup
        mc.delete(mc.listRelatives(self.ikGroupToDrive, c=1, type='parentConstraint')[0])
        # Drive the ikGroup from the limb to the ballCtr.C
        mc.parentConstraint(ballCtr.C, self.ikGroupToDrive, mo = 1)


        # Set up Roll
        roll_attr = 'Roll'
        ball_angle_attr = 'Ball_Roll_Angle'
        toe_angle_attr = 'Toe_Roll_Angle'

        # Create class member so we can access later
        self.BallRollAngleAttr = '{}.{}'.format(self.ikFootCtr.C, ball_angle_attr)
        self.ToeRollAngleAttr = '{}.{}'.format(self.ikFootCtr.C, toe_angle_attr)

       # Set up foot roll connections
        footRoll(
            footCtr = self.ikFootCtr,
            roll_attr = roll_attr,
            ball_angle_attr = ball_angle_attr,
            toe_angle_attr = toe_angle_attr,
            prefix = self.prefix,
            rollAxis = self.rollAxis,
            heelCtr = heelCtr,
            ballCtr = ballCtr,
            toeEndCtr = toeEndCtr,
        )

        # Set up rock
        footRock(
            footCtr = self.ikFootCtr,
            rockAxis = self.rockAxis,
            outer = outerPivot_Grp,
            inner = innerPivot_Grp
        )


        # Set some class properties we can call later

        return {'joints': ikJoints, 'controls': controls}


    def buildDigitigradeFoot(self):


        # Duplicate toe end joint

        toeEndIKJoint = joint.duplicateChain(self.toeJoints[-1], 'jnt', 'IK_jnt')[0]
        mc.parent(toeEndIKJoint, self.ikParentJoint)

        #footJoints = [self.ikParentJoint]
        #footJoints.extend(ikJoints)


        # Create toeEnd CTR and heel CTR
        toeEndCtr = control.Control(prefix='{}_toeRoll'.format(self.prefix), translateTo= toeEndIKJoint,
                                    scale=self.rigScale * 0.25, shape='circleX')

        heelCtr = control.Control(prefix='{}_heelRoll'.format(self.prefix), translateTo= self.heelLoc,
                                  scale = self.rigScale * 0.3, shape='circleX')

        mc.parent(self.heelLoc, heelCtr.C)
        mc.hide(self.heelLoc)


        # Create groups for inner and outer locators
        innerPivot_Grp = self.innerLoc + '_grp'
        outerPivot_Grp = self.outerLoc + '_grp'

        mc.group(em=1, n=innerPivot_Grp)
        mc.group(em=1, n=outerPivot_Grp)

        mc.hide(self.innerLoc)
        mc.hide(self.outerLoc)

        mc.delete(mc.parentConstraint(self.innerLoc, innerPivot_Grp, mo=0))
        mc.delete(mc.parentConstraint(self.outerLoc, outerPivot_Grp, mo=0))

        mc.parent(self.innerLoc, innerPivot_Grp)
        mc.parent(self.outerLoc, outerPivot_Grp)

        # Delete constraint on ik toe joint
        mc.delete(mc.listRelatives(self.ikParentJoint, c=1, type='orientConstraint')[0])


        # Create IK single chain solver from the ball of the foot to the toe
        toe_IKH = mc.ikHandle(n='{}_toe_ikh'.format(self.prefix), sol='ikSCsolver',
                              sj= self.ikParentJoint, ee= toeEndIKJoint)[0]
        mc.hide(toe_IKH)

        # Set up parenting structure
        mc.parent(toe_IKH, self.toeIKCtr.C)
        mc.parent(self.ankleRollDriver, toeEndCtr.C)
        mc.parent(self.toeIKCtr.Off, toeEndCtr.C)
        mc.parent(toeEndCtr.Off, heelCtr.C)
        mc.parent(heelCtr.Off, outerPivot_Grp)
        mc.parent(outerPivot_Grp, innerPivot_Grp)

        # Create a group to hold our feet controls and pivots
        #footRigGrp = mc.group(n='{}_feetRigGrp'.format(self.prefix), em=1)
        #mc.delete(mc.pointConstraint(self.ikParentCtr.C, footRigGrp))
        #mc.parent(innerPivot_Grp, footRigGrp)
        mc.parent(innerPivot_Grp, self.ikFootCtr.C)

        # Delete the constraint on the ikGroup
        #mc.delete(mc.listRelatives(self.ikGroupToDrive, c=1, type='parentConstraint')[0])
        # Drive the ikGroup from the limb to the ballCtr.C
        #mc.parentConstraint(ballCtr.C, self.ikGroupToDrive, mo=1)



        # Set up foot roll
        roll_attr = 'Roll'
        ball_angle_attr = 'Ball_Roll_Angle'
        toe_angle_attr = 'Toe_Roll_Angle'

        # Create class member so we can access later
        self.BallRollAngleAttr = '{}.{}'.format(self.ikFootCtr.C, ball_angle_attr)
        self.ToeRollAngleAttr = '{}.{}'.format(self.ikFootCtr.C, toe_angle_attr)


        footRoll(
            footCtr = self.ikFootCtr,
            roll_attr = roll_attr,
            ball_angle_attr = ball_angle_attr,
            toe_angle_attr = toe_angle_attr,
            prefix = self.prefix,
            rollAxis = self.rollAxis,
            heelCtr = heelCtr,
            ballCtr = self.ballCtr,
            toeEndCtr = toeEndCtr,
        )

        # Set up rock
        footRock(
            footCtr = self.ikFootCtr,
            rockAxis = self.rockAxis,
            outer = outerPivot_Grp,
            inner = innerPivot_Grp
        )





    def setInitialValues(self,
                         BallRollAngle = 45 ,
                         ToeRollAngle = 60,
                         ):

        mc.setAttr(self.BallRollAngleAttr, BallRollAngle)
        mc.setAttr(self.ToeRollAngleAttr, ToeRollAngle)





def footRoll(
            footCtr,
            roll_attr,
            ball_angle_attr,
            toe_angle_attr,
            prefix,
            rollAxis,
            heelCtr,
            ballCtr,
            toeEndCtr,

):


    # Add roll attributes to foot control
    mc.addAttr(footCtr.C, ln=roll_attr, at='double', dv=0, k=1)

    mc.addAttr(footCtr.C, ln=ball_angle_attr, at='double', dv=0, k=1)
    mc.setAttr('{}.{}'.format(footCtr.C, ball_angle_attr), 45)

    mc.addAttr(footCtr.C, ln=toe_angle_attr, at='double', dv=0, k=1)
    mc.setAttr('{}.{}'.format(footCtr.C, toe_angle_attr), 60)

    # Connect heel pivot
    heelRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                    n='{}_heelRoll_clamp'.format(prefix))

    mc.connectAttr('{}.{}'.format(footCtr.C, roll_attr), '{}.inputR'.format(heelRoll_clamp))
    mc.setAttr('{}.minR'.format(heelRoll_clamp), -90)
    mc.setAttr('{}.maxR'.format(heelRoll_clamp), 0)

    if 'x' in rollAxis:
        rollAxisAttr = 'rotateX'

    elif 'y' in rollAxis:
        rollAxisAttr = 'rotateY'

    elif 'z' in rollAxis:
        rollAxisAttr = 'rotateZ'

    rollUpNegative = False
    if '-' in rollAxis:
        rollUpNegative = True

    mc.connectAttr('{}.outputR'.format(heelRoll_clamp), '{}.{}'.format(heelCtr.Off, rollAxisAttr))

    # Connect toe pivot
    toeRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                   n='{}_toeRoll_clamp'.format(prefix))

    toeRoll_setRange = mc.shadingNode('setRange', asUtility=True,
                                      n='{}_toeRoll_setRange'.format(prefix))

    toeRoll_mult_percent = mc.shadingNode('multiplyDivide', asUtility=True,
                                          n='{}_toeRoll_mult_percent'.format(prefix))

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
                                             n='{}_toeRoll_mult_negate'.format(prefix))

        mc.connectAttr('{}.outputX'.format(toeRoll_mult_percent), '{}.input1X'.format(toeRoll_mult_negate))
        mc.setAttr('{}.input2X'.format(toeRoll_mult_negate), -1)
        mc.setAttr('{}.operation'.format(toeRoll_mult_negate), 1)

        toeRollDriver = '{}.outputX'.format(toeRoll_mult_negate)



    else:
        toeRollDriver = '{}.outputX'.format(toeRoll_mult_percent)

    # Feed the out value of our multiply node to toe_loc rotate axis
    mc.connectAttr(toeRollDriver, '{}.{}'.format(toeEndCtr.Off, rollAxisAttr))

    # Connect ball pivot
    ballRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                    n='{}_ballRoll_clamp'.format(prefix))

    ballRoll_setRange = mc.shadingNode('setRange', asUtility=True,
                                       n='{}_ballRoll_setRange'.format(prefix))

    toeRoll_reverse = mc.shadingNode('reverse', asUtility=True,
                                     n='{}_toeRoll_reverse'.format(prefix))

    ballRoll_combine_percentages = mc.shadingNode('multiplyDivide', asUtility=True,
                                                  n='{}_ballRoll_combine_percentages'.format(prefix))

    ballRoll_mult_percent = mc.shadingNode('multiplyDivide', asUtility=True,
                                           n='{}_ballRoll_mult_percent'.format(prefix))

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
                                              n='{}_ballRoll_mult_negate'.format(prefix))

        mc.connectAttr('{}.outputX'.format(ballRoll_mult_percent), '{}.input1X'.format(ballRoll_mult_negate))
        mc.setAttr('{}.input2X'.format(ballRoll_mult_negate), -1)
        mc.setAttr('{}.operation'.format(ballRoll_mult_negate), 1)

        ballRollDriver = '{}.outputX'.format(ballRoll_mult_negate)



    else:
        ballRollDriver = '{}.outputX'.format(ballRoll_mult_percent)

    # Feed the out value of our multiply node to ballCtr rotation
    mc.connectAttr(ballRollDriver, '{}.{}'.format(ballCtr.Offsets[1], rollAxisAttr))


def footRock(
            footCtr,
            rockAxis,
            outer,
            inner
            ):

    # Set up rock
    rock_attr = 'Rock'
    mc.addAttr(footCtr.C, ln=rock_attr, at='double', dv=0, k=1)

    if 'x' in rockAxis:
        rockAxisAttr = 'rotateX'

    elif 'y' in rockAxis:
        rockAxisAttr = 'rotateY'

    elif 'z' in rockAxis:
        rockAxisAttr = 'rotateZ'

    rotateOutNegative = False

    if '-' in rockAxis:
        rotateOutNegative = True

    if rotateOutNegative:
        rotateOuterDirection = -1
        rotateInnerDirection = 1
    else:
        rotateOuterDirection = 1
        rotateInnerDirection = -1

    # Set up SDK to rock outer pivot
    mc.setDrivenKeyframe('{}.{}'.format(outer, rockAxisAttr),
                         currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                         driverValue=0,
                         value=0,
                         inTangentType='linear',
                         outTangentType='linear')

    mc.setDrivenKeyframe('{}.{}'.format(outer, rockAxisAttr),
                         currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                         driverValue=90,
                         value=90 * rotateOuterDirection,
                         inTangentType='linear',
                         outTangentType='linear')

    # Set up SDK to rock inner pivot
    mc.setDrivenKeyframe('{}.{}'.format(inner, rockAxisAttr),
                         currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                         driverValue=0,
                         value=0,
                         inTangentType='linear',
                         outTangentType='linear')

    mc.setDrivenKeyframe('{}.{}'.format(inner, rockAxisAttr),
                         currentDriver='{}.{}'.format(footCtr.C, rock_attr),
                         driverValue=-90,
                         value=90 * rotateInnerDirection,
                         inTangentType='linear',
                         outTangentType='linear')
