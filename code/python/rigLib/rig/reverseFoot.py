"""
reverse foot rig
"""

import maya.cmds as mc


from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name

class Foot():

    def __init__(self,
            footJoints,
            heelLoc,
            innerLoc,
            outerLoc,
            legIKH,
            footCtr,
            parentCtr,
            measureLeg_end_node,
            prefix = 'foot',
            side = 'l',
            forwardAxis='x',
            rollAxis='x',
            rockAxis='z',
            rigScale = 1.0,
            baseRig = None
            ):

        """
        :param legJoints: list(str), hip - knee - ankle
        :param toeJoints: list(str), toe - toeEnd
        :param hipPivotJoint: str, hip position joint
        :param heelLoc: str, heel position locator
        :param prefix: str, prefix to name new objects, INCLUDING SIDE
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """

        self.footJoints = footJoints
        self.heelLoc = heelLoc
        self.innerLoc = innerLoc
        self.outerLoc = outerLoc
        self.legIKH = legIKH
        self.footCtr = footCtr
        self.parentCtr = parentCtr
        self.measureLeg_end_node = measureLeg_end_node
        self.rollAxis = rollAxis
        self.rockAxis = rockAxis
        self.prefix =  prefix
        self.side = side
        self.forwardAxis = forwardAxis
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        #self.rigmodule = module.Module(prefix=self.prefix, baseObj=self.baseRig)

    def build(self):

        # Create control at the ball of the foot
        ballCtr = control.Control(prefix='{}_footRoll'.format(self.prefix), translateTo = self.footJoints[1],
                                  scale=self.rigScale * 0.5, shape='circleX')

        # Create control at the end of the foot (toe)

        toeEndCtr = control.Control(prefix='{}_toeRoll'.format(self.prefix), translateTo = self.footJoints[2],
                                    scale=self.rigScale * 0.25, shape='circleX')

        toeCtr = control.Control(prefix='{}_toe'.format(self.prefix), translateTo = self.footJoints[1],
                                 rotateTo= self.footJoints[1], scale=self.rigScale * 0.5, shape='circleX')

        # Create control at the heel of the foot

        heelCtr = control.Control(prefix='{}_heelRoll'.format(self.prefix), translateTo = self.heelLoc,
                                  scale=self.rigScale * 0.3, shape='circleX')

        controls = [ballCtr, toeEndCtr, toeCtr, heelCtr]

        mc.parent(self.heelLoc, heelCtr.C)
        mc.hide(self.heelLoc)

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
        ball_IKH = mc.ikHandle(n='{}_ball_ikh'.format(self.prefix), sol='ikSCsolver', sj = self.footJoints[0], ee= self.footJoints[1])[0]
        mc.hide(ball_IKH)

        # Create IK single chain solver from the ball of the foot to the toe
        toe_IKH = mc.ikHandle(n='{}_toe_ikh'.format(self.prefix), sol='ikSCsolver',
                              sj=self.footJoints[1], ee=self.footJoints[2])[0]
        mc.hide(toe_IKH)

        # Parent IK handles to controls
        mc.parent(ball_IKH, ballCtr.C)
        mc.parent(toe_IKH, toeCtr.C)

        for ikHandle in self.legIKH:
            mc.parent(ikHandle, ballCtr.C)

        # Set up parenting structure
        mc.parent(ballCtr.Off, toeEndCtr.C)
        mc.parent(toeCtr.Off, toeEndCtr.C)
        mc.parent(toeEndCtr.Off, heelCtr.C)
        mc.parent(heelCtr.Off, outerPivot_Grp)
        mc.parent(outerPivot_Grp, innerPivot_Grp)

        # Create a group to hold our feet controls and pivots
        footRigGrp = mc.group( n = '{}_feetRigGrp'.format(self.prefix), em = 1)
        mc.delete(mc.pointConstraint(self.parentCtr.C, footRigGrp))
        mc.parent(innerPivot_Grp, footRigGrp)
        mc.parent(footRigGrp, self.parentCtr.C)

        # parent the node we use to measure the leg length to the ball control. Because this will give us a more accurate reading
        mc.parent(self.measureLeg_end_node, ballCtr.C)

        # Set up Roll
        roll_attr = 'Roll'
        mc.addAttr(self.footCtr.C, ln=roll_attr, at='double', dv=0, k=1)

        ball_angle_attr = 'Ball_Roll_Angle'
        mc.addAttr(self.footCtr.C, ln=ball_angle_attr, at='double', dv=0, k=1)
        mc.setAttr('{}.{}'.format(self.footCtr.C, ball_angle_attr), 45)

        # Create class member so we can access later
        self.BallRollAngleAttr = '{}.{}'.format(self.footCtr.C, ball_angle_attr)

        toe_angle_attr = 'Toe_Roll_Angle'
        mc.addAttr(self.footCtr.C, ln=toe_angle_attr, at='double', dv=0, k=1)
        mc.setAttr('{}.{}'.format(self.footCtr.C, toe_angle_attr), 60)

        # Create class member so we can access later
        self.ToeRollAngleAttr = '{}.{}'.format(self.footCtr.C, toe_angle_attr)

        # Connect heel pivot
        heelRoll_clamp = mc.shadingNode('clamp', asUtility=True,
                                        n='{}_heelRoll_clamp'.format(self.prefix))

        mc.connectAttr('{}.{}'.format(self.footCtr.C, roll_attr), '{}.inputR'.format(heelRoll_clamp))
        mc.setAttr('{}.minR'.format(heelRoll_clamp), -90)
        mc.setAttr('{}.maxR'.format(heelRoll_clamp), 0)

        if self.rollAxis == 'x' or self.rollAxis == '-x':
            rollAxisAttr = 'rotateX'

        elif self.rollAxis == 'y' or self.rollAxis == '-y':
            rollAxisAttr = 'rotateY'

        elif self.rollAxis == 'z' or self.rollAxis == '-z':
            rollAxisAttr = 'rotateZ'

        rollUpNegative = False
        if '-' in self.rollAxis:
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
        mc.connectAttr('{}.{}'.format(self.footCtr.C, roll_attr), '{}.inputR'.format(toeRoll_clamp))
        # Ball angle attr goes into our clamp minimum
        mc.connectAttr('{}.{}'.format(self.footCtr.C, ball_angle_attr), '{}.minR'.format(toeRoll_clamp))
        # Toe angle attr goes into our clamp maximum
        mc.connectAttr('{}.{}'.format(self.footCtr.C, toe_angle_attr), '{}.maxR'.format(toeRoll_clamp))

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
            mc.setAttr('{}.input2X'.format(toeRoll_mult_negate), -1)
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
        mc.connectAttr('{}.{}'.format(self.footCtr.C, roll_attr), '{}.inputR'.format(ballRoll_clamp))
        # Clamp minimum set to zero
        mc.setAttr('{}.minR'.format(ballRoll_clamp), 0)
        # Ball angle attr goes into our clamp maximum
        mc.connectAttr('{}.{}'.format(self.footCtr.C, ball_angle_attr), '{}.maxR'.format(ballRoll_clamp))

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
            mc.setAttr('{}.input2X'.format(ballRoll_mult_negate), -1)
            mc.setAttr('{}.operation'.format(ballRoll_mult_negate), 1)

            ballRollDriver = '{}.outputX'.format(ballRoll_mult_negate)



        else:
            ballRollDriver = '{}.outputX'.format(ballRoll_mult_percent)

        # Feed the out value of our multiply node to ballCtr rotation
        mc.connectAttr(ballRollDriver, '{}.{}'.format(ballCtr.Off, rollAxisAttr))



        # Set up rock
        rock_attr = 'Rock'
        mc.addAttr(self.footCtr.C, ln=rock_attr, at='double', dv=0, k=1)

        if self.rockAxis == 'x' or self.rockAxis == '-x':
            rockAxisAttr = 'rotateX'

        elif self.rockAxis == 'y' or self.rockAxis == '-y':
            rockAxisAttr = 'rotateY'

        elif self.rockAxis == 'z' or self.rockAxis == '-z':
            rockAxisAttr = 'rotateZ'

        rotateOutNegative = False

        if '-' in self.rockAxis:
            rotateOutNegative = True

        if rotateOutNegative:
            rotateOuterDirection = -1
            rotateInnerDirection = 1
        else:
            rotateOuterDirection = 1
            rotateInnerDirection = -1



        if self.side == 'l':

            # Set up SDK to rock outer pivot
            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue = 90,
                                 value = 90 * rotateOuterDirection,
                                 inTangentType='linear',
                                 outTangentType='linear')

            # Set up SDK to rock inner pivot
            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue= -90,
                                 value = 90 * rotateInnerDirection,
                                 inTangentType='linear',
                                 outTangentType='linear')

        elif self.side == 'r':

            # Set up SDK to rock outer pivot
            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(outerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue = 90,
                                 value = 90 * rotateOuterDirection,
                                 inTangentType='linear',
                                 outTangentType='linear')

            # Set up SDK to rock inner pivot
            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue=0,
                                 value=0,
                                 inTangentType='linear',
                                 outTangentType='linear')

            mc.setDrivenKeyframe('{}.{}'.format(innerPivot_Grp, rockAxisAttr),
                                 currentDriver='{}.{}'.format(self.footCtr.C, rock_attr),
                                 driverValue = -90,
                                 value = 90 * rotateInnerDirection * -1,
                                 inTangentType='linear',
                                 outTangentType='linear')

        # Set some class properties we can call later

        self.controls = controls




    def setInitialValues(self,
                         BallRollAngle = 45 ,
                         ToeRollAngle = 60,
                         ):

        mc.setAttr(self.BallRollAngleAttr, BallRollAngle)
        mc.setAttr(self.ToeRollAngleAttr, ToeRollAngle)