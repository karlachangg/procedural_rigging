"""
biped rig setup
main module
"""

from . import char

from rigLib.rig import spine
from rigLib.rig import neck
from rigLib.rig import ikChain
from rigLib.rig import leg
from rigLib.rig import arm
from rigLib.rig import foot
from rigLib.rig import hand
from rigLib.base import control



import maya.cmds as mc


class Biped(char.Character):

    def __init__(self, characterName):
        char.Character.__init__(self, characterName)

    def build(self):

        self.setup()
        self.makeControlSetup(self.baseRig)
        self.deform()
        self.adjustControlShapes()
        self.setInitialSettings()

    def makeControlSetup(self, baseRig):
        """
           make control setup
           """

        # spine
        spineJoints = ['spine_0_jnt', 'spine_1_jnt', 'spine_2_jnt', 'spine_3_jnt', 'spine_4_jnt', 'spine_5_jnt',
                       'spine_6_jnt']
        rootJoint = 'root_jnt'
        chestJoint = 'chest_jnt'

        self.spineRig = spine.Spine(
            type = 'ikfk',
            spineJoints = spineJoints,
            rootJnt = rootJoint,
            chestJnt = chestJoint,
            spineCurve='spine_curve',
            prefix = 'spine',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.spineRig.build()


        # neck
        neckJoints = ['neck_1_jnt', 'neck_2_jnt', 'neck_3_jnt', 'neck_4_jnt', 'neck_5_jnt']
        headJoint = 'head_jnt'

        self.neckRig = neck.Neck(
            neckJoints = neckJoints,
            headJnt = headJoint,
            neckCurve ='neck_curve',
            prefix = 'neck',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )
        self.neckRig.build()

        # attach neck to spine
        mc.parentConstraint(self.spineRig.rigParts['chestAttachGrp'], self.neckRig.rigParts['baseAttachGrp'], mo=1)

        # left leg
        legJoints = ['hip_jnt', 'knee_jnt', 'ankle_jnt']
        toeJoints = ['toe_jnt', 'toe_end_jnt']
        hipPivotJoint = 'hip_piv_jnt'
        heel_loc = 'heel_loc'
        inner_loc = 'foot_inner_loc'
        outer_loc = 'foot_outer_loc'

        self.leftLegRig = leg.Leg(
            type = '2bones',
            legJoints = legJoints,
            hipPivotJoint = hipPivotJoint,
            prefix = 'leg',
            side = 'l',
            kneeDirection = 'z',
            moveSwitchCtr = 'x',
            rigScale = self.sceneScale,
            bendy= True,
            baseRig = baseRig
            )

        self.leftLegRig.build()

        self.rightLegRig = leg.Leg(
            type = '2bones',
            legJoints = legJoints,
            hipPivotJoint = hipPivotJoint,
            prefix = 'leg',
            side = 'r',
            kneeDirection = '-z',
            moveSwitchCtr='-x',
            rigScale = self.sceneScale,
            bendy = True,
            baseRig = baseRig
            )

        self.rightLegRig.build()

        # left arm
        armJoints = ['shoulder_jnt', 'elbow_jnt', 'wrist_jnt']
        scapulaJnt = 'clavicle_jnt'

        self.leftArmRig = arm.Arm(
            armJoints = armJoints,
            scapulaJoint = scapulaJnt,
            prefix = 'arm',
            side = 'l',
            bendy = True,
            ikCtrOrient='bone',
            elbowDirection='-z',
            forwardAxis='x',
            moveSwitchCtr='x, y',
            rigScale = self.sceneScale,
            baseRig = baseRig
            )

        self.leftArmRig.build()

        # attach left arm to spine
        mc.parentConstraint(self.spineRig.rigParts['chestAttachGrp'], self.leftArmRig.rigParts['bodyAttachGrp'], mo=1)

        self.rightArmRig = arm.Arm(
            armJoints = armJoints,
            scapulaJoint = scapulaJnt,
            prefix = 'arm',
            side = 'r',
            bendy = True,
            elbowDirection = 'z',
            forwardAxis='x',
            moveSwitchCtr = '-x, y',
            rigScale = self.sceneScale,
            baseRig = baseRig
            )
        self.rightArmRig.build()

        # attach right arm to spine
        mc.parentConstraint(self.spineRig.rigParts['chestAttachGrp'], self.rightArmRig.rigParts['bodyAttachGrp'], mo=1)

        # Left hand
        fingerJoints = ['finger_index_0_jnt', 'finger_middle_0_jnt', 'finger_ring_0_jnt', 'finger_pinky_0_jnt', 'thumb_0_jnt']
        metaJoints = True
        innerCupJoint = 'cup_inner_jnt'
        outerCupJoint = 'cup_jnt'

        leftHandRig = hand.Hand(
            fingerBaseJoints = fingerJoints,
            metaJoints = metaJoints,
            innerCupJoint = innerCupJoint,
            outerCupJoint = outerCupJoint,
            includeFingerEnds = False,
            handAttachGrp = self.leftArmRig.rigParts['handAttachGrp'],
            prefix = 'hand',
            side='l',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )
        leftHandRig.build()

        # Right hand

        rightHandRig = hand.Hand(
            fingerBaseJoints = fingerJoints,
            metaJoints = metaJoints,
            innerCupJoint = innerCupJoint,
            outerCupJoint = outerCupJoint,
            includeFingerEnds = False,
            handAttachGrp = self.rightArmRig.rigParts['handAttachGrp'],
            prefix = 'hand',
            side = 'r',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )
        rightHandRig.build()

        # left foot

        toeJoints = ['toe_jnt', 'toe_end_jnt']
        heelLoc = 'heel_loc'
        innerLoc = 'foot_inner_loc'
        outerLoc = 'foot_outer_loc'

        self.leftFootRig = foot.Foot(

            toeJoints = toeJoints,
            heelLoc = heelLoc,
            innerLoc = innerLoc,
            outerLoc = outerLoc,

            fkAnkleJoint = self.leftLegRig.rigParts['fkJoints'][-1],
            ikAnkleJoint  = self.leftLegRig.rigParts['ikJoints'][-1],
            fkFootCtr  = self.leftLegRig.rigParts['fkControls'][-1],
            ikFootCtr = self.leftLegRig.rigParts['ikControl'],
            ikParentCtr = self.leftLegRig.rigParts['ikControl'],
            ikGroupToDrive = self.leftLegRig.rigParts['reverseFootDriven'],
            switchAttr = self.leftLegRig.rigParts['FKIKSwitchAttr'],

            prefix = 'foot',
            side = 'l',
            rollAxis = 'x',
            rockAxis = '-z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.leftFootRig.build()

        # right foot

        self.rightFootRig = foot.Foot(

            toeJoints = toeJoints,
            heelLoc = heelLoc,
            innerLoc = innerLoc,
            outerLoc = outerLoc,

            fkAnkleJoint = self.rightLegRig.rigParts['fkJoints'][-1],
            ikAnkleJoint = self.rightLegRig.rigParts['ikJoints'][-1],
            fkFootCtr = self.rightLegRig.rigParts['fkControls'][-1],
            ikFootCtr = self.rightLegRig.rigParts['ikControl'],
            ikParentCtr = self.rightLegRig.rigParts['ikControl'],
            ikGroupToDrive = self.rightLegRig.rigParts['reverseFootDriven'],
            switchAttr = self.rightLegRig.rigParts['FKIKSwitchAttr'],

            prefix = 'foot',
            side = 'r',
            rollAxis = 'x',
            rockAxis = 'z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.rightFootRig.build()







    def setInitialSettings(self):

        #  Set arms to a T pose
        #self.leftArmRig.setTpose()
        #self.rightArmRig.setTpose()

        self.leftArmRig.setInitialValues(FKIKMode=1, Stretchy= 1)


        self.rightArmRig.setInitialValues(FKIKMode=1, Stretchy=1 )





    def adjustControlShapes(self):

        # head control
        control._translateCtrlShape(self.neckRig.rigParts['controls'][1], axis='y', value=0.613)
        control._translateCtrlShape(self.neckRig.rigParts['controls'][1], axis='z', value= 0.292)
        # neck control
        control._scaleCtrlShape(self.neckRig.rigParts['controls'][0], axis='x, y, z', value=0.75)

        # Spine
        # hip IK
        control._scaleCtrlShape(self.spineRig.rigParts['ikControls'][0], axis='y', value = 0.5)

        # chest IK
        control._scaleCtrlShape(self.spineRig.rigParts['ikControls'][2], axis='y', value = 0.5)

        # middle IK
        control._scaleCtrlShape(self.spineRig.rigParts['ikControls'][1], axis='y', value=0.1)

        # switch control
        control._translateCtrlShape(self.spineRig.rigParts['switchControl'], axis='x', value= 1.0)

        # left leg
        #bendControls
        control._scaleCtrlShape(self.leftLegRig.rigParts['bendControls'][0], axis='x,y,z', value=2)
        control._scaleCtrlShape(self.leftLegRig.rigParts['bendControls'][1], axis = 'x,y,z', value = 2)
        control._scaleCtrlShape(self.leftLegRig.rigParts['bendControls'][2], axis='x,y,z', value=2)
        control._scaleCtrlShape(self.leftLegRig.rigParts['bendControls'][3], axis='x,y,z', value=2)

        # foot ik control
        control._translateCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='y', value = -0.343)
        control._translateCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='z', value= 0.832)
        control._scaleCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='y', value=0.5)
        control._scaleCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='z', value= 1.2)

        # right leg
        # bendControls
        control._scaleCtrlShape(self.rightLegRig.rigParts['bendControls'][0], axis='x,y,z', value=2)
        control._scaleCtrlShape(self.rightLegRig.rigParts['bendControls'][1], axis='x,y,z', value=2)
        control._scaleCtrlShape(self.rightLegRig.rigParts['bendControls'][2], axis='x,y,z', value=2)
        control._scaleCtrlShape(self.rightLegRig.rigParts['bendControls'][3], axis='x,y,z', value=2)

        # foot ik control
        control._translateCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='y', value = -0.343)
        control._translateCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='z', value=0.832)
        control._scaleCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='y', value=0.5)
        control._scaleCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='z', value=1.2)

        # left arm
        # hand ik control
        control._translateCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='x', value = 0.373 )
        control._translateCtrlShape(self.leftArmRig.rigParts['ikControl'], axis= 'y', value = -0.813)
        control._translateCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='z', value = 0.34)
        control._scaleCtrlShape(self.leftArmRig.rigParts['ikControl'], axis = 'x,y,z', value = 0.8)

        # right arm
        # hand ik control
        control._translateCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='x', value = -0.373)
        control._translateCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='y', value =  -0.813)
        control._translateCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='z', value = 0.34)
        control._scaleCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='x,y, z', value = 0.8)













