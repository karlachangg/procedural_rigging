"""
biped rig setup
main module
"""

from . import char

from rigLib.rig import spine
from rigLib.rig import neck
from rigLib.rig import ikChain
from rigLib.rig import legFKIK
from rigLib.rig import armFKIK
from rigLib.rig import hand


import maya.cmds as mc


class Biped(char.Character):

    def __init__(self, characterName):
        char.Character.__init__(self, characterName)

    def build(self):

        self.setup()
        self.makeControlSetup(self.baseRig)
        self.deform()
        self.setInitialPose()

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

        leftLegRig = legFKIK.Leg(
            type = '2bones',
            legJoints = legJoints,
            toeJoints = toeJoints,
            hipPivotJoint = hipPivotJoint,
            heelLoc = heel_loc,
            innerLoc = inner_loc,
            outerLoc = outer_loc,
            prefix = 'leg',
            side = 'l',
            kneeDirection = 'z',
            moveSwitchCtr = 'x',
            rigScale = self.sceneScale,
            bendy= True,
            baseRig = baseRig
            )

        leftLegRig.build()

        rightLegRig = legFKIK.Leg(
            type = '2bones',
            legJoints = legJoints,
            toeJoints = toeJoints,
            heelLoc=heel_loc,
            innerLoc=inner_loc,
            outerLoc=outer_loc,
            hipPivotJoint = hipPivotJoint,
            prefix = 'leg',
            side = 'r',
            kneeDirection = '-z',
            moveSwitchCtr='-x',
            rigScale = self.sceneScale,
            bendy = True,
            baseRig = baseRig
            )

        rightLegRig.build()

        # left arm
        armJoints = ['shoulder_jnt', 'elbow_jnt', 'wrist_jnt']
        scapulaJnt = 'clavicle_jnt'

        self.leftArmRig = armFKIK.Arm(
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

        self.rightArmRig = armFKIK.Arm(
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

    def setInitialPose(self):

        #  Set arms to a T pose
        self.leftArmRig.setTpose()
        self.rightArmRig.setTpose()













