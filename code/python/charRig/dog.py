"""
dog archetype rig setup
main module
"""


from . import char
from . import project
from rigLib.rig import quadSpine
from rigLib.rig import neck
from rigLib.rig import tail
from rigLib.rig import ikChain
from rigLib.rig import leg
from rigLib.rig import arm
from rigLib.rig import limb
from rigLib.rig import hand
from rigLib.rig import bendyLimb
from rigLib.rig import foot

from rigLib.utils import joint

import maya.cmds as mc

class Dog(char.Character):

    def __init__(self, characterName, sceneScale = project.sceneScale):
        char.Character.__init__(self, characterName, sceneScale)

    def build(self):

        self.setup()
        self.makeControlSetup(self.baseRig)
        self.deform()
        #self.setInitialPose()

    def makeControlSetup(self, baseRig):

        # Spine
        spineJoints = ['spine_0_jnt', 'spine_1_jnt', 'spine_2_jnt', 'spine_3_jnt', 'spine_4_jnt', 'spine_5_jnt',
                       'spine_6_jnt']
        rootJoint = 'root_jnt'
        chestJoint = 'chest_jnt'

        self.spineRig = quadSpine.QuadSpine(
            spineJoints = spineJoints,
            rootJnt = rootJoint,
            chestJnt = chestJoint,
            spineCurve = 'spine_curve',
            prefix = 'spine',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.spineRig.build()


        # Neck

        neckJoints = ['neck_1_jnt', 'neck_2_jnt', 'neck_3_jnt', 'neck_4_jnt', 'neck_5_jnt']
        headJoint = 'head_jnt'

        self.neckRig = neck.Neck(
            neckJoints=neckJoints,
            headJnt=headJoint,
            neckCurve='neck_curve',
            prefix = 'neck',
            forwardAxis = 'x',
            upAxis = 'z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )
        self.neckRig.build()

        # attach neck to spine
        mc.parentConstraint(self.spineRig.rigParts['chestAttachGrp'], self.neckRig.rigParts['baseAttachGrp'], mo=1)

        # Tail
        tailJoints = ['tail_1_jnt', 'tail_2_jnt', 'tail_3_jnt', 'tail_4_jnt', 'tail_5_jnt',
                      'tail_6_jnt', 'tail_7_jnt']

        tailRig = tail.Tail(
             tailJoints = tailJoints,
             prefix = 'tail',
             bendy = False,
             rigScale = self.sceneScale,
             baseRig = baseRig,
             )

        tailRig.build()

        # attach tail to spine
        mc.parentConstraint(self.spineRig.rigParts['hipsAttachGrp'], tailRig.rigParts['bodyAttachGrp'], mo=1)


        # Left Front Leg

        armJoints = ['shoulder_jnt', 'elbow_jnt', 'wrist_jnt']
        scapulaJnt = 'scapula_jnt'


        self.leftArmRig = limb.Limb(
            type = 'planti',
            joints = armJoints,
            scapulaJoint = scapulaJnt,
            prefix='frontLeg',
            side='l',
            bendy = False,
            elbowDirection= 'y',
            forwardAxis= '-x',
            moveSwitchCtr='x',
            ikCtrOrient='world',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        self.leftArmRig.build()
        self.leftArmRig.setInitialValues(FKIKMode= 1, Stretchy= 1)

        # attach left arm to spine
        mc.parentConstraint(self.spineRig.rigParts['chestAttachGrp'], self.leftArmRig.rigParts['bodyAttachGrp'], mo=1)


        self.rightArmRig = limb.Limb(
            type = 'planti',
            joints=armJoints,
            scapulaJoint=scapulaJnt,
            prefix='frontLeg',
            side='r',
            bendy=False,
            elbowDirection='-y',
            forwardAxis='x',
            moveSwitchCtr='-x',
            ikCtrOrient='world',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        self.rightArmRig.build()
        self.rightArmRig.setInitialValues(FKIKMode=1, Stretchy=1)

        # attach right arm to spine
        mc.parentConstraint(self.spineRig.rigParts['chestAttachGrp'], self.rightArmRig.rigParts['bodyAttachGrp'], mo=1)

        # Left Back Leg

        legJoints = ['hip_jnt', 'knee_jnt', 'ankle_jnt']
        #footJoints = ['backFoot_jnt', 'backToe_jnt', 'backToeEnd_jnt']
        #hipPivotJoint = 'hip_pivot_jnt'

        #heel_loc = 'backFoot_heel'
        #inner_loc = 'backFoot_inner'
        #outer_loc = 'backFoot_outer'

        self.leftLegRig = limb.Limb(
            type = 'planti',
            joints = legJoints,
            prefix = 'backLeg',
            side = 'l',
            bendy = False,
            elbowDirection = '-y',
            forwardAxis='-x',
            moveSwitchCtr = 'x',
            ikCtrOrient = 'world',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        self.leftLegRig.build()

        # attach left leg to spine
        mc.parentConstraint(self.spineRig.rigParts['hipsAttachGrp'], self.leftLegRig.rigParts['bodyAttachGrp'], mo=1)

        self.leftLegRig.setInitialValues(
                             FKIKMode  = 1,
                             Stretchy = 1
                             )

        # Right Back Leg
        self.rightLegRig = limb.Limb(
            type='planti',
            joints=legJoints,
            prefix='backLeg',
            side='r',
            bendy=False,
            elbowDirection = 'y',
            forwardAxis = 'x',
            moveSwitchCtr='-x',
            ikCtrOrient='world',
            rigScale = self.sceneScale,
            baseRig=baseRig
        )

        self.rightLegRig.build()

        # attach left leg to spine
        mc.parentConstraint(self.spineRig.rigParts['hipsAttachGrp'], self.rightLegRig.rigParts['bodyAttachGrp'], mo=1)

        self.rightLegRig.setInitialValues(
                        FKIKMode = 1,
                        Stretchy = 1,
                        )

        # left front foot
        toeJoints = ['fingers_1_jnt', 'fingers_2_jnt']
        heelLoc = 'frontFoot_heel'
        innerLoc = 'frontFoot_inner'
        outerLoc = 'frontFoot_outer'

        self.leftFrontFootRig = foot.Foot(
            toeJoints = toeJoints,
            heelLoc = heelLoc,
            innerLoc = innerLoc,
            outerLoc = outerLoc,

            fkAnkleJoint=self.leftArmRig.rigParts['fkJoints'][-1],
            ikAnkleJoint=self.leftArmRig.rigParts['ikJoints'][-1],
            fkFootCtr=self.leftArmRig.rigParts['fkControls'][-1],

            ikFootCtr=self.leftArmRig.rigParts['ikControl'],
            ikParentCtr=self.leftArmRig.rigParts['ikGimbalControl'],
            switchAttr=self.leftArmRig.rigParts['FKIKSwitchAttr'],

            ikGroupToDrive = self.leftArmRig.rigParts['reverseFootDriven'],

            prefix = 'frontFoot',
            side = 'l',

            rollAxis = 'x',
            rockAxis = '-z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.leftFrontFootRig.build()

        # right front foot

        self.rightFrontFootRig = foot.Foot(
            toeJoints = toeJoints,
            heelLoc=heelLoc,
            innerLoc=innerLoc,
            outerLoc=outerLoc,

            fkAnkleJoint=self.rightArmRig.rigParts['fkJoints'][-1],
            ikAnkleJoint=self.rightArmRig.rigParts['ikJoints'][-1],
            fkFootCtr=self.rightArmRig.rigParts['fkControls'][-1],

            ikFootCtr=self.rightArmRig.rigParts['ikControl'],
            ikParentCtr=self.rightArmRig.rigParts['ikGimbalControl'],
            switchAttr=self.rightArmRig.rigParts['FKIKSwitchAttr'],


            ikGroupToDrive=self.rightArmRig.rigParts['reverseFootDriven'],

            prefix='frontFoot',
            side='r',

            rollAxis='x',
            rockAxis='z',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        self.rightFrontFootRig.build()

        # left back foot

        backToeJoints = ['toes_1_jnt', 'toes_2_jnt']
        backHeelLoc = 'backFoot_heel'
        backInnerLoc = 'backFoot_inner'
        backOuterLoc = 'backFoot_outer'

        self.leftBackFootRig = foot.Foot(

            toeJoints = backToeJoints,
            heelLoc = backHeelLoc,
            innerLoc = backInnerLoc,
            outerLoc = backOuterLoc,

            fkAnkleJoint=self.leftLegRig.rigParts['fkJoints'][-1],
            ikAnkleJoint=self.leftLegRig.rigParts['ikJoints'][-1],
            fkFootCtr=self.leftLegRig.rigParts['fkControls'][-1],

            ikFootCtr=self.leftLegRig.rigParts['ikControl'],
            ikParentCtr=self.leftLegRig.rigParts['ikGimbalControl'],
            switchAttr=self.leftLegRig.rigParts['FKIKSwitchAttr'],

            ikGroupToDrive=self.leftLegRig.rigParts['reverseFootDriven'],


            prefix='backFoot',
            side='l',
            rollAxis='x',
            rockAxis='-z',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        self.leftBackFootRig.build()

        # right back foot

        self.rightBackFootRig = foot.Foot(
            toeJoints = backToeJoints,
            heelLoc = backHeelLoc,
            innerLoc = backInnerLoc,
            outerLoc = backOuterLoc,

            fkAnkleJoint=self.rightLegRig.rigParts['fkJoints'][-1],
            ikAnkleJoint=self.rightLegRig.rigParts['ikJoints'][-1],
            fkFootCtr=self.rightLegRig.rigParts['fkControls'][-1],

            ikFootCtr=self.rightLegRig.rigParts['ikControl'],
            ikParentCtr=self.rightLegRig.rigParts['ikGimbalControl'],
            switchAttr=self.rightLegRig.rigParts['FKIKSwitchAttr'],

            ikGroupToDrive = self.rightLegRig.rigParts['reverseFootDriven'],
            prefix = 'backFoot',
            side = 'r',
            rollAxis = 'x',
            rockAxis = 'z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.rightBackFootRig.build()







        # Left ear

        # Right ear

        # Face - Blink eye
