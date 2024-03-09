"""
dog archetype rig setup
main module
"""


from . import char
from . import project

from rigLib.base import control
from rigLib.base import module

from rigLib.rig import quadSpine
from rigLib.rig import neck
from rigLib.rig import tail
from rigLib.rig import ikChain
from rigLib.rig import limb
from rigLib.rig import hand
from rigLib.rig import bendyLimb
from rigLib.rig import foot
from rigLib.rig import fkChain



import maya.cmds as mc

class Dog(char.Character):

    def __init__(self, characterName, sceneScale = project.sceneScale):
        char.Character.__init__(self, characterName, sceneScale)

    def build(self):

        self.setup()
        self.preBuildCleanup()
        self.buildRig(self.baseRig)
        self.buildExtraControls(self.baseRig)
        self.deform()
        self.loadControlShapes()
        #self.setInitialPose()

    def buildRig(self, baseRig):

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

        armJoints = ['shoulder_jnt', 'elbow_jnt', 'wrist_jnt', 'fingers_1_jnt']
        scapulaJnt = 'scapula_jnt'
        toeEndJnt = 'fingers_2_jnt'


        self.leftArmRig = limb.Limb(
            type = 'digi',
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
            type = 'digi',
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

        legJoints = ['hip_jnt', 'knee_jnt', 'ankle_jnt', 'toes_1_jnt']
        backToeEndJnt = 'toes_2_jnt'
        #footJoints = ['backFoot_jnt', 'backToe_jnt', 'backToeEnd_jnt']
        #hipPivotJoint = 'hip_pivot_jnt'

        #heel_loc = 'backFoot_heel'
        #inner_loc = 'backFoot_inner'
        #outer_loc = 'backFoot_outer'

        self.leftLegRig = limb.Limb(
            type = 'digi',
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
            type='digi',
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

            ikParentJoint = self.leftArmRig.rigParts['ikJoints'][-1],

            ikFootCtr = self.leftArmRig.rigParts['ikControl'],

            legType = 'digi',

            ballCtr = self.leftArmRig.rigParts['ikControls'][-1],
            toeIKCtr = self.leftArmRig.rigParts['ikControls'][2],
            ankleRollDriver = self.leftArmRig.rigParts['ankleRollGrp'],

            rollAxis='x',
            rockAxis ='-z',
            prefix='frontFoot',
            side='l',
            rigScale= self.sceneScale,
            baseRig= baseRig

        )

        self.leftFrontFootRig.build()



        # right front foot

        self.rightFrontFootRig = foot.Foot(

            toeJoints=toeJoints,
            heelLoc=heelLoc,
            innerLoc=innerLoc,
            outerLoc=outerLoc,

            ikParentJoint=self.rightArmRig.rigParts['ikJoints'][-1],

            ikFootCtr=self.rightArmRig.rigParts['ikControl'],

            legType='digi',

            ballCtr=self.rightArmRig.rigParts['ikControls'][-1],
            toeIKCtr=self.rightArmRig.rigParts['ikControls'][2],
            ankleRollDriver=self.rightArmRig.rigParts['ankleRollGrp'],

            rollAxis = 'x',
            rockAxis = 'z',
            prefix = 'frontFoot',
            side = 'r',
            rigScale = self.sceneScale,
            baseRig = baseRig
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

            ikParentJoint=self.leftLegRig.rigParts['ikJoints'][-1],

            ikFootCtr=self.leftLegRig.rigParts['ikControl'],

            legType='digi',

            ballCtr=self.leftLegRig.rigParts['ikControls'][-1],
            toeIKCtr=self.leftLegRig.rigParts['ikControls'][2],
            ankleRollDriver=self.leftLegRig.rigParts['ankleRollGrp'],

            rollAxis='x',
            rockAxis = '-z',
            prefix='backFoot',
            side='l',
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

            ikParentJoint=self.rightLegRig.rigParts['ikJoints'][-1],

            ikFootCtr=self.rightLegRig.rigParts['ikControl'],

            legType='digi',

            ballCtr=self.rightLegRig.rigParts['ikControls'][-1],
            toeIKCtr=self.rightLegRig.rigParts['ikControls'][2],
            ankleRollDriver=self.rightLegRig.rigParts['ankleRollGrp'],

            rollAxis ='x',
            rockAxis = 'z',
            prefix = 'backFoot',
            side = 'r',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        self.rightBackFootRig.build()

        # Left front fingers
        fingerJoints = ['indexFinger_1_jnt', 'middleFinger_1_jnt', 'ringFinger_1_jnt', 'pinkyFinger_1_jnt']

        leftFingersRig = hand.Hand(
            fingerBaseJoints=fingerJoints,
            metaJoints= False,
            includeFingerEnds=False,
            handAttachGrp=self.leftArmRig.rigParts['footAttachGrp'],
            prefix = 'fingers',
            side='l',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )
        leftFingersRig.build()

        # Right front fingers
        rightFingersRig = hand.Hand(
            fingerBaseJoints=fingerJoints,
            metaJoints = False,
            includeFingerEnds=False,
            handAttachGrp=self.rightArmRig.rigParts['footAttachGrp'],
            moveHandCtr = '-x',
            prefix = 'fingers',
            side= 'r',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )
        rightFingersRig.build()


        # Left back toes
        toeFingerJoints = ['indexToe_1_jnt', 'middleToe_1_jnt', 'ringToe_1_jnt', 'pinkyToe_1_jnt']

        leftToesRig = hand.Hand(
            fingerBaseJoints=toeFingerJoints,
            metaJoints=False,
            includeFingerEnds=False,
            handAttachGrp=self.leftLegRig.rigParts['footAttachGrp'],
            prefix='toes',
            side='l',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )
        leftToesRig.build()

        rightToesRig = hand.Hand(
            fingerBaseJoints=toeFingerJoints,
            metaJoints=False,
            includeFingerEnds=False,
            handAttachGrp=self.rightLegRig.rigParts['footAttachGrp'],
            moveHandCtr = '-x',
            prefix='toes',
            side='r',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )
        rightToesRig.build()








    def buildExtraControls(self, baseRig):

        # Make a module to hold new items
        head_rigmodule = module.Module(prefix = 'headGroup', baseObj = baseRig)

        # Left ear
        earJoints = ['ear_1_jnt', 'ear_2_jnt', 'ear_3_jnt', 'ear_4_jnt', 'ear_5_jnt']

        leftEarJoints = []
        rightEarJoints = []

        for jnt in earJoints:
            leftJoint = 'l_' + jnt
            rightJoint = 'r_' + jnt

            leftEarJoints.append(leftJoint)
            rightEarJoints.append(rightJoint)

        leftEarRig = fkChain.build(leftEarJoints, rigScale=self.sceneScale,
                                     parent= head_rigmodule.controlsGrp, endControl= False,
                                     offsets=['null', 'zero', 'auto'])

        # Right ear
        rightEarRig = fkChain.build(rightEarJoints, rigScale=self.sceneScale,
                                       parent = head_rigmodule.controlsGrp, endControl= False,
                                       offsets=['null', 'zero', 'auto'])



        # Jaw
        jawJoint = 'jaw_jnt'
        jawCtr = control.Control(prefix='Jaw', translateTo=jawJoint,
                                 rotateTo=jawJoint, lockChannels=['s', 'v'],
                                 scale=self.sceneScale, parent=head_rigmodule.controlsGrp,
                                 shape='circle', color='cyan')
        jawConstraint = mc.parentConstraint(jawCtr.C, jawJoint, mo=1)[0]


        mc.parentConstraint(self.neckRig.rigParts['headAttachGrp'], leftEarRig['topControl'].Off, mo=1)
        mc.parentConstraint(self.neckRig.rigParts['headAttachGrp'], rightEarRig['topControl'].Off, mo=1)
        mc.parentConstraint(self.neckRig.rigParts['headAttachGrp'], jawCtr.Off, mo=1)

        for constraint in leftEarRig['constraints']:
            mc.parent(constraint, head_rigmodule.noXformGrp)

        for constraint in rightEarRig['constraints']:
            mc.parent(constraint, head_rigmodule.noXformGrp)

        mc.parent(jawConstraint, head_rigmodule.noXformGrp)

        # Face - Blink eye

