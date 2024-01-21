"""
tortoise rig setup
main module
"""


from . import char
from . import project
from rigLib.rig import spine
from rigLib.rig import neck
from rigLib.rig import tail
from rigLib.rig import ikChain
from rigLib.rig import legFKIK
from rigLib.rig import armFKIK
from rigLib.rig import hand
from rigLib.rig import bendyLimb
from rigLib.rig import reverseFoot
from rigLib.base import control
from rigLib.base import module

from rigLib.utils import joint

import maya.cmds as mc

class Tortoise(char.Character):

    def __init__(self, characterName, sceneScale = project.sceneScale):
        char.Character.__init__(self, characterName, sceneScale)

    def build(self):

        self.setup()
        self.makeControlSetup(self.baseRig)
        self.deform()
        #self.setInitialPose()

    def makeControlSetup(self, baseRig):

        # make rig module

        shell_rigmodule = module.Module(prefix = 'Shell', baseObj = baseRig)

        # Build COG control
        cogJoint = 'root_jnt'
        chestPivot = 'chest_jnt'
        spineBasePivot = 'spineBase_jnt'

        bodyCtrl = control.Control(prefix='COG', translateTo = cogJoint, rotateTo = cogJoint,
                                   scale=self.sceneScale * 5, shape='circleX', parent= shell_rigmodule.controlsGrp )

        # Build SpineBase Control
        spineBaseCtr = control.Control(prefix='SpineBasePivot', translateTo = spineBasePivot, rotateTo = spineBasePivot,
                                   scale = self.sceneScale * 5, shape='circleX', parent = bodyCtrl.C)

        # Build Chest Pivot control
        chestCtr = control.Control(prefix='ChestPivot', translateTo = chestPivot, rotateTo = chestPivot,
                                   scale=self.sceneScale * 5, shape='circleX', parent = spineBaseCtr.C)

        # Connect root joint to chest pivot control
        constraints = []
        rootconstraint = mc.parentConstraint(chestCtr.C, cogJoint, mo = 1)[0]
        constraints.append(rootconstraint)

        chestAttachGrp = mc.group(n = 'ChestAttachGrp', em = 1)
        hipsAttachGrp = mc.group(n='hipsAttachGrp', em=1)

        mc.parentConstraint(chestPivot, chestAttachGrp, mo = 0)
        mc.parentConstraint(spineBasePivot, hipsAttachGrp, mo=0)

        mc.parent(chestAttachGrp, shell_rigmodule.partsGrp)
        mc.parent(hipsAttachGrp, shell_rigmodule.partsGrp)

        # Neck

        neckJoints = ['neck_1_jnt', 'neck_2_jnt', 'neck_3_jnt', 'neck_4_jnt', 'neck_5_jnt']
        neckBaseJnt = 'neckBase_jnt'
        headJoint = 'head_jnt'

        self.neckRig = neck.Neck(
            neckJoints=neckJoints,
            headJnt=headJoint,
            neckCurve='neck_curve',
            prefix = 'Neck',
            forwardAxis = 'x',
            upAxis = 'y',
            middleControl = True,
            headParentToNeckBase= False,
            rigScale = self.sceneScale,
            baseRig = baseRig
        )
        self.neckRig.build()
        self.neckRig.setInitialValues(HeadOrient= 1, Stretchy= 1)

        # Make a control for the neck base

        neckBaseCtr = control.Control(prefix='NeckBase', translateTo=neckBaseJnt, rotateTo=neckBaseJnt,
                                      scale=self.sceneScale * 4, shape='circleX', parent = self.neckRig.rigmodule.controlsGrp)

        mc.parentConstraint(chestAttachGrp, neckBaseCtr.Off, mo = 1)

        mc.parentConstraint(neckBaseCtr.C, neckBaseJnt, mo=1)


        # attach neck to neckBase
        mc.parentConstraint( neckBaseCtr.C, self.neckRig.rigParts['baseAttachGrp'], mo=1)



        # Add Jaw

        # Make a control for the jaw
        jawJoint = 'jaw_jnt'

        jawCtr = control.Control(prefix='Jaw', translateTo = jawJoint, rotateTo = jawJoint,
                                      scale = self.sceneScale, shape='circleX',
                                      parent = self.neckRig.rigParts['headCtr'].C)

        mc.parentConstraint(jawCtr.C, jawJoint, mo=1)

        # Tail
        tailJoints = ['tail_1_jnt', 'tail_2_jnt', 'tail_3_jnt', 'tail_4_jnt', 'tail_5_jnt',
                      'tail_6_jnt', 'tail_7_jnt']

        tailRig = tail.Tail(
             tailJoints = tailJoints,
             prefix = 'Tail',
             bendy = False,
             rigScale = self.sceneScale,
             baseRig = baseRig,
             )

        tailRig.build()

        # attach tail to spine
        mc.parentConstraint( hipsAttachGrp, tailRig.rigParts['bodyAttachGrp'], mo=1)



        # Left Front Leg

        armJoints = ['shoulder_jnt', 'elbow_jnt', 'wrist_jnt']
        scapulaJnt = 'scapula_jnt'
        toeJoints = ['frontToes_jnt', 'frontToesEnd_jnt']
        heelLoc = 'frontFoot_heel'
        innerLoc = 'frontFoot_inner'
        outerLoc = 'frontFoot_outer'

        self.leftArmRig = armFKIK.Arm(
            armJoints=armJoints,
            scapulaJoint=scapulaJnt,
            prefix='frontLeg',
            side='l',
            bendy = False,
            elbowDirection = 'y',
            forwardAxis = 'x',
            moveSwitchCtr = 'x',
            ikCtrOrient = 'world',
            buildFoot = True,
            toeJoints = toeJoints,
            heelLoc = heelLoc,
            innerLoc = innerLoc,
            outerLoc = outerLoc,
            rollAxis = 'x',
            rockAxis = '-z',
            rigScale = self.sceneScale,
            baseRig=baseRig
        )

        self.leftArmRig.build()
        self.leftArmRig.setInitialValues(FKIKMode= 1, Stretchy = 0)
        self.leftArmRig.footRig.setInitialValues(BallRollAngle= 45, ToeRollAngle= 90)

        # attach left arm to spine
        mc.parentConstraint( chestAttachGrp, self.leftArmRig.rigParts['bodyAttachGrp'], mo=1)

        # Right Front Leg
        self.rightArmRig = armFKIK.Arm(
            armJoints = armJoints,
            scapulaJoint = scapulaJnt,
            prefix = 'frontLeg',
            side = 'r',
            bendy = False,
            elbowDirection = '-y',
            forwardAxis = '-x',
            moveSwitchCtr = '-x',
            ikCtrOrient = 'world',
            buildFoot = True,
            toeJoints = toeJoints,
            heelLoc = heelLoc,
            innerLoc = innerLoc,
            outerLoc = outerLoc,
            rollAxis = 'x',
            rockAxis = '-z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )
        self.rightArmRig.build()
        self.rightArmRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.rightArmRig.footRig.setInitialValues(BallRollAngle=0.1, ToeRollAngle=90)

        # attach right arm to spine
        mc.parentConstraint( chestAttachGrp, self.rightArmRig.rigParts['bodyAttachGrp'], mo=1)



        # Left Back Leg

        legJoints = ['hip_jnt', 'knee_jnt', 'ankle_jnt']
        hipScapula = 'hip_pivot_jnt'
        backLegToeJoints = ['backToe_jnt', 'backToeEnd_jnt']
        backHeelLoc = 'backFoot_heel'
        backInnerLoc = 'backFoot_inner'
        backOuterLoc = 'backFoot_outer'

        self.leftLegRig = armFKIK.Arm(
            armJoints = legJoints,
            scapulaJoint = hipScapula,
            prefix = 'backLeg',
            side = 'l',
            bendy = False,
            elbowDirection = 'y',
            forwardAxis = 'x',
            moveSwitchCtr = '-z',
            ikCtrOrient = 'world',
            buildFoot = True,
            toeJoints = backLegToeJoints,
            heelLoc = backHeelLoc,
            innerLoc = backInnerLoc,
            outerLoc = backOuterLoc,
            rollAxis = 'x',
            rockAxis = '-z',
            rigScale = self.sceneScale,
            baseRig = baseRig
            )

        self.leftLegRig.build()
        self.leftLegRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.leftLegRig.footRig.setInitialValues(BallRollAngle=45, ToeRollAngle=90)

        # attach left leg to spine
        mc.parentConstraint(hipsAttachGrp, self.leftLegRig.rigParts['bodyAttachGrp'], mo=1)



        # Right leg
        self.rightLegRig = armFKIK.Arm(

            armJoints=legJoints,
            scapulaJoint=hipScapula,
            prefix='backLeg',
            side='r',
            bendy=False,
            elbowDirection='-y',
            forwardAxis = '-x',
            moveSwitchCtr = '-z',
            ikCtrOrient = 'world',
            buildFoot = True,
            toeJoints = backLegToeJoints,
            heelLoc = backHeelLoc,
            innerLoc = backInnerLoc,
            outerLoc = backOuterLoc,
            rollAxis = 'x',
            rockAxis = '-z',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        self.rightLegRig.build()
        self.rightLegRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.rightLegRig.footRig.setInitialValues(BallRollAngle=45, ToeRollAngle=90)

        # attach left leg to spine
        mc.parentConstraint(hipsAttachGrp, self.rightLegRig.rigParts['bodyAttachGrp'], mo=1)



        # Face - Blink eye
