"""
tortoise rig setup
main module
"""


from . import char
from . import project
from . import char_deform
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
        self.setInitialSettings()
        self.adjustControlShapes()
        self.deform()
        #self.setInitialPose()

    def makeControlSetup(self, baseRig):

        # make rig module

        shell_rigmodule = module.Module(prefix = 'Shell', baseObj = baseRig)

        # Build COG control
        cogJoint = 'root_jnt'
        chestPivot = 'chest_jnt'
        spineBasePivot = 'spineBase_jnt'

        bodyCtrl = control.Control(prefix='COG', translateTo = cogJoint, rotateTo = cogJoint, color = 'cyan',
                                   scale=self.sceneScale * 5, shape='circleX', parent= shell_rigmodule.controlsGrp )

        # Build SpineBase Control
        spineBaseCtr = control.Control(prefix='SpineBasePivot', translateTo = spineBasePivot, rotateTo = spineBasePivot,
                                   scale = self.sceneScale * 5, shape='circleX', parent = bodyCtrl.C, color = 'cyan',)

        # Build Chest Pivot control
        chestCtr = control.Control(prefix='ChestPivot', translateTo = chestPivot, rotateTo = chestPivot, color = 'cyan',
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


        # Make a control for the neck base

        neckBaseCtr = control.Control(prefix='NeckBase', translateTo=neckBaseJnt, rotateTo=neckBaseJnt,
                                      scale=self.sceneScale * 4, shape='circleX', parent = self.neckRig.rigmodule.controlsGrp)

        mc.parentConstraint(chestAttachGrp, neckBaseCtr.Off, mo = 1)

        mc.parentConstraint(neckBaseCtr.C, neckBaseJnt, mo=1)

        # adjust shape
        control._translateCtrlShape(neckBaseCtr, axis = 'y', value = 15)
        control._translateCtrlShape(neckBaseCtr, axis='z', value=11)
        control._rotateCtrlShape(neckBaseCtr, axis='x', value=53)
        control._scaleCtrlShape(neckBaseCtr, axis='x,y, z', value=0.5)


        # attach neck to neckBase
        mc.parentConstraint( neckBaseCtr.C, self.neckRig.rigParts['baseAttachGrp'], mo=1)

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



        # attach right leg to spine
        mc.parentConstraint(hipsAttachGrp, self.rightLegRig.rigParts['bodyAttachGrp'], mo=1)

        # Build face
        self.buildFace(self.neckRig.rigParts['headAttachGrp'])



    def buildFace(self, headFollow):

        # make rig module

        face_rigmodule = module.Module(prefix= 'Face', baseObj=self.baseRig)
        mc.parentConstraint(headFollow, face_rigmodule.topGrp, mo=1)

        # Jaw

        jawJoint = 'jaw_jnt'

        jawCtr = control.Control(prefix='Jaw', translateTo=jawJoint, rotateTo=jawJoint,
                                 scale=self.sceneScale * 0.6, shape='circleX', color='mid_green',
                                 parent = face_rigmodule.controlsGrp)

        mc.parentConstraint(jawCtr.C, jawJoint, mo=1)




        control._translateCtrlShape(jawCtr, axis='z', value=4)
        control._translateCtrlShape(jawCtr, axis='y', value=-3)


        # Eyes

        eyejointName = 'eye_jnt'
        upperLidJointName = 'upperLid_jnt'
        lowerLidJointName = 'lowerLid_jnt'

        # Left eye
        side = 'l'
        eyeJoint = side + '_'  + eyejointName
        self.leftEyeCtr = control.Control(prefix= side + '_eye', translateTo=eyeJoint, rotateTo=eyeJoint,
                                 scale=self.sceneScale * 0.05, shape='circleX', color='yellow', lockChannels=['t', 's', 'v'],
                                 parent= face_rigmodule.controlsGrp)

        mc.orientConstraint(self.leftEyeCtr.C, eyeJoint, mo=1)

        # Upper lid

        upperLidJoint = side + '_' + upperLidJointName
        self.leftUpperLidCtr = control.Control(prefix=side + '_upperLid', translateTo=upperLidJoint, rotateTo=upperLidJoint,
                                          scale=self.sceneScale * 0.05, shape='circleX', color='yellow',
                                          lockChannels=['t', 's', 'v'], offsets=['null', 'zero', 'auto'],
                                          parent=face_rigmodule.controlsGrp)
        mc.orientConstraint(self.leftUpperLidCtr.C, upperLidJoint, mo=1)

        # Lower lid
        lowerLidJoint = side + '_' + lowerLidJointName

        self.leftLowerLidCtr = control.Control(prefix=side + '_lowerLid', translateTo=lowerLidJoint, rotateTo=lowerLidJoint,
                                          scale=self.sceneScale * 0.05, shape='circleX', color='yellow',
                                          lockChannels=['t', 's', 'v'], offsets=['null', 'zero', 'auto'],
                                          parent=face_rigmodule.controlsGrp)
        mc.orientConstraint(self.leftLowerLidCtr.C, lowerLidJoint, mo=1)


        # Right eye

        side = 'r'
        eyeJoint = side + '_' + eyejointName
        self.rightEyeCtr = control.Control(prefix=side + '_eye', translateTo=eyeJoint, rotateTo=eyeJoint,
                                     scale=self.sceneScale * 0.05, shape='circleX', color='yellow',
                                     lockChannels=['t', 's', 'v'],
                                     parent=face_rigmodule.controlsGrp)

        mc.orientConstraint(self.rightEyeCtr.C,  eyeJoint, mo=1)

        # Upper lid
        #upperLidJoint = joint.duplicateChain(eyeJoint, 'jnt', 'upperLid_jnt')[0]
        #parent = mc.listRelatives(eyeJoint, p=1)
        #mc.parent(upperLidJoint, parent)

        upperLidJoint = side + '_' + upperLidJointName

        self.rightUpperLidCtr = control.Control(prefix=side + '_upperLid', translateTo=upperLidJoint, rotateTo=upperLidJoint,
                                          scale=self.sceneScale * 0.05, shape='circleX', color='yellow',
                                          lockChannels=['t', 's', 'v'], offsets=['null', 'zero', 'auto'],
                                          parent=face_rigmodule.controlsGrp)
        mc.orientConstraint(self.rightUpperLidCtr.C, upperLidJoint, mo=1)

        # Lower lid
        lowerLidJoint = side + '_' + lowerLidJointName

        self.rightLowerLidCtr = control.Control(prefix=side + '_lowerLid', translateTo=lowerLidJoint, rotateTo=lowerLidJoint,
                                          scale=self.sceneScale * 0.05, shape='circleX', color='yellow',
                                          lockChannels=['t', 's', 'v'], offsets=['null', 'zero', 'auto'],
                                          parent=face_rigmodule.controlsGrp)
        mc.orientConstraint(self.rightLowerLidCtr.C, lowerLidJoint, mo=1)


        # Add blink attributes
        blink_attr = 'Blink'
        upperLid_attr = 'Upper_Lid'
        lowerLid_attr = 'Lower_Lid'
        # Add to left
        mc.addAttr(self.leftEyeCtr.C, ln = upperLid_attr, at='double', dv=0, k=1, min = 0, max = 10)
        mc.addAttr(self.leftEyeCtr.C, ln=lowerLid_attr, at='double', dv=0, k=1, min = 0, max = 10)
        mc.addAttr(self.leftEyeCtr.C, ln=blink_attr, at='double', dv=0, k=1, min = 0, max = 10)
        # Add to right
        mc.addAttr(self.rightEyeCtr.C, ln=upperLid_attr, at='double', dv=0, k=1, min=0, max=10)
        mc.addAttr(self.rightEyeCtr.C, ln=lowerLid_attr, at='double', dv=0, k=1, min=0, max=10)
        mc.addAttr(self.rightEyeCtr.C, ln=blink_attr, at='double', dv=0, k=1, min=0, max=10)

        # Set up SDKs

        # Left upper lid
        driven = '{}.rz'.format(self.leftUpperLidCtr.C)
        driver = '{}.{}'.format(self.leftEyeCtr.C, upperLid_attr)
        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue = 10,
                             value= -75,
                             inTangentType='linear',
                             outTangentType='linear')

        # Left lower lid
        driven = '{}.rz'.format(self.leftLowerLidCtr.C)
        driver = '{}.{}'.format(self.leftEyeCtr.C, lowerLid_attr)
        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=0,
                             value = 0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=10,
                             value = 60,
                             inTangentType='linear',
                             outTangentType='linear')

        # Left blink
        drivenUp = '{}.rz'.format(self.leftUpperLidCtr.Offsets[2])
        drivenLow = '{}.rz'.format(self.leftLowerLidCtr.Offsets[2])
        driver = '{}.{}'.format(self.leftEyeCtr.C, blink_attr)
        #Upper
        mc.setDrivenKeyframe(drivenUp,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(drivenUp,
                             currentDriver=driver,
                             driverValue=10,
                             value= -57,
                             inTangentType='linear',
                             outTangentType='linear')
        # Lower
        mc.setDrivenKeyframe(drivenLow,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(drivenLow,
                             currentDriver=driver,
                             driverValue=10,
                             value= 20,
                             inTangentType='linear',
                             outTangentType='linear')



        # Follow eye - left upper lid
        l_eyeRotateUpper_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='l_upperLidrotate_mult')
        mc.setAttr('{}.operation'.format(l_eyeRotateUpper_mult), 1)
        mc.connectAttr('{}.rz'.format(self.leftEyeCtr.C), '{}.input1X'.format(l_eyeRotateUpper_mult))
        mc.setAttr('{}.input2X'.format(l_eyeRotateUpper_mult), 0.2)
        mc.connectAttr('{}.outputX'.format(l_eyeRotateUpper_mult), '{}.rz'.format(self.leftUpperLidCtr.Offsets[1]))

        # Follow eye - left lower lid
        l_eyeRotateLower_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='l_lowerLidrotate_mult')
        mc.setAttr('{}.operation'.format(l_eyeRotateLower_mult), 1)
        mc.connectAttr('{}.rz'.format(self.leftEyeCtr.C), '{}.input1X'.format(l_eyeRotateLower_mult))
        mc.setAttr('{}.input2X'.format(l_eyeRotateLower_mult), 0.2)
        mc.connectAttr('{}.outputX'.format(l_eyeRotateLower_mult), '{}.rz'.format(self.leftLowerLidCtr.Offsets[1]))




        # Right upper lid
        driven = '{}.rz'.format(self.rightUpperLidCtr.C)
        driver = '{}.{}'.format(self.rightEyeCtr.C, upperLid_attr)
        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=10,
                             value=-75,
                             inTangentType='linear',
                             outTangentType='linear')

        # Right lower lid
        driven = '{}.rz'.format(self.rightLowerLidCtr.C)
        driver = '{}.{}'.format(self.rightEyeCtr.C, lowerLid_attr)
        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=10,
                             value=60,
                             inTangentType='linear',
                             outTangentType='linear')

        # Right blink
        drivenUp = '{}.rz'.format(self.rightUpperLidCtr.Offsets[2])
        drivenLow = '{}.rz'.format(self.rightLowerLidCtr.Offsets[2])
        driver = '{}.{}'.format(self.rightEyeCtr.C, blink_attr)
        # Upper
        mc.setDrivenKeyframe(drivenUp,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(drivenUp,
                             currentDriver=driver,
                             driverValue=10,
                             value=-57,
                             inTangentType='linear',
                             outTangentType='linear')
        # Lower
        mc.setDrivenKeyframe(drivenLow,
                             currentDriver=driver,
                             driverValue=0,
                             value=0,
                             inTangentType='linear',
                             outTangentType='linear')

        mc.setDrivenKeyframe(drivenLow,
                             currentDriver=driver,
                             driverValue=10,
                             value=20,
                             inTangentType='linear',
                             outTangentType='linear')

        # Follow eye - right upper lid
        r_eyeRotateUpper_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='r_upperLidrotate_mult')
        mc.setAttr('{}.operation'.format(r_eyeRotateUpper_mult), 1)
        mc.connectAttr('{}.rz'.format(self.rightEyeCtr.C), '{}.input1X'.format(r_eyeRotateUpper_mult))
        mc.setAttr('{}.input2X'.format(r_eyeRotateUpper_mult), 0.2)
        mc.connectAttr('{}.outputX'.format(r_eyeRotateUpper_mult), '{}.rz'.format(self.rightUpperLidCtr.Offsets[1]))

        # Follow eye - right lower lid
        r_eyeRotateLower_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='r_lowerLidrotate_mult')
        mc.setAttr('{}.operation'.format(r_eyeRotateLower_mult), 1)
        mc.connectAttr('{}.rz'.format(self.rightEyeCtr.C), '{}.input1X'.format(r_eyeRotateLower_mult))
        mc.setAttr('{}.input2X'.format(r_eyeRotateLower_mult), 0.2)
        mc.connectAttr('{}.outputX'.format(r_eyeRotateLower_mult), '{}.rz'.format(self.rightLowerLidCtr.Offsets[1]))





    def deform(self):
        # Run parent class commands
        char.Character.deform(self)

        # add a delta mush to body
        deltaMushGeo = ['Tortoise1:Body_Front', 'Tortoise1:Body_Back']
        char_deform.buildDeltaMush(self.baseRig, characterName= self.characterName, geoList= deltaMushGeo)

    def adjustControlShapes(self):

        # adjust neck base ik control
        control._translateCtrlShape(self.neckRig.rigParts['controls'][0], axis='z', value=8)
        control._translateCtrlShape(self.neckRig.rigParts['controls'][0], axis='y', value=3)
        control._scaleCtrlShape(self.neckRig.rigParts['controls'][0], axis='x,y,z', value=.5)
        # middle control
        control._scaleCtrlShape(self.neckRig.rigParts['controls'][2], axis='x,y,z', value=.7)
        control._translateCtrlShape(self.neckRig.rigParts['controls'][2], axis='y', value=-2)
        control._translateCtrlShape(self.neckRig.rigParts['controls'][2], axis='z', value=1)
        # head control
        control._scaleCtrlShape(self.neckRig.rigParts['controls'][1], axis='x,y,z', value=1.2)
        control._translateCtrlShape(self.neckRig.rigParts['controls'][1], axis='y', value=1.25)
        control._translateCtrlShape(self.neckRig.rigParts['controls'][1], axis='z', value=-2.63)




        # Adjust left arm controls
        # End control
        control._scaleCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='y', value=0.5)
        control._scaleCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='z', value=1.5)
        control._scaleCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='x', value=1.2)
        control._translateCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='z', value=5)
        control._translateCtrlShape(self.leftArmRig.rigParts['ikControl'], axis='y', value=-5)
        # gimbal control
        control._scaleCtrlShape(self.leftArmRig.rigParts['ikControls'][2], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.leftArmRig.rigParts['ikControls'][2], axis='x', value=30)

        # Adjust foot controls
        control._scaleCtrlShape(self.leftArmRig.rigParts['footControls'][0], axis='x,y,z', value=1.5)
        control._scaleCtrlShape(self.leftArmRig.rigParts['footControls'][1], axis='x,y,z', value=1.5)

        # Adjust fk controls
        #  wrist
        control._scaleCtrlShape(self.leftArmRig.rigParts['fkControls'][2], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.leftArmRig.rigParts['fkControls'][2], axis='x', value=30)
        # toe
        control._scaleCtrlShape(self.leftArmRig.rigParts['fkControls'][3], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.leftArmRig.rigParts['fkControls'][3], axis='x', value=30)
        # elbow
        control._scaleCtrlShape(self.leftArmRig.rigParts['fkControls'][1], axis='x,y,z', value=1.2)
        # shoulder
        control._translateCtrlShape(self.leftArmRig.rigParts['fkControls'][0], axis='z', value=6)
        control._translateCtrlShape(self.leftArmRig.rigParts['fkControls'][0], axis='x', value=5)
        # scapula aim
        control._translateCtrlShape(self.leftArmRig.rigParts['scapulaControls'][1], axis='z', value=5)
        control._translateCtrlShape(self.leftArmRig.rigParts['scapulaControls'][1], axis='x', value=4)
        # scapula
        control._translateCtrlShape(self.leftArmRig.rigParts['scapulaControls'][0], axis='x', value=13)
        control._translateCtrlShape(self.leftArmRig.rigParts['scapulaControls'][0], axis='y', value=22)
        control._translateCtrlShape(self.leftArmRig.rigParts['scapulaControls'][0], axis='z', value=14)
        # switch
        control._rotateCtrlShape(self.leftArmRig.rigParts['switchControl'], axis='y', value=90)



        # Right arm
        # ik end control
        control._scaleCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='y', value=0.5)
        control._scaleCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='z', value=1.5)
        control._scaleCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='x', value=1.2)
        control._translateCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='z', value=5)
        control._translateCtrlShape(self.rightArmRig.rigParts['ikControl'], axis='y', value=-5)
        # gimbal control
        control._scaleCtrlShape(self.rightArmRig.rigParts['ikControls'][2], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.rightArmRig.rigParts['ikControls'][2], axis='x', value=30)

        # Adjust foot controls
        control._scaleCtrlShape(self.rightArmRig.rigParts['footControls'][0], axis='x,y,z', value=1.5)
        control._scaleCtrlShape(self.rightArmRig.rigParts['footControls'][1], axis='x,y,z', value=1.5)

        # Adjust fk controls
        #  wrist
        control._scaleCtrlShape(self.rightArmRig.rigParts['fkControls'][2], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.rightArmRig.rigParts['fkControls'][2], axis='x', value=30)
        # toe
        control._scaleCtrlShape(self.rightArmRig.rigParts['fkControls'][3], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.rightArmRig.rigParts['fkControls'][3], axis='x', value=30)
        # elbow
        control._scaleCtrlShape(self.rightArmRig.rigParts['fkControls'][1], axis='x,y,z', value=1.2)
        # shoulder
        control._translateCtrlShape(self.rightArmRig.rigParts['fkControls'][0], axis='z', value=6)
        control._translateCtrlShape(self.rightArmRig.rigParts['fkControls'][0], axis='x', value=-5)
        # scapula aim
        control._translateCtrlShape(self.rightArmRig.rigParts['scapulaControls'][1], axis='z', value=5)
        control._translateCtrlShape(self.rightArmRig.rigParts['scapulaControls'][1], axis='x', value=-4)
        # scapula
        control._translateCtrlShape(self.rightArmRig.rigParts['scapulaControls'][0], axis='x', value=-13)
        control._translateCtrlShape(self.rightArmRig.rigParts['scapulaControls'][0], axis='y', value=22)
        control._translateCtrlShape(self.rightArmRig.rigParts['scapulaControls'][0], axis='z', value=14)
        # switch
        control._rotateCtrlShape(self.rightArmRig.rigParts['switchControl'], axis='y', value=90)



        # Left leg

        # ik end control
        control._scaleCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='y', value=0.5)
        control._scaleCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='z', value=1.5)
        control._scaleCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='x', value=1.2)
        control._translateCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='z', value=5)
        control._translateCtrlShape(self.leftLegRig.rigParts['ikControl'], axis='y', value=-5)
        # gimbal control
        control._scaleCtrlShape(self.leftLegRig.rigParts['ikControls'][2], axis='x,y,z', value=1.2)
        control._rotateCtrlShape(self.leftLegRig.rigParts['ikControls'][2], axis='x', value=60)

        # Adjust foot controls
        control._scaleCtrlShape(self.leftLegRig.rigParts['footControls'][0], axis='x,y,z', value=1.5)
        control._scaleCtrlShape(self.leftLegRig.rigParts['footControls'][1], axis='x,y,z', value=1.5)

        # Adjust fk controls
        #  wrist
        control._scaleCtrlShape(self.leftLegRig.rigParts['fkControls'][2], axis='x,y,z', value=1.2)
        control._rotateCtrlShape(self.leftLegRig.rigParts['fkControls'][2], axis='x', value=60)
        # toe
        control._scaleCtrlShape(self.leftLegRig.rigParts['fkControls'][3], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.leftLegRig.rigParts['fkControls'][3], axis='x', value=30)
        # elbow
        control._scaleCtrlShape(self.leftLegRig.rigParts['fkControls'][1], axis='x,y,z', value=1.2)
        # shoulder
        control._scaleCtrlShape(self.leftLegRig.rigParts['fkControls'][0], axis='x,y,z', value=1.2)
        control._translateCtrlShape(self.leftLegRig.rigParts['fkControls'][0], axis='x', value=13)
        control._translateCtrlShape(self.leftLegRig.rigParts['fkControls'][0], axis='y', value=-4)
        control._translateCtrlShape(self.leftLegRig.rigParts['fkControls'][0], axis='z', value=-3)
        # scapula aim
        control._translateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][1], axis='x', value=12)
        control._translateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][1], axis='y', value=-3)
        control._translateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][1], axis='z', value=-2)
        # scapula
        control._translateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][0], axis='x', value=24)
        control._translateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][0], axis='y', value=22)
        control._translateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][0], axis='z', value=-5)
        control._rotateCtrlShape(self.leftLegRig.rigParts['scapulaControls'][0], axis='y', value= 31)
        # switch
        control._rotateCtrlShape(self.leftLegRig.rigParts['switchControl'], axis='y', value=90)



        # Right leg

        # ik end control
        control._scaleCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='y', value=0.5)
        control._scaleCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='z', value=1.5)
        control._scaleCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='x', value=1.2)
        control._translateCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='z', value=5)
        control._translateCtrlShape(self.rightLegRig.rigParts['ikControl'], axis='y', value=-5)
        # gimbal control
        control._scaleCtrlShape(self.rightLegRig.rigParts['ikControls'][2], axis='x,y,z', value=1.2)
        control._rotateCtrlShape(self.rightLegRig.rigParts['ikControls'][2], axis='x', value=60)

        # Adjust foot controls
        control._scaleCtrlShape(self.rightLegRig.rigParts['footControls'][0], axis='x,y,z', value=1.5)
        control._scaleCtrlShape(self.rightLegRig.rigParts['footControls'][1], axis='x,y,z', value=1.5)

        # Adjust fk controls
        #  wrist
        control._scaleCtrlShape(self.rightLegRig.rigParts['fkControls'][2], axis='x,y,z', value=1.2)
        control._rotateCtrlShape(self.rightLegRig.rigParts['fkControls'][2], axis='x', value=60)
        # toe
        control._scaleCtrlShape(self.rightLegRig.rigParts['fkControls'][3], axis='x,y,z', value=1.5)
        control._rotateCtrlShape(self.rightLegRig.rigParts['fkControls'][3], axis='x', value=30)
        # elbow
        control._scaleCtrlShape(self.rightLegRig.rigParts['fkControls'][1], axis='x,y,z', value=1.2)
        # shoulder
        control._scaleCtrlShape(self.rightLegRig.rigParts['fkControls'][0], axis='x,y,z', value=1.2)
        control._translateCtrlShape(self.rightLegRig.rigParts['fkControls'][0], axis='x', value=-13)
        control._translateCtrlShape(self.rightLegRig.rigParts['fkControls'][0], axis='y', value=-4)
        control._translateCtrlShape(self.rightLegRig.rigParts['fkControls'][0], axis='z', value=-3)
        # scapula aim
        control._translateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][1], axis='x', value=-12)
        control._translateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][1], axis='y', value=-3)
        control._translateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][1], axis='z', value=-2)
        # scapula
        control._translateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][0], axis='x', value=-24)
        control._translateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][0], axis='y', value=22)
        control._translateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][0], axis='z', value=-5)
        control._rotateCtrlShape(self.rightLegRig.rigParts['scapulaControls'][0], axis='y', value=-31)
        # switch
        control._rotateCtrlShape(self.rightLegRig.rigParts['switchControl'], axis='y', value=90)
        # left eye
        control._translateCtrlShape(self.leftEyeCtr, axis='x', value=1.517)
        control._translateCtrlShape(self.leftEyeCtr, axis='y', value=-0.012)
        control._translateCtrlShape(self.leftEyeCtr, axis='z', value=0.571)
        control._rotateCtrlShape(self.leftEyeCtr, axis='x', value=21.948)
        control._rotateCtrlShape(self.leftEyeCtr, axis='y', value=-20.621)
        control._rotateCtrlShape(self.leftEyeCtr, axis='z', value=-0.465)
        # right eye
        control._translateCtrlShape(self.rightEyeCtr, axis='x', value=-1.517)
        control._translateCtrlShape(self.rightEyeCtr, axis='y', value=-0.012)
        control._translateCtrlShape(self.rightEyeCtr, axis='z', value=0.571)
        control._rotateCtrlShape(self.rightEyeCtr, axis='x', value=21.948)
        control._rotateCtrlShape(self.rightEyeCtr, axis='y', value=20.621)
        control._rotateCtrlShape(self.rightEyeCtr, axis='z', value=0.465)

    def setInitialSettings(self):

        mc.setAttr('{}.{}'.format(self.baseRig.mainCtrl.C, 'jointsVis'), 0)

        self.neckRig.setInitialValues(HeadOrient=1, Stretchy=1)



        self.leftArmRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.leftArmRig.footRig.setInitialValues(BallRollAngle=45, ToeRollAngle=90)

        self.rightArmRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.rightArmRig.footRig.setInitialValues(BallRollAngle=0.1, ToeRollAngle=90)

        self.leftLegRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.leftLegRig.footRig.setInitialValues(BallRollAngle=45, ToeRollAngle=90)

        self.rightLegRig.setInitialValues(FKIKMode=1, Stretchy=0)
        self.rightLegRig.footRig.setInitialValues(BallRollAngle=45, ToeRollAngle=90)

