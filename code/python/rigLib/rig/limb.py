"""
limb FK/IK @ rig
"""

import maya.cmds as mc
from . import bendyLimb
from . import fkChain
from ..base import module
from ..base import control

from ..utils import joint

class Limb():

    def __init__(self,
            joints,
            type,
            scapulaJoint = '',
            prefix = 'limb',
            side = 'l',
            bendy = False,
            ikCtrOrient = 'bone',
            elbowDirection = '-z',
            forwardAxis = 'x',
            elbowSpinAxis = 'x',
            moveSwitchCtr = 'x, y',
            rigScale = 1.0,
            baseRig = None,
            ):
        """
        :param limb Joints: list(str), shoulder - elbow - wrist OR shoulder - elbow - wrist - toe1
        :param type: (str), type of limb. Options: "planti", "digi", or "ungu"
        :param scapulaJoint: str, scapula position joint
        :param prefix: str, prefix to name new objects. Default "arm"
        :param side: str, left of right side indicator. Default 'l'
        :param bendy: bool, option to build bendy limb controls. Default False
        :param ikCtrOrient: bool, option to orient the ik limb control to the bone or world. Default "bone"
        :param elbowDirection: str, local axis of elbow pole vector direction. Default '-z'
        :param forwardAxis: str, axis pointing down the joint chain. Default 'x'
        :param elbowSpinAxis: str, axis along which to spin the pole vector. Default 'x'
        :param moveSwitchCtr: str, axes along which to translate the switch control. Default 'x, y'

        :param buildFoot: bool, option to build a reverse foot rig at the end of the arm. Default False
        :param toeJoints: list(str), toe - toeEnd
        :param heelLoc: str, heel position locator
        :param innerLoc: str, inner rock position locator
        :param outerLoc: str, outer rock position locator
        :param rollAxis: str, axis to ball and toe joints up. Default "-z"
        :param rockAxis: str, axis to roll outer pivot outwards. Default "x"

        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        """

        self.type = type
        self.limbJoints = []

        for jnt in joints:
            newJnt = side + '_' + jnt
            self.limbJoints.append(newJnt)

        self.scapulaJoint = ''

        if scapulaJoint:
            self.scapulaJoint = side + '_' + scapulaJoint


        self.prefix = side + '_' + prefix
        self.side = side
        self.bendy = bendy
        self.elbowDirection = elbowDirection
        self.forwardAxis = forwardAxis
        self.elbowSpinAxis = elbowSpinAxis
        self.moveSwitchCtr = moveSwitchCtr

        self.ikCtrOrient = ikCtrOrient
        self.rigScale = rigScale
        self.baseRig = baseRig


        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        self.rigParts = {'bodyAttachGrp': '',
                         'footAttachGrp': '',
                         'fkControls': '',
                         'switchControl': '',
                         'ikControl': '',
                         'ikGimbalControl': '',
                         'reverseFootDriven': ''
                         }

    def build(self):

        # Make FK rig
        fkRig = self.buildFK()

        # Make IK rig
        if self.type == 'planti':
            ikRig = self.buildIKPlantigrade()
        elif self.type == 'digi':
            ikRig = self.buildIKDigitigrade()
        elif self.type == 'ungu':
            ikRig = self.buildIKUnguligrade()


        # Connect deformation joints to fk and ik joints

        pointConstraints = []
        orientConstraints = []

        # Make list of limb joints including scapula if there is one
        limb_joints = self.limbJoints

        for i in range(len(limb_joints)):
            pConstraint = mc.pointConstraint(fkRig['joints'][i], ikRig['joints'][i], limb_joints[i], mo=0)[0]
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], limb_joints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            pointConstraints.append(pConstraint)
            orientConstraints.append(oConstraint)

        # Make switch control

        self.switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo = limb_joints[1],
                                         color='green',
                                         scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='plus')
        switch_attr = 'FKIK_Switch'
        control._rotateCtrlShape(self.switchCtr, axis='x', value=90)

        mc.addAttr(self.switchCtr.C, ln=switch_attr, at='double', min=0, max=1, dv=0, k=1)

        self.rigParts['switchControl'] = self.switchCtr

        # Create class member so we can access later
        self.FKIKAttr = '{}.{}'.format(self.switchCtr.C, switch_attr)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in pointConstraints:
            weights = mc.pointConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for constraint in orientConstraints:
            weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        pointconstraintGrp = mc.group(pointConstraints, n='defSkeleton_{}_pconstraints'.format(self.prefix))
        orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
        mc.parent(pointconstraintGrp, self.baseRig.noXformGrp)
        mc.parent(orientconstraintGrp, self.baseRig.noXformGrp)

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

        # move switch ctr
        mc.move(units, self.switchCtr.Off, x=x, y=y, z=z, os=1, r=1, wd=1)

        # build scapula rig
        if self.scapulaJoint:
            scapRig = self.buildScapula()

            # connect FK and IK arm rigs to scapula
            mc.pointConstraint(scapRig['armAttach'], fkRig['controls'][0].Off, mo=1)
            mc.orientConstraint(scapRig['armAttach'], fkRig['controls'][0].Off, mo=1)
            mc.pointConstraint(scapRig['armAttach'], ikRig['baseAttachGrp'], mo=1)
            mc.orientConstraint(scapRig['armAttach'], ikRig['baseAttachGrp'], mo=1)

            self.rigParts['bodyAttachGrp'] = scapRig['bodyAttach']

        else:

            # Create body attach group
            bodyAttachGrp = mc.group(em=1, n='{}_bodyAttachGrp'.format(self.prefix))
            mc.delete(mc.parentConstraint(self.limbJoints[0], bodyAttachGrp, mo = 0))
            mc.parent(bodyAttachGrp, self.rigmodule.partsGrp)

            # connect FK and IK arm rigs to boddy attach group
            mc.pointConstraint(bodyAttachGrp, fkRig['controls'][0].Off, mo=1)
            mc.orientConstraint(bodyAttachGrp, fkRig['controls'][0].Off, mo=1)
            mc.pointConstraint(bodyAttachGrp, ikRig['baseAttachGrp'], mo=1)
            mc.orientConstraint(bodyAttachGrp, ikRig['baseAttachGrp'], mo=1)

            self.rigParts['bodyAttachGrp'] = bodyAttachGrp


        # Create foot attach group
        footAttachGrp = mc.group(em=1, n='{}_footAttachGrp'.format(self.prefix))
        mc.parent(footAttachGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(self.limbJoints[2], footAttachGrp, mo=0)

        # Set rigPart objects to call later
        self.rigParts['footAttachGrp'] = footAttachGrp

        # Set some class properties we can call later to set a T pose
        '''
        self.fkControls = fkRig['controls']
        self.pvControl = ikRig['controls'][1]
        self.ikRotateGrp = ikRig['rotateGrp']
        '''

        if self.bendy:
            self.buildBendyLimbs()

        # Create tpose/ a pose switch
        #self.tPose(self.fkControls, self.pvControl, self.ikRotateGrp)

    def buildFK(self):

        # duplicate limb joints to make FK joints
        fkJoints = joint.duplicateChain(self.limbJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)

        # make controls

        fkControlChain = fkChain.build(joints = fkJoints, rigScale=self.rigScale,
                                       shape='circleX', offsets= ['null', 'zero', 'auto'])

        controls = fkControlChain['controls']

        mc.parent(fkControlChain['controls'][0].Off, self.rigmodule.controlsGrp)

        self.rigParts['fkControls'] = controls




        return {'joints': fkJoints, 'controls': controls}

    def buildIKPlantigrade(self):

        # duplicate arm joints to make IK joints
        ikJoints = joint.duplicateChain(self.limbJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)

        # Make controls
        if self.ikCtrOrient == 'bone':
            orientation = ikJoints[2]

        elif self.ikCtrOrient == 'world':
            orientation = self.baseRig.global1Ctrl

        armCtr = control.Control(prefix='{}_arm_ik'.format(self.prefix), translateTo=ikJoints[-1], rotateTo=orientation,
                                 offsets=['null', 'zero', 'auto'],
                                 scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape='cube')

        poleVectorCtr = control.Control(prefix='{}_arm_pv'.format(self.prefix), translateTo=ikJoints[1],
                                        rotateTo=ikJoints[1], offsets=['null', 'zero', 'auto'],
                                        scale=self.rigScale * 0.25, parent=self.rigmodule.controlsGrp, shape='locator')

        wristGimbalCtr = control.Control(prefix='{}_hand_gimbal'.format(self.prefix), translateTo=ikJoints[2],
                                         rotateTo=ikJoints[2],
                                         scale=self.rigScale, lockChannels=['t', 's'], shape='circle')

        controls = [armCtr, poleVectorCtr, wristGimbalCtr]

        self.rigParts['ikControl'] = armCtr
        #self.rigParts['ikControls'] = controls
        self.rigParts['ikGimbalControl'] = wristGimbalCtr



        # Shoulder attach group moves the base of the ik limb. Is used to attach to a scaupla rig or spine rig later
        shoulderAttachGrp = mc.group(n='{}_ikShoulderJnt_driver'.format(self.prefix), em=1, p=self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], shoulderAttachGrp, mo=0))

        # First ik joint should follow shoulderAttachGro
        mc.pointConstraint(shoulderAttachGrp, ikJoints[0], mo=1)

        # ikEndGrp moves the IK handle and objects that are meant to be the "end" of the IK. Follows the IK control, but can be used to follow a reverse foot setup later.
        ikEndGrp = mc.group(n='{}_ikEndEffectors'.format(self.prefix), em=1)
        mc.parentConstraint(armCtr.C, ikEndGrp, mo=0)
        mc.parent(ikEndGrp, self.rigmodule.partsGrp)

        # Add followArmIK to rigParts dictionary bc we will need it in case of a reverse foot setup

        self.rigParts['reverseFootDriven'] = ikEndGrp

        # move pole vector ctr
        units = 5 * self.rigScale

        if 'x' in self.elbowDirection:
            x, y, z = True, False, False

        elif 'y' in self.elbowDirection:
            x, y, z = False, True, False

        elif 'z' in self.elbowDirection:
            x, y, z = False, False, True

        if '-' in self.elbowDirection:
            units = units * -1

        mc.move(units, poleVectorCtr.Off, x=x, y=y, z=z, os=1, r=1, wd=1)

        # make IK handle
        armIK = mc.ikHandle(n='{}_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[-1])[0]
        self.rigParts['ikHandle'] = armIK

        mc.hide(armIK)

        # Parent ikhandle to follow End group
        mc.parent(armIK, ikEndGrp)

        # make pole vector connection line
        pvLinePos1 = mc.xform(ikJoints[1], q=1, t=1, ws=1)
        pvLinePos2 = mc.xform(poleVectorCtr.C, q=1, t=1, ws=1)
        poleVectorCurve = mc.curve(n='{}_pv_curve'.format(self.prefix), d=1, p=[pvLinePos1, pvLinePos2])

        mc.cluster('{}.cv[0]'.format(poleVectorCurve), n='{}_pv1_cls'.format(self.prefix),
                   wn=[ikJoints[1], ikJoints[1]], bs=True)

        mc.cluster('{}.cv[1]'.format(poleVectorCurve), n='{}_pv2_cls'.format(self.prefix),
                   wn=[poleVectorCtr.C, poleVectorCtr.C], bs=True)

        mc.parent(poleVectorCurve, self.rigmodule.controlsGrp)
        mc.setAttr('{}.template'.format(poleVectorCurve), 1)
        mc.setAttr('{}.it'.format(poleVectorCurve), 0)

        # create PV no follow locator
        pvNoFollow = mc.spaceLocator(n='{}_pv_noFollow'.format(self.prefix))
        mc.parent(pvNoFollow, self.rigmodule.partsGrp)
        mc.matchTransform(pvNoFollow, poleVectorCtr.C)

        # create PV follow object and setup
        poleAimLeg = mc.group(n='{}_poleAim'.format(self.prefix), em=1)
        upVector = mc.group(n='{}_pv_upVec'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, upVector, mo=0))
        mc.parent(upVector, self.rigmodule.partsGrp)
        mc.parentConstraint(armCtr.C, upVector, sr='x', mo=1)
        mc.pointConstraint(ikJoints[0], poleAimLeg, mo=0)

        poleAimConstraint = mc.aimConstraint(armCtr.C, poleAimLeg, aimVector=(1, 0, 0), upVector=(0, 0, 1),
                                             worldUpType='objectRotation', worldUpVector=(0, 0, 1),
                                             worldUpObject=upVector, mo=0)[0]

        poleOffsetFollow_noScale = mc.group(n='{}_poleOffsetFollow_noScale'.format(self.prefix), em=1)
        mc.pointConstraint(poleAimLeg, poleOffsetFollow_noScale, mo=0)
        mc.orientConstraint(poleAimLeg, poleOffsetFollow_noScale, mo=0)
        pvFollow = mc.group(n='{}_poleOffsetFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(poleVectorCtr.C, pvFollow, mo=0))
        mc.parentConstraint(poleOffsetFollow_noScale, pvFollow, mo=1)
        mc.parent(poleAimLeg, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow_noScale, self.rigmodule.partsGrp)
        mc.parent(pvFollow, self.rigmodule.partsGrp)

        # constrain pv
        pv_constraint = mc.parentConstraint(pvFollow, pvNoFollow, poleVectorCtr.Offsets[0], mo=1)[0]
        weights = mc.parentConstraint(pv_constraint, q=1, weightAliasList=1)
        mc.setAttr('{}.interpType'.format(pv_constraint), 2)

        # setup pv follow switch
        pv_follow_attr = 'Follow'
        mc.addAttr(poleVectorCtr.C, ln=pv_follow_attr, at='double', min=0, max=1, dv=0, k=1)
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.{}'.format(pv_constraint, weights[0]))
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_pvFollow_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), '{}.inputX'.format(reverse))
        mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(pv_constraint, weights[1]))
        mc.parent(pv_constraint, self.rigmodule.noXformGrp)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_follow_attr), 1)

        # Add spin attribute

        spin_attr = 'Spin'
        mc.addAttr(armCtr.C, ln=spin_attr, at='double', dv=0, k=1)

        self.elbowSpinAxis = self.forwardAxis

        if 'x' in self.elbowSpinAxis:
            spinAxisAttr = 'offsetX'
        elif 'y' in self.elbowSpinAxis:
            spinAxisAttr = 'offsetY'
        elif 'z' in self.elbowSpinAxis:
            spinAxisAttr = 'offsetZ'

        mc.connectAttr('{}.{}'.format(armCtr.C, spin_attr), '{}.{}'.format(poleAimConstraint, spinAxisAttr))


        # Create PV constraint
        mc.poleVectorConstraint(poleVectorCtr.C, armIK)
        # Orient constraint joint to gimbal control
        mc.orientConstraint(wristGimbalCtr.C, ikJoints[2], mo=1)
        # parent gimbal control to ik control
        mc.parent(wristGimbalCtr.Off, armCtr.C)





        # make stretchy arm

        # Create locator to follow shoulder
        shoulderLoc = mc.group(n='{}_IKShoulderLoc'.format(self.prefix), em=1)
        mc.parent(shoulderLoc, shoulderAttachGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], shoulderLoc, mo=0))

        # Create group to follow hand IK control
        followWrist = mc.group(n='{}_IKWristFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, followWrist, mo=0))
        # create empty group under ik arm ctr
        followArmIK = mc.group(n='{}_IKArmFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, followArmIK, mo=0))
        mc.parent(followArmIK, ikEndGrp)
        # point constrain wrist group to group under ik arm ctr
        mc.parentConstraint(followArmIK, followWrist, mo=0)
        mc.parent(followWrist, self.rigmodule.partsGrp)



        # create a distance node to get the length between the two groups
        arm_dist = mc.shadingNode('distanceBetween', asUtility=True, n='{}_arm_length'.format(self.prefix))
        # connect distance node to pv ctr world matrix and group positioned at wrist
        mc.connectAttr('{}.worldMatrix'.format(shoulderLoc), '{}.inMatrix1'.format(arm_dist))
        mc.connectAttr('{}.worldMatrix'.format(followWrist), '{}.inMatrix2'.format(arm_dist))

        # divide arm length by global scale
        arm_dist_global = mc.shadingNode('multiplyDivide', asUtility=True,
                                         n='{}_arm_distance_global'.format(self.prefix))
        mc.setAttr('{}.operation'.format(arm_dist_global), 2)
        mc.connectAttr('{}.distance'.format(arm_dist), '{}.input1X'.format(arm_dist_global))
        mc.connectAttr('{}.sx'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(arm_dist_global))

        # Negate leg distance value for right side
        if '-' in self.forwardAxis:
            arm_dist_negative = mc.shadingNode('multiplyDivide', asUtility=True,
                                               n='{}_arm_distance_negative'.format(self.prefix))
            mc.setAttr('{}.operation'.format(arm_dist_negative), 1)
            mc.connectAttr('{}.outputX'.format(arm_dist_global), '{}.input1X'.format(arm_dist_negative))
            mc.setAttr('{}.input2X'.format(arm_dist_negative), -1)
            sdkDriver = '{}.outputX'.format(arm_dist_negative)

        else:
            sdkDriver = '{}.outputX'.format(arm_dist_global)

        # Get the original length of the upper and lower arm joints

        if 'x' in self.forwardAxis:
            lengthAxisAttr = 'tx'
        elif 'y' in self.forwardAxis:
            lengthAxisAttr = 'ty'
        elif 'z' in self.forwardAxis:
            lengthAxisAttr = 'tz'

        upperArm_length = mc.getAttr('{}.{}'.format(ikJoints[1], lengthAxisAttr))
        lowerArm_length = mc.getAttr('{}.{}'.format(ikJoints[2], lengthAxisAttr))

        # Calculate the length of fully extended arm
        armLength = upperArm_length + lowerArm_length

        # Create blender for stretchy arm setup
        stretch_attr = 'Stretchy'
        mc.addAttr(armCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)
        # Create class member so we can access later
        self.StretchyAttr = '{}.{}'.format(armCtr.C, stretch_attr)

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        mc.addAttr(armCtr.C, ln=length1_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(armCtr.C, ln=length2_attr, at='double', min=1, dv=1, k=1)

        if '-' in self.forwardAxis:
            fAxisDirection = 0
        else:
            fAxisDirection = 1

        blenderStretchUpperArm = boneStretch(prefix = '{}_bone1'.format(self.prefix),
                        boneOrigLength = upperArm_length,
                        totalLimbLength = armLength,
                        lengthAttr = '{}.{}'.format(armCtr.C, length1_attr),
                        stretchAttr = self.StretchyAttr,
                        stretchDriver = sdkDriver ,
                        forwardAxisPositive = fAxisDirection
                        )

        blenderStretchLowerArm = boneStretch(prefix = '{}_bone2'.format(self.prefix),
                                             boneOrigLength = lowerArm_length,
                                             totalLimbLength = armLength,
                                             lengthAttr = '{}.{}'.format( armCtr.C, length2_attr),
                                             stretchAttr = self.StretchyAttr,
                                             stretchDriver = sdkDriver,
                                             forwardAxisPositive = fAxisDirection
                                             )


        # make elbow pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        blenderPinUpperArm = poleVectorPin(prefix = '{}_bone1'.format(self.prefix),
                                           pinAttr = '{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                           pvCtrl = poleVectorCtr.C,
                                           boneLocator = shoulderLoc,
                                           globalCtrl = '{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                           forwardAxisPositive = fAxisDirection
                                           )

        blenderPinLowerArm = poleVectorPin(prefix = '{}_bone2'.format(self.prefix),
                                           pinAttr = '{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                           pvCtrl = poleVectorCtr.C,
                                           boneLocator = followWrist,
                                           globalCtrl = '{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                           forwardAxisPositive = fAxisDirection
                                           )



        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderStretchUpperArm), '{}.input[0]'.format(blenderPinUpperArm))
        mc.connectAttr('{}.output'.format(blenderStretchLowerArm), '{}.input[0]'.format(blenderPinLowerArm))

        # connect pin/ no pin blender output to ik elbow joint translate X
        mc.connectAttr('{}.output'.format(blenderPinUpperArm), '{}.{}'.format(ikJoints[1], lengthAxisAttr))
        mc.connectAttr('{}.output'.format(blenderPinLowerArm), '{}.{}'.format(ikJoints[2], lengthAxisAttr))

        mc.setAttr('{}.{}'.format(armCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)


        return {'joints': ikJoints, 'controls': controls, 'poleVecLine': poleVectorCurve,
                'baseAttachGrp': shoulderAttachGrp }




    def buildScapula(self):

        # Make scapula control

        scapCtr = control.Control(prefix='{}_scapula'.format(self.prefix), translateTo= self.scapulaJoint,
                                  rotateTo = self.scapulaJoint, scale=self.rigScale, parent=self.rigmodule.controlsGrp,
                                  shape = 'quadArrow', offsets= ['null', 'zero', 'auto'])

        scapAimCtr = control.Control(prefix='{}_scapula_translate'.format(self.prefix), translateTo= self.limbJoints[0], rotateTo= self.limbJoints[0],
                                  scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape = 'squareY',
                                     lockChannels= ['r', 's'])

        self.rigParts['scapulaControls'] = [scapCtr, scapAimCtr]

        # setup aim constraint on scapCtr

        upVector = mc.group(n = '{}_aim_upVec'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(scapAimCtr.C, upVector, mo=0))
        mc.parent(upVector, scapAimCtr.C)

        mc.aimConstraint( scapAimCtr.C, scapCtr.Offsets[2], aimVector=(1, 0, 0), upVector=(0, 1, 0),
                         worldUpType='objectRotation', worldUpVector=(0, 1, 0), worldUpObject=upVector, mo = 1)

        # connect scapula joint to control
        constraint = mc.parentConstraint(scapCtr.C, self.scapulaJoint, mo = 1 )[0]
        mc.parent(constraint, self.rigmodule.noXformGrp)

        # make locator to follow end of clavicle
        endPos = mc.spaceLocator( n = '{}_scapEnd_pos'.format(self.prefix) )
        mc.parent(endPos, self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(self.limbJoints[0], endPos))
        mc.parent(endPos, scapCtr.C)
        mc.hide(endPos)

        # make body attach group
        bodyAttachGrp = mc.group(n= '{}_scap_bodyAttachGrp'.format(self.prefix),em = 1, p = self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(self.scapulaJoint, bodyAttachGrp, mo = 0) )

        mc.parentConstraint( bodyAttachGrp, scapCtr.Off, mo = 1)
        mc.parentConstraint(bodyAttachGrp, scapAimCtr.Off, mo=1)

        return {'armAttach': endPos, 'bodyAttach': bodyAttachGrp}

    def buildBendyLimbs(self):
        pass

    def buildIKDigitigrade(self):
        pass
    def buildIKUnguligrade(self):
        pass

    def setInitialValues(self,
                         FKIKMode=1,
                         Stretchy=1,
                         ):

        mc.setAttr(self.FKIKAttr, FKIKMode)
        mc.setAttr(self.StretchyAttr, Stretchy)


def boneStretch( prefix, boneOrigLength, totalLimbLength, lengthAttr, stretchAttr, stretchDriver, forwardAxisPositive):
    '''

    :param prefix: prefix to name nodes created
    :param boneOrigLength: original length of bone (no stretch amount)
    :param totalLimbLength: sum of bone lengths making up the limb
    :param lengthAttr: attribute to increase bone length manually
    :param stretchBlender: attribute to blend between stretch and non-stretch bone length
    :param stretchDriver: will drive our SDK, its the distance between the shoulder and end control
    :param forwardAxisPositive: (bool) true if foward axis is positive, false if negative
    :return: stretch/no stretch blender node
    '''


    # make blender node to switch between stretch and non-stretched amount
    blenderStretch = mc.shadingNode('blendTwoAttr', asUtility=True,
                                            n='{}_blender_stretch'.format(prefix))
    mc.connectAttr(stretchAttr, '{}.attributesBlender'.format(blenderStretch))

    # Multiply stretch amount by length multiplier
    length_stretch_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length_mult'.format(prefix))
    mc.setAttr('{}.operation'.format(length_stretch_mult), 1)
    mc.connectAttr(lengthAttr, '{}.input1X'.format(length_stretch_mult))

    # Have leg stretchy SDK feed into this multDiv node
    sdkDriven = '{}.input2X'.format(length_stretch_mult)


    # Feed result of multDiv node to blender stretch value
    mc.connectAttr('{}.outputX'.format(length_stretch_mult), '{}.input[1]'.format(blenderStretch))

    # Multiply non stretchy leg by length attribute
    length_noStretch_mult = mc.shadingNode('multiplyDivide', asUtility=True,
                                            n='{}_length_noStretch_mult'.format(prefix))
    mc.setAttr('{}.operation'.format(length_noStretch_mult), 1)
    mc.connectAttr(lengthAttr, '{}.input1X'.format(length_noStretch_mult))
    mc.setAttr('{}.input2X'.format(length_noStretch_mult), boneOrigLength)

    # Feed result of multDiv node to blender non-stretch value
    mc.connectAttr('{}.outputX'.format(length_noStretch_mult), '{}.input[0]'.format(blenderStretch))


    # Set up SDK to stretch arm after it's extended beyond its length

    # upper leg
    mc.setDrivenKeyframe(sdkDriven,
                         currentDriver = stretchDriver,
                         driverValue = totalLimbLength,
                         value = boneOrigLength,
                         inTangentType = 'linear',
                         outTangentType = 'linear')

    mc.setDrivenKeyframe(sdkDriven,
                         currentDriver = stretchDriver,
                         driverValue = totalLimbLength * 2,
                         value = boneOrigLength * 2,
                         inTangentType='spline',
                         outTangentType='spline')

    animCurve = mc.keyframe(sdkDriven, query=True, name=True)[0]

    if forwardAxisPositive:
        mc.setAttr('{}.postInfinity'.format(animCurve), 1)

    else:
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)

    return blenderStretch

def poleVectorPin( prefix, pinAttr, pvCtrl, boneLocator, globalCtrl, forwardAxisPositive):

    """

    :param prefix: prefix to name nodes created
    :param pinAttr: (str) object.attribute of Pin switch
    :param pvCtrl: (str) name of pole vector control
    :param boneLocator: (str) name of locator positioned at the bone to pin
    :param globalCtrl: (str) name.attr of global scaling control
    :param forwardAxisPositive: (bool) true if foward axis is positive, false if negative
    :return: pin/no pin blender node
    """

    # make blender node which will blend between pin and no pin values
    blenderPin = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_blenderPin'.format(prefix))
    mc.connectAttr(pinAttr, '{}.attributesBlender'.format(blenderPin))

    # get distance between boneLocator and PV control
    bone_pv_dist = mc.shadingNode('distanceBetween', asUtility=True,
                                      n='{}_bone_2_pv_distance'.format(prefix))
    # connect distance node to shoulder location and pv ctr world matrix
    mc.connectAttr('{}.worldMatrix'.format(boneLocator), '{}.inMatrix1'.format(bone_pv_dist))
    mc.connectAttr('{}.worldMatrix'.format(pvCtrl), '{}.inMatrix2'.format(bone_pv_dist))

    # make multiply divide node to take into account global scale
    boneGlobalLength = mc.shadingNode('multiplyDivide', asUtility=True,
                                       n='{}_boneGlobalLength'.format(prefix))
    mc.setAttr('{}.operation'.format(boneGlobalLength), 2)
    mc.connectAttr('{}.distance'.format(bone_pv_dist), '{}.input1X'.format(boneGlobalLength))
    mc.connectAttr(globalCtrl, '{}.input2X'.format(boneGlobalLength))

    if forwardAxisPositive:

        # connect distance from shoulder to pv ctr to blender input
        mc.connectAttr('{}.outputX'.format(boneGlobalLength), '{}.input[1]'.format(blenderPin))

    else:
        # multiply length value by -1
        negateOutput = mc.shadingNode('multiplyDivide', asUtility=True,
                                            n='{}_negate'.format(prefix))
        mc.setAttr('{}.operation'.format(negateOutput), 1)
        mc.connectAttr('{}.outputX'.format(boneGlobalLength), '{}.input1X'.format(negateOutput))
        mc.setAttr('{}.input2X'.format(negateOutput), -1)
        mc.connectAttr('{}.outputX'.format(negateOutput), '{}.input[1]'.format(blenderPin))

    return blenderPin



