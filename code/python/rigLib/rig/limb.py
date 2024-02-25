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
                         'FKIKSwitchAttr': '',
                         'ikControl': '',
                         'ikJoints': '',
                         'fkJoints': '',
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

        # Define rigParts dictionary items
        self.rigParts['fkJoints'] = fkRig['joints']
        self.rigParts['ikJoints'] = ikRig['joints']
        self.rigParts['fkControls'] = fkRig['controls']


        # Make switch control

        self.switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo = self.limbJoints[1],
                                         color='green',
                                         scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='plus')
        switch_attr = 'FKIK_Switch'
        control._rotateCtrlShape(self.switchCtr, axis='x', value=90)

        mc.addAttr(self.switchCtr.C, ln=switch_attr, at='double', min=0, max=1, dv=0, k=1)

        self.rigParts['switchControl'] = self.switchCtr
        self.rigParts['FKIKSwitchAttr'] = '{}.{}'.format(self.switchCtr.C, switch_attr)

        # Create class member so we can access later
        self.FKIKAttr = '{}.{}'.format(self.switchCtr.C, switch_attr)


        # Connect deformation joints to fk and ik joints
        orientConstraints = []

        for i in range(len(self.limbJoints)):
            oConstraint = mc.orientConstraint(fkRig['joints'][i], ikRig['joints'][i], self.limbJoints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(oConstraint), 2)
            orientConstraints.append(oConstraint)


        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in orientConstraints:
            weights = mc.orientConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        # Setup blend between joint scales

        for i in range(len(self.limbJoints)):
            blendNode = mc.shadingNode('blendColors', asUtility=True,
                                       n='{}_jointScale_blend{}'.format(self.prefix, i))
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.blender'.format(blendNode))

            mc.connectAttr('{}.sx'.format(ikRig['joints'][i]), '{}.color1.color1R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(ikRig['joints'][i]), '{}.color1.color1G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(ikRig['joints'][i]), '{}.color1.color1B'.format(blendNode))

            mc.connectAttr('{}.sx'.format(fkRig['joints'][i]), '{}.color2.color2R'.format(blendNode))
            mc.connectAttr('{}.sy'.format(fkRig['joints'][i]), '{}.color2.color2G'.format(blendNode))
            mc.connectAttr('{}.sz'.format(fkRig['joints'][i]), '{}.color2.color2B'.format(blendNode))

            mc.connectAttr('{}.outputR'.format(blendNode), '{}.sx'.format(self.limbJoints[i]))
            mc.connectAttr('{}.outputG'.format(blendNode), '{}.sy'.format(self.limbJoints[i]))
            mc.connectAttr('{}.outputB'.format(blendNode), '{}.sz'.format(self.limbJoints[i]))


        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(self.switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        orientconstraintGrp = mc.group(orientConstraints, n='defSkeleton_{}_oconstraints'.format(self.prefix))
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

        # Create body attach group
        bodyAttachGrp = mc.group(em=1, n='{}_bodyAttachGrp'.format(self.prefix))
        mc.delete(mc.parentConstraint(self.limbJoints[0], bodyAttachGrp, mo=0))
        mc.parent(bodyAttachGrp, self.rigmodule.partsGrp)

        # connect FK and IK arm rigs to boddy attach group
        mc.parent( fkRig['bodyAttachGrp'],bodyAttachGrp )
        mc.parent( ikRig['bodyAttachGrp'], bodyAttachGrp )


        # build scapula rig
        if self.scapulaJoint:

            scapRig = self.buildScapula()

            # connect FK and IK arm rigs to scapula
            mc.pointConstraint(scapRig['armAttach'],bodyAttachGrp, mo=1)
            mc.orientConstraint(scapRig['armAttach'], bodyAttachGrp, mo=1)

            self.rigParts['bodyAttachGrp'] = scapRig['bodyAttach']

        else:
            self.rigParts['bodyAttachGrp'] = bodyAttachGrp


        # Create foot attach group
        footAttachGrp = mc.group(em=1, n='{}_footAttachGrp'.format(self.prefix))
        mc.parent(footAttachGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(self.limbJoints[2], footAttachGrp, mo=0)

        # Set rigPart objects to call later
        self.rigParts['footAttachGrp'] = footAttachGrp

        if self.bendy:
            self.buildBendyLimbs()

    def buildFK(self):

        # duplicate limb joints to make FK joints
        fkJoints = joint.duplicateChain(self.limbJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)

        # make controls

        fkControlChain = fkChain.build(joints = fkJoints, rigScale=self.rigScale,
                                       shape='circleX', offsets= ['null', 'zero', 'auto'])

        controls = fkControlChain['controls']

        mc.parent(fkControlChain['controls'][0].Off, self.rigmodule.controlsGrp)

        # Create body attach group
        bodyAttachGrp = mc.group(em=1, n='{}_fkBodyAttachGrp'.format(self.prefix))
        mc.delete(mc.parentConstraint(self.limbJoints[0], bodyAttachGrp, mo=0))
        mc.parent(bodyAttachGrp, self.rigmodule.partsGrp)

        # Have top fk control follow the body attach group
        mc.pointConstraint(bodyAttachGrp, controls[0].Off, mo=1)
        mc.orientConstraint(bodyAttachGrp, controls[0].Off, mo=1)


        return {'joints': fkJoints, 'controls': controls, 'bodyAttachGrp': bodyAttachGrp}

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

        wristGimbalCtr = control.Control(prefix='{}_hand_gimbal'.format(self.prefix), translateTo=ikJoints[-1],
                                         rotateTo=ikJoints[1],
                                         scale=self.rigScale, lockChannels=['t', 's'], shape='circle')

        controls = [armCtr, poleVectorCtr, wristGimbalCtr]

        self.rigParts['ikControl'] = armCtr
        self.rigParts['ikGimbalControl'] = wristGimbalCtr



        # Shoulder attach group moves the base of the ik limb. Is used to attach to a scaupla rig or spine rig later
        bodyAttachGrp = mc.group(n='{}_ikBodyAttachGrp'.format(self.prefix), em=1, p=self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], bodyAttachGrp, mo=0))

        # First ik joint should follow shoulderAttachGro
        mc.pointConstraint(bodyAttachGrp, ikJoints[0], mo=1)

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
        mc.parent(shoulderLoc, bodyAttachGrp)
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
            scaleAxisAttr = 'sx'
        elif 'y' in self.forwardAxis:
            lengthAxisAttr = 'ty'
            scaleAxisAttr = 'sy'
        elif 'z' in self.forwardAxis:
            lengthAxisAttr = 'tz'
            scaleAxisAttr = 'sz'

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

        blenderStretchUpper = boneStretch(prefix='{}_bone1'.format(self.prefix),
                                          totalLimbLength=armLength,
                                          lengthAttr='{}.{}'.format(armCtr.C, length1_attr),
                                          stretchAttr=self.StretchyAttr,
                                          stretchDriver=sdkDriver,
                                          forwardAxisPositive=fAxisDirection
                                          )

        blenderStretchLower = boneStretch(prefix='{}_bone2'.format(self.prefix),
                                          totalLimbLength=armLength,
                                          lengthAttr='{}.{}'.format(armCtr.C, length2_attr),
                                          stretchAttr=self.StretchyAttr,
                                          stretchDriver=sdkDriver,
                                          forwardAxisPositive=fAxisDirection
                                          )


        # make elbow pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)

        blenderPinUpper = poleVectorPin(prefix='{}_bone1'.format(self.prefix),
                                        pinAttr='{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl=poleVectorCtr.C,
                                        boneLocator=shoulderLoc,
                                        boneOrigLength=upperArm_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive=fAxisDirection
                                        )

        blenderPinLower = poleVectorPin(prefix='{}_bone2'.format(self.prefix),
                                        pinAttr='{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl=poleVectorCtr.C,
                                        boneLocator=followWrist,
                                        boneOrigLength=lowerArm_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive=fAxisDirection
                                        )


        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderStretchUpper), '{}.input[0]'.format(blenderPinUpper))
        mc.connectAttr('{}.output'.format(blenderStretchLower), '{}.input[0]'.format(blenderPinLower))

        # connect blender outputs to ik joints' translate X
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.{}'.format(ikJoints[0], scaleAxisAttr))
        mc.connectAttr('{}.output'.format(blenderPinLower), '{}.{}'.format(ikJoints[1], scaleAxisAttr))

        mc.setAttr('{}.{}'.format(armCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)


        return {'joints': ikJoints, 'controls': controls, 'poleVecLine': poleVectorCurve,
                'bodyAttachGrp': bodyAttachGrp }




    def buildScapula(self):

        # make body attach group
        bodyAttachGrp = mc.group(n='{}_scap_bodyAttachGrp'.format(self.prefix), em=1, p=self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(self.scapulaJoint, bodyAttachGrp, mo=0))


        # Make scapula control

        scapCtr = control.Control(prefix='{}_scapula'.format(self.prefix), translateTo= self.scapulaJoint,
                                  rotateTo = self.scapulaJoint, scale=self.rigScale, parent=self.rigmodule.controlsGrp,
                                  shape = 'quadArrow', offsets= ['null', 'zero', 'auto'])

        self.rigParts['scapulaControls'] = [scapCtr]

        mc.parentConstraint(bodyAttachGrp, scapCtr.Off, mo=1)


        # setup aim and no aim locators
        scapulaGrp = mc.group(n = '{}_scapulaLocatorsGrp'.format(self.prefix), em = 1)
        noAimLoc = mc.spaceLocator(n='{}_scapula_NoAim'.format(self.prefix))
        aimLoc = mc.spaceLocator(n='{}_scapula_Aim'.format(self.prefix))

        mc.parent(noAimLoc, scapulaGrp)
        mc.parent(aimLoc, scapulaGrp)
        mc.parent(scapulaGrp, self.rigmodule.partsGrp)

        mc.parentConstraint(bodyAttachGrp, scapulaGrp, mo = 0)

        ikControl = self.rigParts['ikControl'].C

        # Create group that follows wrist result joint
        wristPositionGrp = mc.group(n='{}_ikWristResultGrp'.format(self.prefix), em=1)
        mc.parent(wristPositionGrp, self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(ikControl, wristPositionGrp, mo=0))


        # Make upvector that follows end result of wrist
        upVector = mc.group(n='{}_aim_upVec'.format(self.prefix), em=1)
        mc.delete(mc.pointConstraint(ikControl, upVector, mo=0))
        mc.delete(mc.orientConstraint(aimLoc, upVector, mo=0))
        mc.parent(upVector, wristPositionGrp)

        fkWristPositionGrp = mc.group(n='{}_fkWristResultGrp'.format(self.prefix), em=1)
        mc.parent(fkWristPositionGrp, bodyAttachGrp)
        mc.delete(mc.parentConstraint(ikControl, fkWristPositionGrp, mo = 0))

        # Constrain wristPosition group between ik control and a static position for when in FK mode
        wristPosConstraint = mc.pointConstraint(fkWristPositionGrp, ikControl, wristPositionGrp, mo= 1)[0]
        wristWeights = mc.pointConstraint(wristPosConstraint, q=1, weightAliasList = 1)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_wristAimSwitch_reverse'.format(self.prefix))
        mc.connectAttr(self.rigParts['FKIKSwitchAttr'], '{}.inputX'.format(reverse))

        mc.connectAttr(self.rigParts['FKIKSwitchAttr'], '{}.{}'.format(wristPosConstraint, wristWeights[1]))
        mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(wristPosConstraint, wristWeights[0]))


        mc.aimConstraint( wristPositionGrp, aimLoc, aimVector=(-1, 0, 0), upVector=(0, 1, 0),
                         worldUpType='objectRotation', worldUpVector=(0, 1, 0), worldUpObject=upVector, mo = 1)

        # Orient constraint scapula control offset between aim and no aim locators
        oconstraint = mc.orientConstraint(aimLoc, noAimLoc, scapCtr.Offsets[1], mo = 1)[0]
        weights = mc.orientConstraint(oconstraint, q=1, weightAliasList=1)
        mc.setAttr('{}.interpType'.format(oconstraint), 2)

        # Create attribute to drive scapula aim
        scap_aim_attr = 'Scapula_Aim'
        mc.addAttr(ikControl, ln = scap_aim_attr, at='double', min=0, max=1, dv=0, k=1)

        # Create remap node
        remap = mc.shadingNode('remapValue', asUtility=True, n='{}_scapAim_remap'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(ikControl, scap_aim_attr), '{}.inputValue'.format(remap))
        mc.setAttr( '{}.inputMax'.format(remap), 1)
        mc.setAttr('{}.inputMin'.format(remap), 0)
        mc.setAttr('{}.outputMax'.format(remap), 0.5)
        mc.setAttr('{}.outputMin'.format(remap), 0)


        mc.connectAttr('{}.outValue'.format(remap), '{}.{}'.format(oconstraint, weights[0]))
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_scapAim_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(ikControl, scap_aim_attr), '{}.inputX'.format(reverse))
        mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(oconstraint, weights[1]))
        mc.parent(oconstraint, self.rigmodule.noXformGrp)
        mc.setAttr('{}.{}'.format(ikControl, scap_aim_attr), 1.0)


        # connect scapula joint to control
        constraint = mc.parentConstraint(scapCtr.C, self.scapulaJoint, mo = 1 )[0]
        mc.parent(constraint, self.rigmodule.noXformGrp)

        # make locator to follow end of clavicle
        endPos = mc.spaceLocator( n = '{}_scapEnd_pos'.format(self.prefix) )
        mc.parent(endPos, self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(self.limbJoints[0], endPos))
        mc.parent(endPos, scapCtr.C)
        mc.hide(endPos)


        return {'armAttach': endPos, 'bodyAttach': bodyAttachGrp}

    def buildBendyLimbs(self):
        pass

    def buildIKDigitigrade(self):

        # duplicate arm joints to make IK joints
        ikJoints = joint.duplicateChain(self.limbJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)

        # Make controls
        if self.ikCtrOrient == 'bone':
            orientation = ikJoints[-1]

        elif self.ikCtrOrient == 'world':
            orientation = self.baseRig.global1Ctrl

        armCtr = control.Control(prefix='{}_arm_ik'.format(self.prefix), translateTo=ikJoints[-1], rotateTo=orientation,
                                 offsets=['null', 'zero', 'auto'],
                                 scale=self.rigScale, parent=self.rigmodule.controlsGrp, shape='cube')

        poleVectorCtr = control.Control(prefix='{}_arm_pv'.format(self.prefix), translateTo=ikJoints[1],
                                        rotateTo=ikJoints[1], offsets=['null', 'zero', 'auto'],
                                        scale=self.rigScale * 0.25, parent=self.rigmodule.controlsGrp, shape='locator')

        wristGimbalCtr = control.Control(prefix='{}_hand_gimbal'.format(self.prefix), translateTo=ikJoints[-1],
                                         rotateTo=ikJoints[-1],
                                         scale=self.rigScale, lockChannels=['t', 's'], shape='circle')

        # Create control at the ball of the foot
        ballCtr = control.Control(prefix='{}_footRoll'.format(self.prefix), translateTo = ikJoints[-1],
                                  scale=self.rigScale * 0.5, shape='circleX')

        controls = [armCtr, poleVectorCtr, wristGimbalCtr, ballCtr]

        self.rigParts['ikControl'] = armCtr
        self.rigParts['ikGimbalControl'] = wristGimbalCtr

        # Body attach group moves the base of the ik limb. Is used to attach to a scaupla rig or spine rig later
        bodyAttachGrp = mc.group(n='{}_ikBodyAttachGrp'.format(self.prefix), em=1, p=self.rigmodule.partsGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], bodyAttachGrp, mo=0))

        # First ik joint should follow shoulderAttachGrp
        mc.pointConstraint(bodyAttachGrp, ikJoints[0], mo=1)

        # ikEndGrp moves the IK handle and objects that are meant to be the "end" of the IK.
        # In the plantigrabe limb, this follows the IK control, but here we'll go ahead and add it to the ball roll control
        ikEndGrp = mc.group(n='{}_ikEndEffectors'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(ballCtr.C, ikEndGrp, mo=0))
        mc.parent(ikEndGrp, ballCtr.C)

        # Add followArmIK to rigParts dictionary bc we will need it in case of a reverse foot setup

        self.rigParts['reverseFootDriven'] = ikEndGrp

        # move pole vector ctr
        units = 7 * self.rigScale

        if 'x' in self.elbowDirection:
            x, y, z = True, False, False

        elif 'y' in self.elbowDirection:
            x, y, z = False, True, False

        elif 'z' in self.elbowDirection:
            x, y, z = False, False, True

        if '-' in self.elbowDirection:
            units = units * -1

        mc.move(units, poleVectorCtr.Off, x=x, y=y, z=z, os=1, r=1, wd=1)


        # make IK handles
        kneeIK = mc.ikHandle(n='{}_Knee_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        mc.hide(kneeIK)

        ankleIK = mc.ikHandle(n='{}_Ankle_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[2], ee=ikJoints[3])[0]
        mc.hide(ankleIK)


        # Parent ikhandle to follow End group
        mc.parent(kneeIK, ikEndGrp)
        mc.parent(ankleIK, ikEndGrp)


        # Create leg Aim group
        ikRotateAimGrp = mc.group(n='{}_ikRotate_Aim_Grp'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(ikEndGrp, ikRotateAimGrp, mo=0))
        mc.parent(ikRotateAimGrp, self.rigmodule.partsGrp)
        mc.aimConstraint(bodyAttachGrp, ikRotateAimGrp, aimVector=(1, 0, 0), upVector=(0, 0, 1),
                         worldUpType = 'object', worldUpVector=(0, 0, 1), worldUpObject = poleVectorCtr.C, mo=0)[0]
        mc.pointConstraint(armCtr.C, ikRotateAimGrp, mo = 1)

        # Create leg No Aim group
        ikRotateNoAimGrp = mc.group(n='{}_ikRotate_No_Aim_Grp'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(ikRotateAimGrp, ikRotateNoAimGrp, mo=0))
        mc.parent(ikRotateNoAimGrp, self.rigmodule.partsGrp)
        mc.pointConstraint(armCtr.C, ikRotateNoAimGrp, mo=1)
        mc.orientConstraint(armCtr.C, ikRotateNoAimGrp, mo=1)

        # Create leg rotate group above ikEndGrp
        ikRotateGrp = mc.group(n='{}_ikRotate_Grp'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(ikRotateAimGrp, ikRotateGrp, mo=0))
        mc.parent(ikRotateGrp, self.rigmodule.controlsGrp)
        mc.pointConstraint(armCtr.C, ikRotateGrp, mo=1)

        # Parent ballCtr to ikRotateGrp
        mc.parent(ballCtr.Off, ikRotateGrp)


        ikRotateGrp_oConstraint = mc.orientConstraint(ikRotateAimGrp, ikRotateNoAimGrp, ikRotateGrp, mo=1)[0]
        weights = mc.orientConstraint(ikRotateGrp_oConstraint, q=1, weightAliasList=1)
        mc.setAttr('{}.interpType'.format(ikRotateGrp_oConstraint), 2)

        # Set up leg aim attribute
        leg_aim_attr = 'Leg_Aim'
        mc.addAttr(armCtr.C, ln = leg_aim_attr, at='double', min=0, max=1, dv=0, k=1)
        mc.connectAttr('{}.{}'.format(armCtr.C, leg_aim_attr), '{}.{}'.format(ikRotateGrp_oConstraint, weights[0]))
        reverse_LegAim = mc.shadingNode('reverse', asUtility=True, n='{}_legAim_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(armCtr.C, leg_aim_attr), '{}.inputX'.format(reverse_LegAim))
        mc.connectAttr('{}.outputX'.format(reverse_LegAim), '{}.{}'.format(ikRotateGrp_oConstraint, weights[1]))
        mc.setAttr('{}.{}'.format(armCtr.C, leg_aim_attr), 1.0)



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
        #mc.parentConstraint(armCtr.C, upVector, sr='x', mo=1)
        mc.parentConstraint(armCtr.C, upVector, mo=1)
        mc.pointConstraint(ikJoints[0], poleAimLeg, mo=0)

        # The pole aim axis will depend on the orientation of the foot control.. it should be the axis pointing out,
        # not in the direction of the IK bend. But it will change if we orient the end ctr to the bone or to world
        # TO DO: set up pole axis, and aim constraint up and aim vectors in a smarter way. Its hard coded right now

        poleAimAxis = 'y'

        poleAimConstraint = mc.aimConstraint(armCtr.C, poleAimLeg, aimVector= (0, 1, 0), upVector=(1, 0, 0),
                                             worldUpType='objectRotation', worldUpVector=(1, 0, 0),
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

        if 'x' in poleAimAxis:
            spinAxisAttr = 'offsetX'
        elif 'y' in poleAimAxis:
            spinAxisAttr = 'offsetY'
        elif 'z' in poleAimAxis:
            spinAxisAttr = 'offsetZ'

        mc.connectAttr('{}.{}'.format(armCtr.C, spin_attr), '{}.{}'.format(poleAimConstraint, spinAxisAttr))

        # Create PV constraint
        mc.poleVectorConstraint(poleVectorCtr.C, kneeIK)

        # Orient constraint joint to gimbal control
        mc.orientConstraint(wristGimbalCtr.C, ikJoints[-1], mo=1)
        # parent gimbal control to ik control
        mc.parent(wristGimbalCtr.Off, armCtr.C)

        # make stretchy arm

        # Create locator to follow shoulder
        shoulderLoc = mc.group(n='{}_IKShoulderLoc'.format(self.prefix), em=1)
        mc.parent(shoulderLoc, bodyAttachGrp)
        mc.delete(mc.parentConstraint(ikJoints[0], shoulderLoc, mo=0))

        # Create group to follow hand IK control
        followFoot = mc.group(n='{}_IKFootFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, followFoot, mo=0))
        # create empty group under ik arm ctr
        followArmIK = mc.group(n='{}_IKArmMeasure'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(armCtr.C, followArmIK, mo=0))
        mc.parent(followArmIK, ikEndGrp)
        # parent constrain wrist group to group under ik end grp
        mc.parentConstraint(followArmIK, followFoot, mo=0)
        mc.parent(followFoot, self.rigmodule.partsGrp)

        # Create group to follow ankle bone position
        followAnkle =  mc.group(n='{}_IKAnkleFollow'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(ikJoints[2], followAnkle, mo=0))
        # create empty group under ik arm ctr
        followAnkleIK = mc.group(n='{}_IKAnkleMeasure'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(ikJoints[2], followAnkleIK, mo=0))
        mc.parent(followAnkleIK, ikEndGrp)
        # parent constrain ankle group to group under ik end grp
        mc.parentConstraint(followAnkleIK, followAnkle, mo=0)
        mc.parent(followAnkle, self.rigmodule.partsGrp)


        # create a distance node to get the length between the two groups
        arm_dist = mc.shadingNode('distanceBetween', asUtility=True, n='{}_arm_length'.format(self.prefix))
        # connect distance node to pv ctr world matrix and group positioned at wrist
        mc.connectAttr('{}.worldMatrix'.format(shoulderLoc), '{}.inMatrix1'.format(arm_dist))
        mc.connectAttr('{}.worldMatrix'.format(followAnkle), '{}.inMatrix2'.format(arm_dist))

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
            scaleAxisAttr = 'sx'
        elif 'y' in self.forwardAxis:
            lengthAxisAttr = 'ty'
            scaleAxisAttr = 'sy'
        elif 'z' in self.forwardAxis:
            lengthAxisAttr = 'tz'
            scaleAxisAttr = 'sz'

        upperBone_length = mc.getAttr('{}.{}'.format(ikJoints[1], lengthAxisAttr))
        middleBone_length = mc.getAttr('{}.{}'.format(ikJoints[2], lengthAxisAttr))
        lowerBone_length = mc.getAttr('{}.{}'.format(ikJoints[3], lengthAxisAttr))

        # Calculate the length of fully extended arm
        limbLength = upperBone_length + middleBone_length

        # Create blender for stretchy arm setup
        stretch_attr = 'Stretchy'
        mc.addAttr(armCtr.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)

        # Create class member so we can access later
        self.StretchyAttr = '{}.{}'.format(armCtr.C, stretch_attr)

        # Make length attributes
        length1_attr = 'Length1'
        length2_attr = 'Length2'
        length3_attr = 'Length3'

        mc.addAttr(armCtr.C, ln=length1_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(armCtr.C, ln=length2_attr, at='double', min=1, dv=1, k=1)
        mc.addAttr(armCtr.C, ln=length3_attr, at='double', min=1, dv=1, k=1)

        if '-' in self.forwardAxis:
            fAxisDirection = 0
        else:
            fAxisDirection = 1

        blenderStretchUpper = boneStretch(prefix='{}_bone1'.format(self.prefix),
                                          totalLimbLength = limbLength,
                                          lengthAttr = '{}.{}'.format(armCtr.C, length1_attr),
                                          stretchAttr = self.StretchyAttr,
                                          stretchDriver = sdkDriver,
                                          forwardAxisPositive = fAxisDirection
                                          )

        blenderStretchMiddle = boneStretch(prefix = '{}_bone2'.format(self.prefix),
                                          totalLimbLength = limbLength,
                                          lengthAttr = '{}.{}'.format(armCtr.C, length2_attr),
                                          stretchAttr = self.StretchyAttr,
                                          stretchDriver = sdkDriver,
                                          forwardAxisPositive = fAxisDirection
                                          )
        '''
        blenderStretchLower = boneStretch(prefix='{}_bone3'.format(self.prefix),
                                           totalLimbLength=limbLength,
                                           lengthAttr = '{}.{}'.format(armCtr.C, length3_attr),
                                           stretchAttr=self.StretchyAttr,
                                           stretchDriver=sdkDriver,
                                           forwardAxisPositive=fAxisDirection
                                           )'''

        # make elbow pin to pv setup
        pv_pin_attr = 'Pin'
        mc.addAttr(poleVectorCtr.C, ln=pv_pin_attr, at='double', min=0, max=1, dv=0, k=1)



        blenderPinUpper = poleVectorPin(prefix = '{}_bone1'.format(self.prefix),
                                        pinAttr = '{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl = poleVectorCtr.C,
                                        boneLocator = shoulderLoc,
                                        boneOrigLength = upperBone_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive = fAxisDirection
                                        )

        blenderPinMiddle = poleVectorPin(prefix = '{}_bone2'.format(self.prefix),
                                        pinAttr = '{}.{}'.format(poleVectorCtr.C, pv_pin_attr),
                                        pvCtrl = poleVectorCtr.C,
                                        boneLocator = followAnkle,
                                        boneOrigLength = middleBone_length,
                                        globalCtrl='{}.scaleX'.format(self.baseRig.global1Ctrl.C),
                                        forwardAxisPositive=fAxisDirection
                                        )

        # Make a pin/no pin blender for the last bone that just has a value of 1

        '''blenderPinLower = mc.shadingNode('blendTwoAttr', asUtility=True, n='{}_bone3_blenderPin'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), '{}.attributesBlender'.format(blenderPinLower))
        mc.setAttr('{}.input[1]'.format(blenderPinLower), 1.0)'''


        # connect stretch/no stretch blender output to pin/noPin blender input
        mc.connectAttr('{}.output'.format(blenderStretchUpper), '{}.input[0]'.format(blenderPinUpper))
        mc.connectAttr('{}.output'.format(blenderStretchMiddle), '{}.input[0]'.format(blenderPinMiddle))
        #mc.connectAttr('{}.output'.format(blenderStretchLower), '{}.input[0]'.format(blenderPinLower))

        # connect blender outputs to ik joints' scale
        mc.connectAttr('{}.output'.format(blenderPinUpper), '{}.{}'.format(ikJoints[0], scaleAxisAttr))
        mc.connectAttr('{}.output'.format(blenderPinMiddle), '{}.{}'.format(ikJoints[1], scaleAxisAttr))
        #mc.connectAttr('{}.output'.format(blenderPinLower), '{}.{}'.format(ikJoints[2], scaleAxisAttr))



        mc.setAttr('{}.{}'.format(armCtr.C, stretch_attr), 1)
        mc.setAttr('{}.{}'.format(poleVectorCtr.C, pv_pin_attr), 0)

        return {'joints': ikJoints, 'controls': controls, 'poleVecLine': poleVectorCurve,
                'bodyAttachGrp': bodyAttachGrp}





    def buildIKUnguligrade(self):
        pass

    def setInitialValues(self,
                         FKIKMode=1,
                         Stretchy=1,
                         ):

        mc.setAttr(self.FKIKAttr, FKIKMode)
        mc.setAttr(self.StretchyAttr, Stretchy)


def boneStretch( prefix, totalLimbLength, lengthAttr, stretchAttr, stretchDriver, forwardAxisPositive):
    '''

    :param prefix: prefix to name nodes created
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

    # Set blender input 0 to 1 in case we don't feed something into it later
    mc.setAttr('{}.input[0]'.format(blenderStretch), 1.0)

    # Multiply stretch amount by length multiplier
    length_stretch_mult = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_length_mult'.format(prefix))
    mc.setAttr('{}.operation'.format(length_stretch_mult), 1)
    mc.connectAttr(lengthAttr, '{}.input1X'.format(length_stretch_mult))

    # Have leg stretchy SDK feed into this multDiv node
    sdkDriven = '{}.input2X'.format(length_stretch_mult)


    # Feed result of multDiv node to blender stretch value
    mc.connectAttr('{}.outputX'.format(length_stretch_mult), '{}.input[1]'.format(blenderStretch))

    # Feed result of multDiv node to blender non-stretch value
    mc.connectAttr(lengthAttr, '{}.input[0]'.format(blenderStretch))


    # Set up SDK to stretch arm after it's extended beyond its length

    # upper leg
    mc.setDrivenKeyframe(sdkDriven,
                         currentDriver = stretchDriver,
                         driverValue = totalLimbLength,
                         value = 1.0,
                         inTangentType = 'linear',
                         outTangentType = 'linear')

    mc.setDrivenKeyframe(sdkDriven,
                         currentDriver = stretchDriver,
                         driverValue = totalLimbLength * 2,
                         value = 1.0 * 2,
                         inTangentType='spline',
                         outTangentType='spline')

    animCurve = mc.keyframe(sdkDriven, query=True, name=True)[0]

    if forwardAxisPositive:
        mc.setAttr('{}.postInfinity'.format(animCurve), 1)

    else:
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)



    return blenderStretch


def poleVectorPin( prefix, pinAttr, pvCtrl, boneLocator, boneOrigLength, globalCtrl, forwardAxisPositive):

    """

    :param prefix: prefix to name nodes created
    :param pinAttr: (str) object.attribute of Pin switch
    :param pvCtrl: (str) name of pole vector control
    :param boneLocator: (str) name of locator positioned at the bone to pin
    :param boneOrigLength: (float) value of bone length before stretching
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


    # divide new limb length by original limb length
    boneLengthScale = mc.shadingNode('multiplyDivide', asUtility=True,
                                      n='{}_boneLengthScale'.format(prefix))
    mc.setAttr('{}.operation'.format(boneLengthScale), 2)

    if not forwardAxisPositive:
        boneOrigLength = boneOrigLength * (-1)

    mc.setAttr('{}.input2X'.format(boneLengthScale), boneOrigLength)

    # feed new limb length value to boneLengthScale node
    mc.connectAttr('{}.outputX'.format(boneGlobalLength), '{}.input1X'.format(boneLengthScale))
    # connect distance from shoulder to pv ctr to blender input
    mc.connectAttr('{}.outputX'.format(boneLengthScale), '{}.input[1]'.format(blenderPin))

    return blenderPin