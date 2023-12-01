"""
leg FK/IK @ rig
"""

import maya.cmds as mc

from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name

class Leg():

    def __init__(self,
            legJoints,
            toeJoint,
            hipPivotJoint,
            pvLocator,
            prefix = 'leg',
            side = 'l',
            rigScale = 1.0,
            baseRig = None
            ):
        """
        :param legJoints: list(str), hip - knee - ankle
        :param toeJoint: str, toe joint
        :param hipPivotJoint: str, hip position joint
        :param pvLocator: str, reference locator for position of pole vector control
        :param prefix: str, prefix to name new objects
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """
        self.legJoints = []

        for jnt in legJoints:
            newJnt = side + '_' + jnt
            self.legJoints.append(newJnt)


        self.toeJoint = side + '_' + toeJoint
        self.hipPivotJoint = side + '_' + hipPivotJoint
        self.pvLocator = pvLocator
        self.prefix = side + '_' + prefix
        self.side = side
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix=self.prefix, baseObj=self.baseRig)

        # make attach groups
        bodyAttachGrp = mc.group(n='{}_BodyAttach_grp', em=1, p=self.rigmodule.partsGrp)


    def build(self):

        # Make FK rig
        fkRig = self.buildFK()

        # Make IK rig
        ikRig = self.buildIK()

        # Connect deformation joints to fk and ik joints

        constraints = []
        for i in range(len(self.legJoints)):
            constraint = mc.parentConstraint( fkRig['joints'][i], ikRig['joints'][i], self.legJoints[i], mo=0)[0]
            mc.setAttr('{}.interpType'.format(constraint), 2)
            constraints.append(constraint)

        # Make switch control

        switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo= self.legJoints[1],
                                 scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='sphere')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(switchCtr.C, ln = switch_attr, at= 'double', min = 0, max = 1, dv = 0, k = 1)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.inputX'.format(reverse) )

        for constraint in constraints:
            weights = mc.parentConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.{}'.format( constraint, weights[1] ) )
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format( ctrl.Off ) )
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ikRig['poleVecLine']))

        # organize
        constraintGrp = mc.group(constraints, n='defSkeleton_{}_constraints'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

        # move switch ctr
        if self.side == 'l':
            mc.move(5, switchCtr.Off, x=True, os=1)
        elif self.side == 'r':
            mc.move(-5, switchCtr.Off, x=True, os=1)











    def buildFK(self):

        # duplicate leg joints to make FK joints
        fkJoints = joint.duplicateChain(self.legJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)

        # make controls

        controls = []
        hipCtr = control.Control(prefix = '{}_hip'.format(self.prefix), translateTo = fkJoints[0], rotateTo = fkJoints[0],
                                  scale = self.rigScale * 1.5, parent = self.rigmodule.controlsGrp, shape = 'circleX')
        kneeCtr = control.Control(prefix = '{}_knee'.format(self.prefix), translateTo = fkJoints[1], rotateTo = fkJoints[1],
                                  scale = self.rigScale, parent = hipCtr.C, shape = 'circleX')
        footCtr = control.Control(prefix = '{}_foot'.format(self.prefix), translateTo = fkJoints[2], rotateTo = fkJoints[2],
                                        scale = self.rigScale, parent = kneeCtr.C, shape = 'circleY')
        controls.append( hipCtr )
        controls.append( kneeCtr )
        controls.append( footCtr )

        # connect controls

        mc.parentConstraint(hipCtr.C, fkJoints[0], mo = 0 )
        mc.parentConstraint(kneeCtr.C, fkJoints[1], mo=0)
        mc.parentConstraint(footCtr.C, fkJoints[2], mo=0)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, hipCtr.Off, mo=1)[0]
        constraintGrp = mc.group(constraint, n= '{}_fk_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)



        return {'joints' : fkJoints, 'controls' : controls }






    def buildIK(self):

        # duplicate leg joints to make IK joints
        ikJoints = joint.duplicateChain(self.legJoints, 'jnt', 'IK_jnt')
        mc.parent(ikJoints[0], self.rigmodule.jointsGrp)



        # Make controls
        controls = []
        legCtr = control.Control(prefix='{}_leg_ik'.format(self.prefix), translateTo=ikJoints[2],
                                  scale=self.rigScale, parent = self.rigmodule.controlsGrp, shape='square')
        poleVectorCtr = control.Control(prefix='{}_leg_pv'.format(self.prefix), translateTo=ikJoints[1],
                                 scale=self.rigScale * 0.5, parent = self.rigmodule.controlsGrp, shape='orb')
        controls.append(legCtr)
        controls.append(poleVectorCtr)

        # move pole vector ctr
        mc.move( 5, poleVectorCtr.Off, z = True, os = 1)

        # make IK handle
        legIK = mc.ikHandle(n='{}_leg_ikh'.format(self.prefix), sol='ikRPsolver', sj=ikJoints[0], ee=ikJoints[2])[0]
        mc.hide(legIK)
        mc.parent(legIK, self.rigmodule.noXformGrp)

        # pole vector position setup
        poleAimLeg = mc.group(n = '{}_poleAim'.format(self.prefix), em = 1)
        upVector = mc.group(n = '{}_pv_upVec'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(legCtr.C, upVector, mo=0))
        mc.parent(upVector, legCtr.C)

        mc.pointConstraint(ikJoints[0], poleAimLeg, mo = 0)
        mc.aimConstraint(legCtr.C, poleAimLeg, aimVector = (1,0,0), upVector = (0,0,1),
                         worldUpType = 'objectRotation', worldUpVector = (0, 0, 1), worldUpObject = upVector,  mo=0)

        poleOffsetFollow_noScale = mc.group(n = '{}_poleOffsetFollow_noScale'.format(self.prefix), em = 1)
        mc.pointConstraint(poleAimLeg, poleOffsetFollow_noScale , mo=0)
        mc.orientConstraint(poleAimLeg, poleOffsetFollow_noScale, mo=0)
        poleOffsetFollow = mc.group(n = '{}_poleOffsetFollow'.format(self.prefix), em = 1)
        mc.delete(mc.parentConstraint(poleVectorCtr.C, poleOffsetFollow, mo = 0))
        mc.parentConstraint( poleOffsetFollow_noScale, poleOffsetFollow, mo = 1 )

        mc.parentConstraint( poleOffsetFollow, poleVectorCtr.Off, mo=1)
        mc.parent(poleAimLeg, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow_noScale, self.rigmodule.partsGrp)
        mc.parent(poleOffsetFollow, self.rigmodule.partsGrp)


        # attach objects to controls
        mc.parentConstraint(legCtr.C, legIK)
        mc.poleVectorConstraint(poleVectorCtr.C, legIK)
        mc.orientConstraint(legCtr.C, ikJoints[2], mo = 1)

        # attach to hip pivot
        constraint = mc.pointConstraint(self.hipPivotJoint, ikJoints[0], mo = 1 )[0]
        constraintGrp = mc.group(constraint, n='{}_ik_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

        # make pole vector connection line

        pvLinePos1 = mc.xform(ikJoints[1], q=1, t=1, ws=1)
        pvLinePos2 = mc.xform(poleVectorCtr.C, q=1, t=1, ws=1)
        poleVectorCurve = mc.curve(n='{}_pv_curve'.format(self.prefix), d=1, p=[pvLinePos1, pvLinePos2])

        mc.cluster('{}.cv[0]'.format(poleVectorCurve), n='{}_pv1_cls'.format(self.prefix), wn=[ikJoints[1], ikJoints[1]],
                   bs=True)
        mc.cluster('{}.cv[1]'.format(poleVectorCurve), n='{}_pv2_cls'.format(self.prefix),
                   wn=[poleVectorCtr.C, poleVectorCtr.C], bs=True)

        mc.parent(poleVectorCurve, self.rigmodule.controlsGrp)
        mc.setAttr('{}.template'.format(poleVectorCurve), 1)
        mc.setAttr('{}.it'.format(poleVectorCurve), 0)







        return {'joints' : ikJoints, 'controls' : controls, 'poleVecLine': poleVectorCurve}





