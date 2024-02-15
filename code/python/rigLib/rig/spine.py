"""
spine @ rig
"""
import maya.cmds as mc

from ..base import module
from ..base import control
from ..utils import joint

from . import fkChain

class Spine():

    def __init__(self,
                 type,
                 spineJoints,
                 rootJnt,
                 chestJnt,
                 spineCurve,
                 prefix = 'spine',
                 forwardAxis = 'x',
                 upAxis = 'y',
                 rigScale = 1.0,
                 baseRig = None):

        """
        :param type: str,  IK/FK or hybrid spine
        :param spineJoints: list(str) list of 7 spine joints
        :param rootJnt: str, root joint
        :param spineCurve: str, name of spine cubic curve with 7 cvs matching 7 spine joints
        :param prefix: str, prefix to name new objects
        :param forwardAxis: str, axis pointing down the joint chain. Default 'x'
        :param upAxis: str, axis defining object up direction to be used in IK spline twist setup. Default 'y'
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
       """
        self.type = type
        self.spineJoints = spineJoints
        self.rootJnt = rootJnt
        self.chestJnt = chestJnt
        self.spineCurve = spineCurve
        self.prefix = prefix
        self.forwardAxis = forwardAxis
        self.upAxis = upAxis
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        # rigParts dictionary

        self.rigParts = {
            'module': self.rigmodule,
            'chestAttachGrp': ''
        }


    def build(self):

        if self.type == 'ikfk':
            self.buildIKFKSpine()
        elif self.type == 'hybrid':
            self.buildHybridSpine()

    def buildIKFKSpine(self):

        defSpineJoints = [self.rootJnt]

        for jnt in self.spineJoints:
            defSpineJoints.append(jnt)
        defSpineJoints.append(self.chestJnt)

        # Make FK rig
        fkRig = self.buildFK()

        # Make IK rig
        ikRig = self.buildIK()

        # make body control
        bodyCtrl = control.Control(prefix = '{}_body'.format(self.prefix), translateTo = self.rootJnt,
                                   scale = self.rigScale * 3, shape = 'circleY', parent = self.rigmodule.controlsGrp)

        # ik and fk controls follow bodyCTR
        for ctr in ikRig['controls']:
            mc.parent(ctr.Off, bodyCtrl.C)

        mc.parent(fkRig['controls'][0].Off, bodyCtrl.C)

        # fk root joint follows body control
        mc.parentConstraint(bodyCtrl.C, fkRig['root'], mo = 1)


        # Connect deformation joints to fk and ik joints

        constraints = []
        for i in range(len(defSpineJoints)):
            constraints.append( mc.parentConstraint(fkRig['joints'][i], ikRig['joints'][i], defSpineJoints[i], mo = 0) [0])
            # mc.connectAttr('{}.scale'.format(ikJnt), '{}.scale'.format(defJnt))

        # Make switch control

        switchCtr = control.Control(prefix='{}_FKIK'.format(self.prefix), translateTo=self.spineJoints[0],
                                    scale=self.rigScale * 0.5, parent=self.rigmodule.controlsGrp, shape='sphere')
        switch_attr = 'FKIK_Switch'
        mc.addAttr(switchCtr.C, ln=switch_attr, at='double', min=0, max=1, dv=0, k=1)

        # make reverse node
        reverse = mc.shadingNode('reverse', asUtility=True, n='{}_leg_switch_reverse'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.inputX'.format(reverse))

        for constraint in constraints:
            weights = mc.parentConstraint(constraint, q=1, weightAliasList=1)
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.{}'.format(constraint, weights[1]))
            mc.connectAttr('{}.outputX'.format(reverse), '{}.{}'.format(constraint, weights[0]))

        # FK IK control visibility
        for ctrl in fkRig['controls']:
            mc.connectAttr('{}.outputX'.format(reverse), '{}.v'.format(ctrl.Off))
        for ctrl in ikRig['controls']:
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.v'.format(ctrl.Off))

        # Setup blend between joint scales


        for i in range(len(defSpineJoints)):

            blendNode = mc.shadingNode('blendColors', asUtility=True, n='{}_jointScale_blend{}'.format(self.prefix, 1))
            mc.connectAttr('{}.{}'.format(switchCtr.C, switch_attr), '{}.blender'.format(blendNode))

            mc.connectAttr('{}.sx'.format(ikRig['joints'][i] ), '{}.color1.color1R'.format(blendNode) )
            mc.connectAttr('{}.sy'.format(ikRig['joints'][i]), '{}.color1.color1G'.format(blendNode) )
            mc.connectAttr('{}.sz'.format(ikRig['joints'][i]), '{}.color1.color1B'.format(blendNode) )

            mc.connectAttr('{}.sx'.format(fkRig['joints'][i]), '{}.color2.color2R'.format(blendNode) )
            mc.connectAttr('{}.sy'.format(fkRig['joints'][i]), '{}.color2.color2G'.format(blendNode) )
            mc.connectAttr('{}.sz'.format(fkRig['joints'][i]), '{}.color2.color2B'.format(blendNode) )

            mc.connectAttr('{}.outputR'.format(blendNode), '{}.sx'.format(defSpineJoints[i]) )
            mc.connectAttr('{}.outputG'.format(blendNode), '{}.sy'.format(defSpineJoints[i]) )
            mc.connectAttr('{}.outputB'.format(blendNode), '{}.sz'.format(defSpineJoints[i]) )






        # organize
        constraintGrp = mc.group(constraints, n='defSkeleton_{}_constraints'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

        # move switch ctr
        mc.move(3, switchCtr.Off, x=True, os=1)

        # make attach groups
        chestAttachGrp = mc.group(n='{}_chestAttach_grp'.format(self.prefix), em=True)
        mc.parentConstraint(self.chestJnt, chestAttachGrp, mo=0)
        mc.parent(chestAttachGrp, self.rigmodule.partsGrp)

        # rigParts dictionary
        self.rigParts['chestAttachGrp'] = chestAttachGrp



    def buildHybridSpine(self):

        # Create a list of the whole spine joint chain including root and chest

        spineDeformJoints = [self.rootJnt]

        for jnt in self.spineJoints:
            spineDeformJoints.append(jnt)

        spineDeformJoints.append(self.chestJnt)

        # Make IK rig

        ikRig = self.buildIK()

        # Connect deformation joints to ik joints

        constraints = []

        for i in range(len(spineDeformJoints)):
            constraints.append(mc.parentConstraint(ikRig['joints'][i], spineDeformJoints[i], mo=0)[0])
            mc.connectAttr('{}.scale'.format(ikRig['joints'][i]), '{}.scale'.format(spineDeformJoints[i]))

        # make body control
        bodyCtrl = control.Control(prefix='{}_body'.format(self.prefix), translateTo=self.rootJnt, rotateTo =self.rootJnt,
                                   scale=self.rigScale * 3, shape='circleX', parent=self.rigmodule.controlsGrp)

        # Make FK Controls which will drive IK controls
        fkBackCtr = control.Control(prefix='{}_backFK'.format(self.prefix), translateTo = self.rootJnt, rotateTo =self.rootJnt,
                                     scale=self.rigScale * 2, shape='circleX', parent= bodyCtrl.C)

        fkFrontCtr = control.Control(prefix='{}_frontFK'.format(self.prefix), translateTo=self.rootJnt, rotateTo =self.rootJnt,
                                    scale=self.rigScale * 2, shape='circleX', parent=bodyCtrl.C)

        fkTorsoCtr = control.Control(prefix='{}_torso'.format(self.prefix), translateTo=self.spineJoints[4],
                                     rotateTo=self.spineJoints[4],
                                     scale=self.rigScale * 2, shape='circleX', parent=fkFrontCtr.C)

        mc.parent(ikRig['controls'][0].Off, fkBackCtr.C)
        mc.parent(ikRig['controls'][2].Off, fkTorsoCtr.C)
        mc.parent(ikRig['controls'][1].Off, bodyCtrl.C)

        # make attach groups
        chestAttachGrp = mc.group(n='{}_chestAttach_grp'.format(self.prefix), em=True)
        mc.parentConstraint(self.chestJnt, chestAttachGrp, mo=0)
        mc.parent(chestAttachGrp, self.rigmodule.partsGrp)

        hipsAttachGrp = mc.group(n='{}_hipsAttach_grp'.format(self.prefix), em=True)
        mc.parentConstraint(self.spineJoints[0], hipsAttachGrp, mo=0)
        mc.parent(hipsAttachGrp, self.rigmodule.partsGrp)

        # organize
        constraintGrp = mc.group(constraints, n='defSkeleton_{}_constraints'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

        # rigParts dictionary
        self.rigParts = {
            'module': self.rigmodule,
            'chestAttachGrp': chestAttachGrp,
            'hipsAttachGrp': hipsAttachGrp
        }



    def buildFK(self):

        '''
        Create a simple FK rig. Duplicates deformation joints, creates simple FK rig on duplicate, and returns
        library of objects
        '''

        # duplicate spine joints to make FK spine joints
        fkSpineJoints = joint.duplicateChain(self.spineJoints, oldSuffix='jnt', newSuffix='FK_jnt')

        # Duplicate root joint
        fk_root_name = self.rootJnt.replace('jnt', 'FK_jnt')

        fk_root_jnt = mc.duplicate(self.rootJnt, n = fk_root_name, parentOnly = True)[0]

        # Duplicate chest joint
        fk_chest_name = self.chestJnt.replace('jnt', 'FK_jnt')
        fk_chest_jnt = mc.duplicate(self.chestJnt, n = fk_chest_name, parentOnly = True)[0]

        mc.parent(fkSpineJoints[0], fk_root_jnt)
        mc.parent(fk_root_jnt, self.rigmodule.jointsGrp)
        mc.parent(fk_chest_jnt, fkSpineJoints[-1])

        # Make ikJoints list to return
        fkJoints = [fk_root_jnt]
        for jnt in fkSpineJoints:
            fkJoints.append(jnt)
        fkJoints.append(fk_chest_jnt)

        fkControlChain = fkChain.build( joints = fkSpineJoints, rigScale = self.rigScale * 2,
                                        shape = 'circleX', lockChannels = ['t'])
        controls = fkControlChain['controls']

        return {'joints': fkJoints, 'controls': controls, 'root': fk_root_jnt, 'chest': fk_chest_jnt }


    def buildIK(self ):

        '''
        Create an IK rig. Duplicates deformation joints, creates an IK spline rig, and returns
        library of objects
        '''

        # duplicate spine joints to make IK spine joints
        ikSpineJoints = joint.duplicateChain(self.spineJoints, oldSuffix= 'jnt', newSuffix='IK_jnt')

        # Duplicate root joint
        ik_root_name = self.rootJnt.replace('jnt', 'IK_jnt')
        ik_root_jnt = mc.duplicate(self.rootJnt, n = ik_root_name, parentOnly = True)[0]

        # Duplicate chest joint
        ik_chest_name = self.chestJnt.replace('jnt', 'IK_jnt')
        ik_chest_jnt = mc.duplicate(self.chestJnt, n = ik_chest_name, parentOnly = True)[0]


        mc.parent(ikSpineJoints[0], ik_root_jnt)
        mc.parent(ik_root_jnt, self.rigmodule.jointsGrp)
        mc.parent(ik_chest_jnt, ikSpineJoints[-1])

        # Make ikJoints list to return
        ikJoints = [ik_root_jnt]
        for jnt in ikSpineJoints:
            ikJoints.append(jnt)
        ikJoints.append(ik_chest_jnt)


        # make controls

        hipsCtrlIK = control.Control(prefix ='{}_hips_IK'.format(self.prefix), translateTo = ikSpineJoints[0],
                                     rotateTo = ikSpineJoints[0],
                                     scale = self.rigScale * 2, shape='squareY', parent = self.rigmodule.controlsGrp )

        middleCtrlIK = control.Control(prefix = '{}_mid_IK'.format(self.prefix), scale = self.rigScale * 1.5,
                                       rotateTo=ikSpineJoints[3],
                                       shape='squareY', parent = self.rigmodule.controlsGrp )

        chestCtrlIK = control.Control(prefix = '{}_chest_IK'.format(self.prefix), translateTo = ikSpineJoints[-1],
                                      rotateTo = ikSpineJoints[-1],
                                      scale = self.rigScale * 1.5, shape = 'squareY', parent = self.rigmodule.controlsGrp)

        controls = [hipsCtrlIK, middleCtrlIK, chestCtrlIK]


        # attach middle control
        mc.delete(mc.pointConstraint(chestCtrlIK.C, hipsCtrlIK.C, middleCtrlIK.Off, mo = 0))

        # Create empty groups to drive middle control
        followChest = mc.group(em=1, n='{}_chestFollow'.format(self.prefix))
        followHips = mc.group(em=1, n='{}_hipsFollow'.format(self.prefix))
        mc.delete(mc.parentConstraint(middleCtrlIK.C, followChest))
        mc.delete(mc.parentConstraint(middleCtrlIK.C, followHips))
        mc.parent(followChest, chestCtrlIK.C)
        mc.parent(followHips, hipsCtrlIK.C)

        mc.pointConstraint(followChest, followHips, middleCtrlIK.Off, mo=1)

        # make locators to drive spine curve clusters

        spineCurveCVs = mc.ls('{}.cv[*]'.format(self.spineCurve), fl=1)
        numSpineCVs = len(spineCurveCVs)
        spineCurveShape = mc.listRelatives(self.spineCurve, ad=True, shapes=True)[0]
        driverLocators = []
        driverLocatorOffsets = []
        locGrp = mc.group(n='{}_cv_drivers'.format(self.prefix), em=True)

        for i in range(numSpineCVs):
            cvXform = mc.xform(spineCurveCVs[i], q=True, t=True)
            loc = mc.spaceLocator(n='{}_cv{}_loc'.format(self.prefix, i + 1))[0]
            driverLocators.append(loc)
            # get locator shape node
            locShape = mc.listRelatives(loc, ad=1, shapes=True)[0]
            offset = mc.group(n='{}_cv{}_loc_offset'.format(self.prefix, i + 1))
            driverLocatorOffsets.append(offset)
            mc.parent(offset, locGrp)
            mc.xform(offset, t=cvXform)
            mc.connectAttr('{}.worldPosition[0]'.format(locShape), spineCurveShape + '.controlPoints[%d]' % (i))

        mc.parent(locGrp, self.rigmodule.partsGrp)
        mc.select(cl=1)

        # make locators to follow 3 IK controls

        spineLocators = ['{}_start_loc'.format(self.prefix), '{}_mid_loc'.format(self.prefix), '{}_end_loc'.format(self.prefix)]
        spineLocatorOffsets = []
        spineLocatorsGrp = mc.group(n='{}_position_drivers'.format(self.prefix), em=True)
        for i in range(3):
            mc.spaceLocator(n=spineLocators[i])
            offset = mc.group(n=spineLocators[i] + '_offset')
            spineLocatorOffsets.append(offset)
            mc.parent(offset, spineLocatorsGrp)

        mc.parent(spineLocatorsGrp, self.rigmodule.partsGrp)
        mc.parentConstraint(hipsCtrlIK.C, spineLocatorOffsets[0], mo=False)
        mc.parentConstraint(middleCtrlIK.C, spineLocatorOffsets[1], mo=False)
        mc.parentConstraint(chestCtrlIK.C, spineLocatorOffsets[2], mo=False)

        # connect spine CV drivers to locators at hips, chest, and middle
        # cv 0
        mc.parentConstraint(spineLocators[0], driverLocatorOffsets[0], mo=True)
        # cv 3
        mc.parentConstraint(spineLocators[1], driverLocatorOffsets[3], mo=True)
        # cv 6
        mc.parentConstraint(spineLocators[2], driverLocatorOffsets[6], mo=True)

        # cv 1
        mc.parentConstraint(spineLocators[0], spineLocators[1], driverLocatorOffsets[1], mo=True)
        mc.parentConstraint(spineLocators[0], driverLocatorOffsets[1], e=True, w=0.66)
        mc.parentConstraint(spineLocators[1], driverLocatorOffsets[1], e=True, w=0.34)
        # cv 2
        mc.parentConstraint(spineLocators[0], spineLocators[1], driverLocatorOffsets[2], mo=True)
        mc.parentConstraint(spineLocators[0], driverLocatorOffsets[2], e=True, w=0.34)
        mc.parentConstraint(spineLocators[1], driverLocatorOffsets[2], e=True, w=0.66)
        # cv 4
        mc.parentConstraint(spineLocators[1], spineLocators[2], driverLocatorOffsets[4], mo=True)
        mc.parentConstraint(spineLocators[1], driverLocatorOffsets[4], e=True, w=0.66)
        mc.parentConstraint(spineLocators[2], driverLocatorOffsets[4], e=True, w=0.34)
        # cv 5
        mc.parentConstraint(spineLocators[1], spineLocators[2], driverLocatorOffsets[5], mo=True)
        mc.parentConstraint(spineLocators[1], driverLocatorOffsets[5], e=True, w=0.34)
        mc.parentConstraint(spineLocators[2], driverLocatorOffsets[5], e=True, w=0.66)

        # make IK handle

        spineIk = mc.ikHandle(n = '{}_ikHandle'.format(self.prefix), sol = 'ikSplineSolver', sj = ikSpineJoints[0],
                              ee = ikSpineJoints[-1], c = self.spineCurve, ccv = 0, parentCurve = 0)[0]
        mc.hide(spineIk)
        mc.parent(spineIk, self.rigmodule.noXformGrp)
        mc.parent(self.spineCurve, self.rigmodule.noXformGrp)

        # set up IK twist

        # Forward Axis setting

        if self.forwardAxis == 'x':
            fAxisAttr = 0
            stretchAxis, squashAxis1, squashAxis2 = 'scaleX', 'scaleY', 'scaleZ'

        elif self.forwardAxis == 'y':
            fAxisAttr = 2
            stretchAxis, squashAxis1, squashAxis2 = 'scaleY', 'scaleX', 'scaleZ'

        elif self.forwardAxis == 'z':
            fAxisAttr = 4
            stretchAxis, squashAxis1, squashAxis2 = 'scaleZ', 'scaleY', 'scaleX'

        elif self.forwardAxis == '-x':
            fAxisAttr = 1
            stretchAxis, squashAxis1, squashAxis2 = 'scaleX', 'scaleY', 'scaleZ'

        elif self.forwardAxis == '-y':
            fAxisAttr = 3
            stretchAxis, squashAxis1, squashAxis2 = 'scaleY', 'scaleX', 'scaleZ'

        elif self.forwardAxis == '-z':
            fAxisAttr = 5
            stretchAxis, squashAxis1, squashAxis2 = 'scaleZ', 'scaleY', 'scaleX'

        # Up Axis setting

        if self.upAxis == 'x':
            upAxisAttr = 6
            x, y, z = 1, 0, 0

        elif self.upAxis == 'y':
            upAxisAttr = 0
            x, y, z = 0, 1, 0

        elif self.upAxis == 'z':
            upAxisAttr = 3
            x, y, z = 0, 0, 1

        elif self.upAxis == '-x':
            upAxisAttr = 7
            x, y, z = -1, 0, 0

        elif self.upAxis == '-y':
            upAxisAttr = 1
            x, y, z = 0, -1, 0

        elif self.upAxis == '-z':
            upAxisAttr = 4
            x, y, z = 0, 0, -1

        mc.setAttr('{}.dTwistControlEnable'.format(spineIk), 1)
        mc.setAttr('{}.dWorldUpType'.format(spineIk), 4)
        mc.setAttr('{}.dForwardAxis'.format(spineIk), fAxisAttr)
        mc.setAttr('{}.dWorldUpAxis'.format(spineIk), upAxisAttr)
        mc.setAttr('{}.dWorldUpVector'.format(spineIk), x, y, z, type="double3")
        mc.setAttr('{}.dWorldUpVectorEnd'.format(spineIk), x, y, z, type="double3")

        mc.connectAttr('{}.worldMatrix[0]'.format(spineLocators[2]), '{}.dWorldUpMatrixEnd'.format(spineIk))
        mc.connectAttr('{}.worldMatrix[0]'.format(spineLocators[0]), '{}.dWorldUpMatrix'.format(spineIk))

        # Stretchy Spine Setup

        # make curve info node

        curveInfoNode = mc.arclen(self.spineCurve, ch=True)

        # Connect spine curve to curveInfo
        spine_init_length = mc.getAttr('{}.arcLength'.format(curveInfoNode))

        # Make multiplyDivide node to scale initial amount by global scale
        spineGlobalScale = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_global_scale'.format(self.prefix))

        mc.setAttr('{}.operation'.format(spineGlobalScale), 1)
        mc.setAttr('{}.input1X'.format(spineGlobalScale), spine_init_length)
        mc.connectAttr('{}.scaleX'.format(self.baseRig.global1Ctrl.C), '{}.input2X'.format(spineGlobalScale))

        # Make multiplyDivide node to get stretch amount
        stretchRatio = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_stretch_ratio'.format(self.prefix))

        mc.setAttr('{}.operation'.format(stretchRatio), 2)
        mc.connectAttr('{}.arcLength'.format(curveInfoNode), '{}.input1X'.format(stretchRatio))
        mc.connectAttr('{}.outputX'.format(spineGlobalScale), '{}.input2X'.format(stretchRatio))

        # Make blender to turn off Stretchy setup

        stretch_attr = 'Stretchy'
        mc.addAttr(chestCtrlIK.C, ln=stretch_attr, at='double', min=0, max=1, dv=0, k=1)
        self.StretchyAttr = '{}.{}'.format(chestCtrlIK.C, stretch_attr)

        # make blender node for stretchy spine
        blenderStretch = mc.shadingNode('blendColors', asUtility=True,
                                                n='{}_blenderSpineStretch'.format(self.prefix))
        mc.connectAttr('{}.{}'.format(chestCtrlIK.C, stretch_attr), '{}.blender'.format(blenderStretch))
        mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.color1.color1R'.format(blenderStretch))
        mc.setAttr( '{}.color2.color2R'.format(blenderStretch), 1)


        # Connect output of stretchRatio with all spine joints scaleX except last
        for jnt in ikSpineJoints[:-1]:
            mc.connectAttr('{}.outputR'.format(blenderStretch), '{}.{}'.format(jnt, stretchAxis))

        # set up spine squash

        # Make multiplyDivide node to get square root of spine
        sqrRootSpineStretch = mc.shadingNode('multiplyDivide', asUtility=True,
                                             n='sqrRoot_{}_stretch'.format(self.prefix))
        mc.setAttr('{}.operation'.format(sqrRootSpineStretch), 3)
        mc.connectAttr('{}.outputX'.format(stretchRatio), '{}.input1X'.format(sqrRootSpineStretch))
        mc.setAttr('{}.input2X'.format(sqrRootSpineStretch), 0.5)

        # Make multiplyDivide node to divide 1 by square root of spine
        divideSqrRoot = mc.shadingNode('multiplyDivide', asUtility=True, n='{}_divide_sqr_root'.format(self.prefix))
        mc.setAttr('{}.operation'.format(divideSqrRoot), 2)
        mc.setAttr('{}.input1X'.format(divideSqrRoot), 1.0)
        mc.connectAttr('{}.outputX'.format(sqrRootSpineStretch), '{}.input2X'.format(divideSqrRoot))

        # Feed squash values through the blendColors node
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.color1.color1G'.format(blenderStretch))
        mc.connectAttr('{}.outputX'.format(divideSqrRoot), '{}.color1.color1B'.format(blenderStretch))
        mc.setAttr('{}.color2.color2G'.format(blenderStretch), 1)
        mc.setAttr('{}.color2.color2B'.format(blenderStretch), 1)

        # Connect output X to scale Y and Z of spine joints except first and last
        for jnt in ikSpineJoints[1:-1]:
            mc.connectAttr('{}.outputG'.format(blenderStretch), '{}.{}'.format(jnt, squashAxis1))
            mc.connectAttr('{}.outputB'.format(blenderStretch), '{}.{}'.format(jnt, squashAxis2))


        # attach chest and root joints
        chestConstraint = mc.orientConstraint(chestCtrlIK.C, ik_chest_jnt, mo=1)[0]
        rootConstraint = mc.parentConstraint(hipsCtrlIK.C, ik_root_jnt, mo=1)[0]
        constraintGrp = mc.group([rootConstraint, chestConstraint], n='{}_ik_constraintGrp'.format(self.prefix))
        mc.parent(constraintGrp, self.baseRig.noXformGrp)

        return { 'joints': ikJoints, 'controls': controls, 'root': ik_root_jnt, 'chest': ik_chest_jnt }

    def setInitialValues(self,
                         Stretchy = 1,
                         ):

        mc.setAttr(self.StretchyAttr, Stretchy)