"""
module for making top rig structure and rig module
"""
import maya.cmds as mc
from . import control

sceneObjectType = 'rig'

class Base():
    """
    class for building top rig structure
    """

    def __init__(self, characterName = 'new', scale = 1.0):
        """
        @param characterName: str, character name
        @param scale: float, general scale of the rig
        @return: None
        """
        self.characterName = characterName
        self.scale = scale



    def build(self):

        self.topGrp = mc.group(n=self.characterName, em=1)
        self.rigGrp = mc.group(n='rig_grp', em=1, p=self.topGrp)
        self.modelGrp = mc.group(n='model_grp', em=1, p=self.topGrp)

        characterNameAttr = 'characterName'
        sceneObjectTypeAttr = 'sceneObjectType'

        for at in [characterNameAttr, sceneObjectTypeAttr]:
            mc.addAttr(self.topGrp, ln=at, dt='string')

        mc.setAttr(self.topGrp + '.' + characterNameAttr, self.characterName, type='string', l=1)
        mc.setAttr(self.topGrp + '.' + sceneObjectTypeAttr, sceneObjectType, type='string', l=1)

        # make global TRS control
        self.global1Ctrl = control.Control(
            prefix = 'global',
            scale = self.scale * 10,
            parent = self.rigGrp,
            lockChannels = ['v'],
            offsets = ['master', 'shot'],
            shape = 'circleY',
            color = 'black'
        )

        for axis in ['y', 'z']:
            mc.connectAttr(self.global1Ctrl.C + '.sx', self.global1Ctrl.C + '.s' + axis)
            mc.setAttr(self.global1Ctrl.C + '.s' + axis, k=0)

        #self._rotateGlobalCtrlShape(self.global1Ctrl.C)

        # make more groups

        self.jointsGrp = mc.group(n='joints_grp', em=1, p=self.global1Ctrl.C)
        self.modulesGrp = mc.group(n='modules_grp', em=1, p=self.global1Ctrl.C)
        self.noXformGrp = mc.group(n='noXform_grp', em=1, p=self.rigGrp)
        mc.setAttr(self.noXformGrp + '.it', 0, l=1)

        # make rig control with visibility attributes
        self.mainCtrl = control.Control(
            prefix='main',
            scale=self.scale * .3,
            parent=self.global1Ctrl.C,
            lockChannels=['v', 't', 'r', 's'],
            offsets= None,
            shape = 'orb',
            color = 'black'
        )
        mainVisAttrs = ['modelVis', 'jointsVis']
        mainDispAttrs = ['modelDisp', 'jointsDisp']
        mainObjList = [self.modelGrp, self.jointsGrp]

        # Add rig visiblilty connections

        for attr, obj in zip(mainVisAttrs, mainObjList):
            mc.addAttr(self.mainCtrl.C, ln=attr, at='enum', enumName='off:on', k=1)
            mc.setAttr('{}.{}'.format(self.mainCtrl.C, attr), cb=1)
            mc.setAttr('{}.{}'.format(self.mainCtrl.C, attr), 1)
            mc.connectAttr('{}.{}'.format(self.mainCtrl.C, attr), '{}.v'.format(obj))

        # Add rig visiblilty connections

        for attr, obj in zip(mainDispAttrs, mainObjList):
            mc.addAttr(self.mainCtrl.C, ln=attr, at='enum', enumName='normal:template:reference', k=1)
            mc.setAttr('{}.{}'.format(self.mainCtrl.C, attr), cb=1)
            mc.setAttr('{}.ove'.format(obj), 1)
            mc.connectAttr('{}.{}'.format(self.mainCtrl.C, attr), '{}.ovdt'.format(obj))



    def _rotateGlobalCtrlShape(self, ctrlObject):

        # flatten ctrl object shape

        ctrlShapes = mc.listRelatives(ctrlObject, s=1, type='nurbsCurve')
        clusters = mc.cluster(ctrlShapes)[1]
        mc.setAttr(clusters + '.rz', 90)
        mc.delete(ctrlShapes, ch=1)

    def _adjustMainCtrShape(self, ctr, scale):

        ctrlShapes = mc.listRelatives(ctr.C, s=1, type='nurbsCurve')
        cls = mc.cluster(ctrlShapes)[1]
        mc.setAttr(cls + '.ry', 90)
        mc.delete(ctrlShapes, ch=1)
        mc.move(5* scale, ctr.Off, moveY = True, relative = True)

class Module():
    """
	class for building module rig structure
	"""

    def __init__(
				self,
				prefix = 'new',
				baseObj = None
				):

        """
		@param prefix: str, prefix to name new objects
		@param baseObj: instance of base.module.Base class
		@return: None
		"""
        self.topGrp = mc.group( n = '{}_rig_grp'.format(prefix), em = 1 )
        self.controlsGrp = mc.group( n = '{}_controls_grp'.format(prefix), em = 1, p = self.topGrp)
        self.partsGrp = mc.group( n = '{}_parts_grp'.format(prefix), em = 1, p = self.topGrp)
        self.jointsGrp = mc.group( n = '{}_joints_grp'.format(prefix), em = 1, p = self.partsGrp)
        self.noXformGrp = mc.group( n = '{}_noXform_grp'.format(prefix), em = 1, p = self.topGrp)

        mc.hide( self.partsGrp, self.noXformGrp )
        mc.setAttr( self.noXformGrp + '.it', 0, l = 1)

        # parent module
        if baseObj:
            mc.parent ( self.topGrp, baseObj.modulesGrp)
