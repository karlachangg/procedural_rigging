"""
character rig setup
main module
"""

from rigLib.base import control
from rigLib.base import module

from . import project
from . import char_deform
import maya.cmds as mc


class Character:

    def __init__(self, characterName):

        self.characterName = characterName
        self.sceneScale = project.sceneScale
        self.projectPath = project.mainProjectPath
        self.modelFilePath = '%s/%s/model/%s_model.mb'
        self.jointFilePath = '%s/%s/build/%s_skeleton.mb'
        self.wiresFilePath = '%s/%s/build/%s_wires.mb'
        self.baseRig = module.Base(characterName = self.characterName, scale = self.sceneScale )

    def setup(self):

        """
        main function to build character rig
        :param characterName:
        :return:
        """

        # new scene
        mc.file ( new = True, f = True)

        # make base
        self.baseRig.build()

        # import model
        modelFile = self.modelFilePath % ( self.projectPath, self.characterName, self.characterName)
        mc.file(modelFile, i = 1)

        # import joints
        jointFile = self.jointFilePath % ( self.projectPath, self.characterName, self.characterName)
        mc.file(jointFile, i = 1)

        # import wires
        wiresFile = self.wiresFilePath % (self.projectPath, self.characterName, self.characterName)
        mc.file(wiresFile, i=1)

        # parent model
        modelGrp = '%s_model_grp' % self.characterName
        mc.parent( modelGrp, self.baseRig.modelGrp)

        # parent skeleton joints
        skeletonGrp = 'skeleton_grp'
        mc.parent(skeletonGrp, self.baseRig.jointsGrp)

    def deform(self):

        # deform setup
        char_deform.build(self.baseRig, self.characterName)
