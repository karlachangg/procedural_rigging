"""
character rig setup
main module
"""

from rigLib.base import control
from rigLib.base import module

from . import project
from . import char_deform
import maya.cmds as mc
import os.path

class Character:

    def __init__(self, characterName, sceneScale = project.sceneScale):

        self.characterName = characterName
        self.sceneScale = sceneScale
        self.projectPath = project.mainProjectPath
        self.modelFilePath = '%s/%s/' + project.modelDir + '/%s_model.mb'
        self.jointFilePath = '%s/%s/' + project.rigBuildDir +'/%s_skeleton.mb'
        self.wiresFilePath = '%s/%s/' + project.rigBuildDir +'/%s_wires.mb'
        self.baseRig = module.Base(characterName = self.characterName, scale = self.sceneScale )
        self.skeletonGrp = 'skeleton_grp'

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
        if os.path.isfile(wiresFile):
            mc.file(wiresFile, i=1)

        # parent model
        modelGrp = project.modelGrp % self.characterName
        mc.parent( modelGrp, self.baseRig.modelGrp)


        # parent skeleton joints
        mc.parent(self.skeletonGrp, self.baseRig.jointsGrp)

    def deform(self):

        # deform setup
        char_deform.build(self.baseRig, self.characterName)

    def hideDeformationSkeleton(self):
        mc.hide(self.skeletonGrp)

    def preBuildCleanup(self):

        # Perform cleanup tasks before building

        # Set all deformation joints segment scale compensate to 0
        joints = mc.ls(type='joint')
        for joint in joints:
            mc.setAttr('{}.segmentScaleCompensate'.format(joint), 1)


