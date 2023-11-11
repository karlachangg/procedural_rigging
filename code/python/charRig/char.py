"""
character rig setup
main module
"""

from rigLib.base import control
from rigLib.base import module

from . import project
from . import char_deform
import maya.cmds as mc

sceneScale = project.sceneScale
projectPath = project.mainProjectPath
modelFilePath = '%s/%s/model/%s_model.mb'
jointFilePath = '%s/%s/build/%s_skeleton.mb'

def build(characterName):

    """
    main function to build character rig
    :param characterName:
    :return:
    """

    # new scene
    mc.file ( new = True, f = True)

    # make base
    baseRig = module.Base(characterName = characterName, scale = sceneScale )

    # import model
    modelFile = modelFilePath % ( projectPath, characterName, characterName)
    mc.file(modelFile, i = 1)

    # import joints
    jointFile = jointFilePath % ( projectPath, characterName, characterName)
    mc.file(jointFile, i = 1)

    # parent model
    modelGrp = '%s_model_grp' % characterName
    mc.parent( modelGrp, baseRig.modelGrp)

    # parent skeleton joints
    skeletonGrp = 'skeleton_grp'
    mc.parent(skeletonGrp, baseRig.jointsGrp)

    # deform setup
    char_deform.build(baseRig, characterName)
