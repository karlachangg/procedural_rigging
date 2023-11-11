"""
biped rig setup
main module
"""

from rigLib.base import control
from rigLib.base import module

from . import project
from . import char_deform



from rigLib.rig import spine
import maya.cmds as mc

sceneScale = project.sceneScale
projectPath = project.mainProjectPath
modelFilePath = '%s/%s/model/%s_model.mb'
jointFilePath = '%s/%s/build/%s_skeleton.mb'
wiresFilePath = '%s/%s/build/%s_wires.mb'

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

    # import wires
    wiresFile = wiresFilePath % (projectPath, characterName, characterName)
    mc.file(wiresFile, i=1)

    # parent model
    modelGrp = '%s_model_grp' % characterName
    mc.parent( modelGrp, baseRig.modelGrp)

    # parent skeleton joints
    skeletonGrp = 'skeleton_grp'
    mc.parent(skeletonGrp, baseRig.jointsGrp)



    # deform setup
    char_deform.build(baseRig, characterName)

    # control setup
    makeControlSetup(baseRig)


'''
def build():

    characterName = 'biped'
    biped = char.build(characterName)

    
    '''


def makeControlSetup(baseRig):

    """
    make control setup
    """

    # spine
    spineJoints = ['spine_0_jnt', 'spine_1_jnt', 'spine_2_jnt', 'spine_3_jnt', 'spine_4_jnt', 'spine_5_jnt', 'spine_6_jnt']
    rootJoint = 'root_jnt'
    chestJoint = 'chest_jnt'

    spineRig = spine.build(
                    spineJoints = spineJoints,
                    rootJnt = rootJoint,
                    chestJnt = chestJoint,
                    spineCurve = 'spine_curve',
                    prefix = 'spine',
                    rigScale = sceneScale,
                    baseRig = baseRig
                    )