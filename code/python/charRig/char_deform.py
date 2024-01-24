"""
character rig setup
deformation module
"""
import maya.cmds as mc
import maya.mel as mm
import os
from . import project
from rigTools import deformerWeightsPlus

#skinWeightsDir = 'build/weights/skinClusters'
swExt = '.xml'

def build( baseRig, characterName ):

    modelGrp = project.modelGrp % characterName

    # load skin weights
    geoList = _getModelGeoObjects(modelGrp)
    loadSkinWeights(characterName, geoList)

    # apply mush deformers, wrappers, etc.
    # make twist joints if you want

def buildDeltaMush(baseRig, characterName, geoList):

    _applyDeltaMush(geoList)



def _makeWrap(wrappedObjs, wrapperObj):

    mc.select(wrappedObjs)
    mc.select(wrapperObj, add = 1)
    mm.eval('doWrapArgList "7" {"1", "0", "1", "2", "1", "1", "0", "0" }')

def _applyDeltaMush(geoList):

    for geo in geoList:
        name = geo + '_deltaMush'
        deltaMushDf = mc.deltaMush( geo, smoothingIterations = 20, n = name)[0]

def _getModelGeoObjects(modelGrp):

    geoList = [ mc.listRelatives(o, p=1)[0] for o in mc.listRelatives(modelGrp, ad = 1, type = 'mesh') ]
    return geoList

def saveSkinWeights(characterName, geoList = []):

    """
    save weights for character geometry objects
    """

    for obj in geoList:
        # weights files
        wtFile = os.path.join(project.mainProjectPath, characterName, project.skinWeightsDir, obj + swExt)

        # save skin weight file
        sdw = deformerWeightsPlus.SkinDeformerWeights()
        sdw.saveWeightInfo(fpath = wtFile, meshes = [obj])

def loadSkinWeights( characterName, geoList = []):
    """
    load skin weights for character geometry objects
    """

    # weight folders
    wtDir = os.path.join(project.mainProjectPath, characterName, project.skinWeightsDir)
    wtFiles = os.listdir(wtDir)

    print('wtDir is: ' + wtDir)
    print(wtFiles)

    # load skin weights

    for wtFile in wtFiles:

        print(wtFile)
        extRes = os.path.splitext(wtFile)
        print(extRes)

        #check extension format
        if not extRes:

            continue

        # check skin weight file
        if not extRes[1] == swExt:
                continue

        # check geometry list
        if geoList and not extRes[0] in geoList:

            continue

        # check if object exists
        if not mc.objExists(extRes[0]):

            continue

        fullpathWtFile = os.path.join(wtDir, wtFile)
        sdw = deformerWeightsPlus.SkinDeformerWeights(path = fullpathWtFile)
        sdw.applyWeightInfo()

