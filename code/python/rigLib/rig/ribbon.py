'''
From Jeff Brodsky
'''

import maya.cmds as cmds

# you need a curve that is parameterized 0-1
# you also need a bunch of joints already positioned along the curve. They don't need to be perfectly on the curve.



def buildRibbon(curve, surf, rigJnts):

    percentageDict = getPositionPercentageValues(curve, rigJnts)

    # motion paths use percentage values rather than pure U or V values
#
    # then
    attachJointsToMotionPath(percentageDict, curve, rigJnts)
    ##
    # optional, since a curve doesn't provide orientation info you can use a nurbs surface to do this.. you can dupe your curve twice to loft the
    # surface... delete history and skin the surface to the same joints as your curve.
    folliclesForOrientation(surf, rigJnts)


def getPositionPercentageValues(curve, rigJnts):
    dictPercentageValues = dict()

    curveShape = cmds.listRelatives(curve, s=True)[0]

    arcLen = cmds.createNode('arcLengthDimension', n='temp_delete_arcLen')
    arcLenTrans = cmds.listRelatives(arcLen, p=True)[0]
    npoc = cmds.createNode('nearestPointOnCurve')
    cmds.connectAttr('%s.worldSpace[0]' % curveShape, '%s.nurbsGeometry' % arcLen)
    cmds.connectAttr('%s.worldSpace[0]' % curveShape, '%s.inputCurve' % npoc)

    # get total curve length
    cmds.setAttr('%s.uParamValue' % arcLen, 1.0)
    totLength = cmds.getAttr('%s.arcLength' % arcLen)

    for i in range(len(rigJnts)):
        tempTrans = cmds.createNode('transform', n='tempTransDELETE')
        cmds.delete(cmds.parentConstraint(rigJnts[i], tempTrans))
        cmds.connectAttr('%s.translate' % tempTrans, '%s.inPosition' % npoc, f=True)
        nearestParam = cmds.getAttr('%s.parameter' % npoc)
        cmds.setAttr('%s.uParamValue' % arcLen, nearestParam)
        pos = cmds.getAttr('%s.arcLength' % arcLen)
        dictPercentageValues[rigJnts[i]] = pos / totLength

        cmds.delete(tempTrans)
    cmds.delete(arcLenTrans, npoc)
    return (dictPercentageValues)


def attachJointsToMotionPath(dictPercentageValues, curve, rigJnts):
    curveShape = cmds.listRelatives(curve, s=True)[0]

    motionPaths = []
    for i in range(len(rigJnts)):
        motionPaths.append(cmds.createNode('motionPath', n='%s_motionPath' % rigJnts[i]))
        cmds.setAttr('%s.fm' % motionPaths[i], True)
        cmds.connectAttr('%s.worldSpace[0]' % curveShape, '%s.geometryPath' % motionPaths[i])

        cmds.setAttr('%s.uValue' % motionPaths[i], dictPercentageValues[rigJnts[i]])

        cmds.connectAttr('%s.allCoordinates' % motionPaths[i], '%s.translate' % rigJnts[i])


def folliclesForOrientation(surf, rigJnts):
    surfaceShape = cmds.listRelatives(surf, s=True)[0]

    follicles = []

    for i in range(len(rigJnts)):
        cpos = cmds.createNode('closestPointOnSurface', n='%s_cpos' % rigJnts[i])
        cmds.connectAttr('%s.worldSpace[0]' % surfaceShape, '%s.inputSurface' % cpos)

        cmds.connectAttr('%s.translate' % rigJnts[i], '%s.inPosition' % cpos)
        folShape = cmds.createNode('follicle', n='%s_follicle' % rigJnts[i])
        folTransTemp = cmds.listRelatives(folShape, p=True)[0]
        folTrans = cmds.rename(folTransTemp, '%sTrans' % folShape)
        cmds.setAttr('%s.visibility' % folTrans, 0)
        cmds.connectAttr('%s.local' % surfaceShape, '%s.inputSurface' % folShape)
        cmds.connectAttr('%s.worldMatrix[0]' % surfaceShape, '%s.inputWorldMatrix' % folShape)
        cmds.connectAttr('%s.outRotate' % folShape, '%s.rotate' % folTrans)
        cmds.connectAttr('%s.outTranslate' % folShape, '%s.translate' % folTrans)
        cmds.connectAttr('%s.parameterU' % cpos, '%s.parameterU' % folShape)
        cmds.connectAttr('%s.parameterV' % cpos, '%s.parameterV' % folShape)
        cmds.orientConstraint(folTrans, rigJnts[i], mo=True)
        follicles.append(folTrans)

    cmds.group(follicles)
    return (follicles)