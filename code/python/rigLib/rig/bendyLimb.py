"""
bendyLimb @ rig
"""
import maya.cmds as mc

from ..base import module
from ..base import control
from ..utils import joint


class BendyLimb():

    def __init__(self,
                 startJoint,
                 endJoint,
                 numberOfBendyJoints,
                 numberOfBendyControls,
                 aimAxis = 'x',
                 upAxis = 'z',
                 prefix = 'limb',
                 rigScale = 1.0,
                 baseRig = None):

        self.startJoint = startJoint
        self.endJoint = endJoint
        self.numberOfBendyJoints = numberOfBendyJoints
        self.numberOfBendyControls = numberOfBendyControls
        self.aimAxis = aimAxis
        self.upAxis = upAxis
        self.prefix = prefix
        self.rigScale = rigScale
        self.baseRig = baseRig
        self.bendyJoints = []
        # make rig module

        self.rigmodule = module.Module(prefix=self.prefix, baseObj=self.baseRig)

    def build(self):

        # Create locator to follow the start and end joints
        limbStartPos = mc.spaceLocator(n = '{}_limbStartPos'.format(self.prefix))[0]
        limbEndPos = mc.spaceLocator(n='{}_limbEndPos'.format(self.prefix))[0]

        startPosConstraint = mc.parentConstraint(self.startJoint, limbStartPos, mo = 0)[0]
        endPosConstraint = mc.parentConstraint(self.endJoint, limbEndPos, mo=0)[0]
        mc.setAttr('{}.interpType'.format(startPosConstraint), 2)
        mc.setAttr('{}.interpType'.format(endPosConstraint), 2)

        mc.parent(limbStartPos, self.rigmodule.partsGrp)
        mc.parent(limbEndPos, self.rigmodule.partsGrp)
        mc.hide(limbStartPos, limbEndPos)

        # Make bendy joints
        bendyLimb = joint.duplicateChain([self.startJoint, self.endJoint], newSuffix= 'bend_jnt', prefix = self.prefix)
        bendyStartJoint = bendyLimb[0]
        bendyEndJoint = bendyLimb[1]

        bendyJoints  = joint.segmentJointchain(bendyStartJoint, bendyEndJoint, numberOfSegments = 4,
                                               prefix = '{}_bend'.format(self.prefix) )

        mc.parent(bendyJoints[0], self.rigmodule.jointsGrp)


        # Make curve to drive bendy joints

        cvPositions = []

        for jnt in bendyJoints:
            pos = mc.xform(jnt, q = 1, t = 1, ws = 1 )
            cvPositions.append(pos)

        bendCurve = mc.curve( n = '{}_curve'.format(self.prefix), d = 3, p = cvPositions)
        mc.rebuildCurve(bendCurve, keepRange=0, keepControlPoints=True)
        mc.parent(bendCurve, self.rigmodule.noXformGrp)
        mc.hide(bendCurve)

        # Make bendy controls
        # I'd like to make the number of controls procedural but for now lets hardcode it at 3




        bendStartCtr = control.Control(prefix='{}_bend0'.format(self.prefix), translateTo = limbStartPos, rotateTo= limbStartPos,
                                     scale=self.rigScale, shape='diamond', parent=self.rigmodule.controlsGrp,
                                       offsets = ['null', 'zero', 'auto'], lockChannels = ['v'])

        bendMidCtr = control.Control(prefix='{}_bend1'.format(self.prefix), offsets = ['null', 'zero', 'auto'], rotateTo= limbStartPos,
                                       scale=self.rigScale, shape='diamond', parent=self.rigmodule.controlsGrp,
                                     lockChannels = ['v'])
        '''
        bendEndCtr = control.Control(prefix='{}_bend2'.format(self.prefix), translateTo= limbEndPos,
                                       rotateTo = limbEndPos, offsets = ['null', 'zero', 'auto'],
                                       scale=self.rigScale, shape='diamond', parent=self.rigmodule.controlsGrp,
                                     lockChannels = ['v'])
        '''


        bendControls = [bendStartCtr, bendMidCtr]
        startCtrConstraint = mc.parentConstraint(limbStartPos, bendStartCtr.Off, mo = 1)[0]
        mc.setAttr('{}.interpType'.format(startCtrConstraint), 2)

        #mc.pointConstraint(limbEndPos, bendEndCtr.Off, mo=1)
        # TO DO: Change this to just parent constraint?
        #endCtrConstraint = mc.orientConstraint(limbEndPos, bendEndCtr.Off, mo=1)[0]
        #mc.setAttr('{}.interpType'.format(endCtrConstraint), 2)

        mc.pointConstraint(limbStartPos, limbEndPos, bendMidCtr.Off, mo=0)
        midCtrConstraint = mc.orientConstraint(limbStartPos, bendMidCtr.Off, skip = self.aimAxis.replace('-', ''), mo=1)[0]
        mc.setAttr('{}.interpType'.format(midCtrConstraint), 2)

        if self.aimAxis == 'x' or self.aimAxis == '-x':
            skip = ['y', 'z']

        elif self.aimAxis == 'y' or self.aimAxis == '-y':
            skip = ['x', 'z']
        elif self.aimAxis == 'z' or self.aimAxis == '-z':
            skip = ['x', 'y']

        midCtrConstraint2 = mc.orientConstraint(limbStartPos, limbEndPos, bendMidCtr.Offsets[1], skip= skip, mo=0)[0]
        mc.setAttr('{}.interpType'.format(midCtrConstraint2), 2)

        # Create driving locators- these will follow the bendy controls and drive the curve locators
        driverLocators = []
        for i in range(len(bendControls)):
            loc = mc.spaceLocator(n = '{}_bend_driverLoc_{}'.format(self.prefix, i) )[0]
            mc.delete(mc.parentConstraint( bendControls[i].C, loc, mo = 0))
            mc.parent(loc, bendControls[i].C )
            mc.hide(loc)
            driverLocators.append(loc)
        driverLocators.append(limbEndPos)


        # Make locators to connect to the curve

        spineCurveCVs = mc.ls('{}.cv[*]'.format(bendCurve), fl=1)
        numSpineCVs = len(spineCurveCVs)
        spineCurveShape = mc.listRelatives(bendCurve, ad=True, shapes=True)[0]
        cvLocators = []
        cvLocatorOffsets = []
        locGrp = mc.group(n='{}_bendCurve_cv_drivers'.format(self.prefix), em=True)

        for i in range(numSpineCVs):
            cvXform = mc.xform(spineCurveCVs[i], q=True, t=True)

            loc = mc.spaceLocator(n='{}_cv{}_loc'.format(self.prefix, i + 1))[0]
            offset = mc.group(loc, n='{}_cv{}_loc_offset'.format(self.prefix, i + 1))
            cvLocators.append(loc)

            # get locator shape node
            locShape = mc.listRelatives(loc, ad=1, shapes=True)[0]

            cvLocatorOffsets.append(offset)
            mc.parent(offset, locGrp)
            mc.xform(offset, t=cvXform)
            mc.delete(mc.orientConstraint(bendyJoints[i], offset, mo = 0))
            mc.connectAttr('{}.worldPosition[0]'.format(locShape), spineCurveShape + '.controlPoints[%d]' % (i))

        mc.parent(locGrp , self.rigmodule.partsGrp )

        # Make locator at start and end to hold position but not orientation


        #startNoRotate = mc.spaceLocator(n = '{}_startLoc_noRotate'.format(self.prefix))[0]
        endNoRotate = mc.spaceLocator(n='{}_endLoc_noRotate'.format(self.prefix))[0]
        endNoRotateOffset = mc.group(endNoRotate, n = '{}_endLoc_noRotate_Offset'.format(self.prefix))

        mc.parent(endNoRotateOffset, self.rigmodule.partsGrp)

        mc.delete(mc.parentConstraint(limbEndPos, endNoRotateOffset, mo = 0))

        mc.pointConstraint(limbEndPos, endNoRotateOffset, mo=0)
        endNoRotateConstraint = mc.orientConstraint(limbStartPos, endNoRotateOffset, mo = 0)[0]
        endNoRotateConstraint2 = mc.orientConstraint(limbEndPos, endNoRotate, skip = skip, mo = 1)[0]
        mc.setAttr('{}.interpType'.format(endNoRotateConstraint), 2)
        mc.setAttr('{}.interpType'.format(endNoRotateConstraint2), 2)


        # Connect curve locators to driving locators

        # cv 0
        loc0Constraint = mc.parentConstraint(driverLocators[0], cvLocatorOffsets[0], mo=True)[0]
        mc.setAttr('{}.interpType'.format(loc0Constraint), 2)

        # cv 2
        loc2Constraint = mc.parentConstraint(driverLocators[1], cvLocatorOffsets[2], mo=True)[0]
        mc.setAttr('{}.interpType'.format(loc2Constraint), 2)
        # cv 4
        loc4Constraint= mc.parentConstraint(driverLocators[2], cvLocatorOffsets[4], mo=True)[0]
        mc.setAttr('{}.interpType'.format(loc4Constraint), 2)

        # cv 1
        loc1Constraint = mc.parentConstraint(driverLocators[0], driverLocators[1], cvLocatorOffsets[1], mo=True)[0]
        mc.parentConstraint(driverLocators[0], cvLocatorOffsets[1], e=True, w=0.5)
        mc.parentConstraint(driverLocators[1], cvLocatorOffsets[1], e=True, w=0.5)
        mc.setAttr('{}.interpType'.format(loc1Constraint), 2)

        # cv 3
        loc3Constraint = mc.parentConstraint(driverLocators[1], endNoRotate, cvLocatorOffsets[3], mo=True)[0]
        mc.parentConstraint(driverLocators[1], cvLocatorOffsets[3], e=True, w=0.5)
        mc.parentConstraint(endNoRotate, cvLocatorOffsets[3], e=True, w=0.5)
        mc.setAttr('{}.interpType'.format(loc3Constraint), 2)

        # remove last bendy joint
        #mc.delete(bendyJoints[-1])
        #bendyJoints.pop()

        # Make another set of joints to ride along curve
        ridingJoints = joint.duplicateChain(bendyJoints[:-1], oldSuffix = 'bend', newSuffix= 'followcurve' )

        for jnt in ridingJoints:
            if mc.listRelatives(jnt, p = 1):
                mc.parent(jnt, world = 1)
        ridingJntGrp = mc.group(ridingJoints, n = '{}_followCurve_Joints_Grp'.format(self.prefix) )
        mc.parent(ridingJntGrp, self.rigmodule.noXformGrp)
        mc.hide(ridingJntGrp)


        # Drive riding joints with curve
        percentageDict = getPositionPercentageValues(bendCurve, ridingJoints)
        attachJointsToMotionPath(percentageDict, bendCurve, ridingJoints)



        if self.aimAxis == 'x':
            aimVector = [1, 0, 0]

        elif self.aimAxis == 'y':
            aimVector = [0, 1, 0]

        elif self.aimAxis == 'z':
            aimVector = [0, 0, 1]

        elif self.aimAxis == '-x':
            aimVector = [-1, 0, 0]

        elif self.aimAxis == '-y':
            aimVector = [0, -1, 0]

        elif self.aimAxis == '-z':
            aimVector = [0, 0, -1]


        if self.upAxis == 'x':
            upVector = [1, 0, 0]

        elif self.upAxis == 'y':
            upVector = [0, 1, 0]

        elif self.upAxis == 'z':
            upVector = [0, 0, 1]

        elif self.upAxis == '-x':
            upVector = [-1, 0, 0]

        elif self.upAxis == '-y':
            upVector = [0, -1, 0]

        elif self.upAxis == '-z':
            upVector = [0, 0, -1]

        # List to hold constraints
        jointConstraints = []

        for i in range(len(bendyJoints)):


            if bendyJoints[i] == bendyJoints[-1]:
                lastbendyJntConstraint = mc.parentConstraint(limbEndPos, bendyJoints[i], mo = 1 )[0]
                mc.setAttr('{}.interpType'.format(lastbendyJntConstraint), 2)
                jointConstraints.append(lastbendyJntConstraint)

            else:
                pConstraint = mc.pointConstraint(ridingJoints[i], bendyJoints[i], mo=1)[0]
                jointConstraints.append(pConstraint)


                tConstraint = mc.tangentConstraint(bendCurve, bendyJoints[i], aimVector=aimVector, upVector=upVector,
                             worldUpType='objectRotation', worldUpVector= upVector, worldUpObject= cvLocators[i])[0]

                jointConstraints.append(tConstraint)

        constraintGrp = mc.group(jointConstraints, n = '{}_bendyJointConstraints'.format(self.prefix) )
        mc.parent(constraintGrp, self.rigmodule.noXformGrp)
        self.bendyJoints = bendyJoints



def createPlaneFromCurve(curve, surfaceName, direction = 'x'):


    width = 0.02

    curve1 = mc.duplicate(curve)
    curve2 = mc.duplicate(curve)

    if direction == 'x' or direction == '-x':
        mc.move(width, curve1, x=True, os=1)
        mc.move(-width, curve2, x=True, os=1)
    elif direction == 'y' or direction == '-y':
        mc.move(width, curve1, y=True, os=1)
        mc.move(-width, curve2, y=True, os=1)
    elif direction == 'z' or direction == '-z':
        mc.move(width, curve1, z=True, os=1)
        mc.move(-width, curve2, z=True, os=1)

    surface = mc.loft(curve1, curve2, n = surfaceName, ch=True, rn=True, ar=True)[0]
    mc.delete(surface, constructionHistory=1)
    mc.delete(curve1, curve2)

    return surface



def getPositionPercentageValues(curve, rigJnts):
    dictPercentageValues = dict()

    curveShape = mc.listRelatives(curve, s=True)[0]

    arcLen = mc.createNode('arcLengthDimension', n='temp_delete_arcLen')
    arcLenTrans = mc.listRelatives(arcLen, p=True)[0]
    npoc = mc.createNode('nearestPointOnCurve')
    mc.connectAttr('%s.worldSpace[0]' % curveShape, '%s.nurbsGeometry' % arcLen)
    mc.connectAttr('%s.worldSpace[0]' % curveShape, '%s.inputCurve' % npoc)

    # get total curve length
    mc.setAttr('%s.uParamValue' % arcLen, 1.0)
    totLength = mc.getAttr('%s.arcLength' % arcLen)

    for i in range(len(rigJnts)):
        tempTrans = mc.createNode('transform', n='tempTransDELETE')
        mc.delete(mc.parentConstraint(rigJnts[i], tempTrans))
        mc.connectAttr('%s.translate' % tempTrans, '%s.inPosition' % npoc, f=True)
        nearestParam = mc.getAttr('%s.parameter' % npoc)
        mc.setAttr('%s.uParamValue' % arcLen, nearestParam)
        pos = mc.getAttr('%s.arcLength' % arcLen)
        dictPercentageValues[rigJnts[i]] = pos / totLength

        mc.delete(tempTrans)
    mc.delete(arcLenTrans, npoc)
    return (dictPercentageValues)


def attachJointsToMotionPath(dictPercentageValues, curve, rigJnts):
    curveShape = mc.listRelatives(curve, s=True)[0]

    motionPaths = []
    for i in range(len(rigJnts)):
        motionPaths.append(mc.createNode('motionPath', n='%s_motionPath' % rigJnts[i]))
        mc.setAttr('%s.fm' % motionPaths[i], True)
        mc.connectAttr('%s.worldSpace[0]' % curveShape, '%s.geometryPath' % motionPaths[i])

        mc.setAttr('%s.uValue' % motionPaths[i], dictPercentageValues[rigJnts[i]])

        mc.connectAttr('%s.allCoordinates' % motionPaths[i], '%s.translate' % rigJnts[i])


def folliclesForOrientation(surf, rigJnts):
    surfaceShape = mc.listRelatives(surf, s=True)[0]

    follicles = []

    for i in range(len(rigJnts)):
        cpos = mc.createNode('closestPointOnSurface', n='%s_cpos' % rigJnts[i])
        mc.connectAttr('%s.worldSpace[0]' % surfaceShape, '%s.inputSurface' % cpos)

        mc.connectAttr('%s.translate' % rigJnts[i], '%s.inPosition' % cpos)
        folShape = mc.createNode('follicle', n='%s_follicle' % rigJnts[i])
        folTransTemp = mc.listRelatives(folShape, p=True)[0]
        folTrans = mc.rename(folTransTemp, '%sTrans' % folShape)
        mc.setAttr('%s.visibility' % folTrans, 0)
        mc.connectAttr('%s.local' % surfaceShape, '%s.inputSurface' % folShape)
        mc.connectAttr('%s.worldMatrix[0]' % surfaceShape, '%s.inputWorldMatrix' % folShape)
        mc.connectAttr('%s.outRotate' % folShape, '%s.rotate' % folTrans)
        mc.connectAttr('%s.outTranslate' % folShape, '%s.translate' % folTrans)
        mc.connectAttr('%s.parameterU' % cpos, '%s.parameterU' % folShape)
        mc.connectAttr('%s.parameterV' % cpos, '%s.parameterV' % folShape)
        mc.orientConstraint(folTrans, rigJnts[i], mo=True)
        follicles.append(folTrans)

    return (follicles)


