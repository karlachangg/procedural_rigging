"""
hand @ rig
"""

import maya.cmds as mc
from . import fkChain
from . import fingerCurl

from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name


class Hand():

    def __init__(self,
            fingerBaseJoints,
            handAttachGrp,
            metaJoints = True,
            innerCupJoint = '',
            outerCupJoint = '',
            includeFingerEnds = False,
            moveHandCtr = 'x',
            prefix = 'hand',
            side = 'l',
            rigScale = 1.0,
            baseRig = None
            ):
        """
        :param armJoints: list(str), shoulder - elbow - wrist
        :param scapulaJoint: str, scapula position joint
        :param prefix: str, prefix to name new objects
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """


        self.fingerBaseJoints = []

        for jnt in fingerBaseJoints:
            newJnt = side + '_' + jnt
            self.fingerBaseJoints.append(newJnt)

        self.metaJoints = metaJoints

        self.innerCupJoint = ''

        if innerCupJoint:
            self.innerCupJoint = side + '_' + innerCupJoint

        self.outerCupJoint = ''

        if outerCupJoint:
            self.outerCupJoint = side + '_' + outerCupJoint

        self.handAttachGrp = handAttachGrp
        self.includeFingerEnds = includeFingerEnds

        self.moveHandCtr = moveHandCtr
        self.prefix = side + '_' + prefix
        self.side = side
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)


    def build(self):

        # Make group to drive whole hand
        handGrp = mc.group(em = 1, n = '{}_handGrp'.format(self.prefix))

        # Connect group to handFollowGrp
        mc.parentConstraint(self.handAttachGrp, handGrp, mo = 0)
        mc.parent(handGrp, self.rigmodule.controlsGrp )

        fingerJointChains = []
        for fingerBaseJnt in self.fingerBaseJoints:
            jointChain = joint.listHierarchy(fingerBaseJnt, withEndJoints= False)
            fingerJointChains.append(jointChain)


        # Make group to hold joint constraints
        constraintGroup = mc.group(em=1, n='{}_constraintGrp'.format(self.prefix))
        mc.parent(constraintGroup, self.rigmodule.noXformGrp)

        jointConstraints = []

        # Create cup controls
        if self.innerCupJoint:

            innerCupCtr = control.Control(prefix='{}_cupInner'.format(self.side), translateTo= self.innerCupJoint,
                                          rotateTo=self.innerCupJoint, offsets=['null', 'zero', 'auto'],
                                       scale=self.rigScale * 0.25, parent = handGrp, shape='circleX')

            innerCupConstraint = mc.parentConstraint(innerCupCtr.C, self.innerCupJoint, mo=1)[0]
            jointConstraints.append(innerCupConstraint)

        if self.outerCupJoint:

            outerCupCtr = control.Control(prefix='{}_cupOuter'.format(self.side), translateTo=self.outerCupJoint,
                                          rotateTo=self.outerCupJoint, offsets=['null', 'zero', 'auto'],
                                          scale=self.rigScale * 0.25, parent=handGrp, shape='circleX')

            outerCupConstraint = mc.parentConstraint(outerCupCtr.C, self.outerCupJoint, mo=1)[0]
            jointConstraints.append(outerCupConstraint)



        # Create finger controls

        fingerRigs = []
        for fingerChain in fingerJointChains:

            fingerRig = fkChain.build(fingerChain, rigScale=self.rigScale * 0.2,
                                           offsets=['null', 'zero', 'auto'],
                                           parent=handGrp)

            fingerRigs.append(fingerRig)
            jointConstraints.append(fingerRig['constraints'])

        for constraint in jointConstraints:
            mc.parent(constraint, constraintGroup)


        # If there is an inner cup joint, parent the ring and pinky finger to it
        # How to more easily know which finger rig is the ring and pinky? Maybe there is a smarter way, hardcoding for now based on order

        if self.outerCupJoint:
            ringFingerRig = fingerRigs[2]
            pinkyFingerRig = fingerRigs[3]
            mc.parent(ringFingerRig['topControl'].Off, outerCupCtr.C)
            mc.parent(pinkyFingerRig['topControl'].Off, outerCupCtr.C)

        if self.innerCupJoint:
            thumbRig = fingerRigs[-1]
            mc.parent(thumbRig['topControl'].Off, innerCupCtr.C)



        # Create hand control with special attributes

        handCtr = control.Control(prefix='{}_hand'.format(self.prefix), translateTo= handGrp,
                                      rotateTo= handGrp, lockChannels= ['t', 'r', 's', 'v'],
                                      scale=self.rigScale * 0.25, parent = handGrp, shape='sun', color = 'green')

        # move hand ctr
        units = 3 * self.rigScale
        x, y, z = False, False, False
        if 'x' in self.moveHandCtr:
            x = True

        if 'y' in self.moveHandCtr:
            y = True

        if 'z' in self.moveHandCtr:
            z = True

        if '-' in self.moveHandCtr:
            units = units * -1

        # move switch ctr
        mc.move(units, handCtr.Off, x=x, y=y, z=z, os=1, r=1, wd=1)





        # Add curl attributes

        fingerControlsDict = {}

        for rig in fingerRigs:
            # controls of one finger
            controls = rig['controls']

            fingerControls  = []

            for i in range(len(controls)):

                if i == 0:
                    if self.metaJoints:
                        continue

                fingerControls.append(controls[i].C)


            # get name of finger
            fingerName = name.getName(fingerControls[0])

            fingerControlsDict[fingerName] = fingerControls




        fingerCurl.addFingerCurlDrivers(fingerControls=fingerControlsDict,
            ctr = handCtr.C,
            prefix = self.prefix,
            curlAxis = '-z')




        '''spread_attr = 'Spread'

        mc.addAttr(handCtr.C, ln = spread_attr, at = 'double', min = -5, max = 10, dv=0, k=1)

        # SDKs to drive finger curls
        curlAttrs = [thumbCurl_attr, indexCurl_attr, middleCurl_attr, ringCurl_attr, pinkyCurl_attr]
        fingerRigs = [thumbRig, indexFingerRig, middleFingerRig, ringFingerRig, pinkyFingerRig]


        # SDK for finger spread

        driver = '{}.{}'.format(handCtr.C, spread_attr )
        driverValue1 = 0
        drivenValue1 = 0
        driverValue2 = 10
        spread_angle = 45

        # Index finger
        driven = '{}.ry'.format(indexFingerRig['topControl'].Offsets[1])

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=driverValue1,
                             value=drivenValue1,
                             inTangentType='spline',
                             outTangentType='spline')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=driverValue2,
                             value= spread_angle * -1,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurve = mc.keyframe(driven, query=True, name=True)[0]
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)

        # Ring finger
        driven = '{}.ry'.format(ringFingerRig['topControl'].Offsets[1])

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=driverValue1,
                             value=drivenValue1,
                             inTangentType='spline',
                             outTangentType='spline')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=driverValue2,
                             value=spread_angle * 0.5,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurve = mc.keyframe(driven, query=True, name=True)[0]
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)

        # Pinky finger
        driven = '{}.ry'.format(pinkyFingerRig['topControl'].Offsets[1])

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=driverValue1,
                             value=drivenValue1,
                             inTangentType='spline',
                             outTangentType='spline')

        mc.setDrivenKeyframe(driven,
                             currentDriver=driver,
                             driverValue=driverValue2,
                             value=spread_angle,
                             inTangentType='spline',
                             outTangentType='spline')

        animCurve = mc.keyframe(driven, query=True, name=True)[0]
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)'''
