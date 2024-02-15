"""
hand @ rig
"""

import maya.cmds as mc
from . import fkChain
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

        # Get finger joint chains
        indexBaseJoint = self.fingerBaseJoints[0]
        middleBaseJoint = self.fingerBaseJoints[1]
        ringBaseJoint = self.fingerBaseJoints[2]
        pinkyBaseJoint = self.fingerBaseJoints[3]
        thumbBaseJoint = self.fingerBaseJoints[4]

        indexFingerJoints = joint.listHierarchy(indexBaseJoint, withEndJoints= False)
        middleFingerJoints = joint.listHierarchy(middleBaseJoint, withEndJoints=False)
        ringFingerJoints = joint.listHierarchy(ringBaseJoint, withEndJoints=False)
        pinkyFingerJoints = joint.listHierarchy(pinkyBaseJoint, withEndJoints=False)
        thumbFingerJoints = joint.listHierarchy(thumbBaseJoint, withEndJoints=False)

        jointConstraints = []

        # Create cup controls
        innerCupCtr = control.Control(prefix='{}_cupInner'.format(self.side), translateTo= self.innerCupJoint,
                                      rotateTo=self.innerCupJoint, offsets=['null', 'zero', 'auto'],
                                   scale=self.rigScale * 0.25, parent = handGrp, shape='circleX')

        outerCupCtr = control.Control(prefix='{}_cupOuter'.format(self.side), translateTo=self.outerCupJoint,
                                      rotateTo=self.outerCupJoint, offsets=['null', 'zero', 'auto'],
                                      scale=self.rigScale * 0.25, parent=handGrp, shape='circleX')

        innerCupConstraint = mc.parentConstraint(innerCupCtr.C, self.innerCupJoint, mo = 1)[0]
        outerCupConstraint = mc.parentConstraint(outerCupCtr.C, self.outerCupJoint, mo=1)[0]
        jointConstraints.append(innerCupConstraint)
        jointConstraints.append(outerCupConstraint)

        # Create finger controls

        if self.metaJoints:

            # Index finger
            indexMetaCtr = control.Control(prefix='{}_indexMeta'.format(self.side),
                                          translateTo = indexFingerJoints[0], rotateTo = indexFingerJoints[0],
                                          scale=self.rigScale * 0.2, parent = handGrp, shape='circleX')
            constraint = mc.parentConstraint(indexMetaCtr.C, indexFingerJoints[0], mo = 1)[0]
            jointConstraints.append(constraint)

            indexFingerRig = fkChain.build(indexFingerJoints[1:], rigScale=self.rigScale * 0.2, parent = indexMetaCtr.C,
                                      offsets=['null', 'zero', 'auto'])

            # Middle finger

            middleMetaCtr = control.Control(prefix='{}_middleMeta'.format(self.side),
                                           translateTo = middleFingerJoints[0], rotateTo = middleFingerJoints[0],
                                           scale=self.rigScale * 0.2, parent=handGrp, shape='circleX')

            constraint = mc.parentConstraint(middleMetaCtr.C, middleFingerJoints[0], mo=1)[0]
            jointConstraints.append(constraint)

            middleFingerRig = fkChain.build(middleFingerJoints[1:], rigScale=self.rigScale * 0.2, parent = middleMetaCtr.C,
                                     offsets=['null', 'zero', 'auto'])

            # Ring Finger

            ringMetaCtr = control.Control(prefix='{}_ringMeta'.format(self.side),
                                            translateTo = ringFingerJoints[0], rotateTo = ringFingerJoints[0],
                                            scale=self.rigScale * 0.2, parent = outerCupCtr.C, shape='circleX')

            constraint = mc.parentConstraint(ringMetaCtr.C, ringFingerJoints[0], mo=1)[0]
            jointConstraints.append(constraint)

            ringFingerRig = fkChain.build(ringFingerJoints[1:], rigScale=self.rigScale * 0.2, parent = ringMetaCtr.C,
                                      offsets=['null', 'zero', 'auto'])

            # Pinky Finger

            pinkyMetaCtr = control.Control(prefix='{}_pinkyMeta'.format(self.side),
                                          translateTo=pinkyFingerJoints[0], rotateTo=pinkyFingerJoints[0],
                                          scale=self.rigScale * 0.2, parent=outerCupCtr.C, shape='circleX')

            constraint = mc.parentConstraint(pinkyMetaCtr.C, pinkyFingerJoints[0], mo=1)[0]
            jointConstraints.append(constraint)

            pinkyFingerRig = fkChain.build(pinkyFingerJoints[1:], rigScale=self.rigScale * 0.2, parent=pinkyMetaCtr.C,
                                          offsets=['null', 'zero', 'auto'])

            # Thumb Finger

            thumbRig = fkChain.build(thumbFingerJoints, rigScale=self.rigScale * 0.25, parent=innerCupCtr.C,
                                           offsets=['null', 'zero', 'auto'])



        else:

            indexFingerRig = fkChain.build(indexFingerJoints, rigScale=self.rigScale * 0.2, offsets = ['null', 'zero', 'auto'],
                                     parent = handGrp)

            middleFingerRig = fkChain.build(middleFingerJoints, rigScale=self.rigScale * 0.2,
                                            parent=handGrp, offsets=['null', 'zero', 'auto'])

            ringFingerRig = fkChain.build(ringFingerJoints, rigScale=self.rigScale * 0.2,
                                            parent=outerCupCtr.C, offsets=['null', 'zero', 'auto'])
            pinkyFingerRig = fkChain.build(pinkyFingerJoints, rigScale=self.rigScale * 0.2,
                                          parent=outerCupCtr.C, offsets=['null', 'zero', 'auto'])

            thumbRig = fkChain.build(thumbFingerJoints, rigScale=self.rigScale * 0.25,
                                          parent=innerCupCtr.C, offsets=['null', 'zero', 'auto'])




        # Create hand control with special attributes

        handCtr = control.Control(prefix='{}_hand'.format(self.side), translateTo= handGrp,
                                      rotateTo= handGrp, lockChannels= ['t', 'r', 's', 'v'],
                                      scale=self.rigScale * 0.25, parent = handGrp, shape='square')

        # move hand ctr
        if self.side == 'l':
            mc.move(3, handCtr.Off, x=True, os=1, r=1, wd=1)

        elif self.side == 'r':
            mc.move(-3, handCtr.Off, x=True, os=1, r=1, wd=1)

        # Move constraints on joints
        constraintGroup = mc.group(em = 1, n = '{}_constraintGrp'.format(self.prefix) )
        mc.parent(constraintGroup, self.rigmodule.noXformGrp)
        mc.parent(indexFingerRig['constraints'], constraintGroup )
        mc.parent(middleFingerRig['constraints'], constraintGroup)
        mc.parent(ringFingerRig['constraints'], constraintGroup)
        mc.parent(pinkyFingerRig['constraints'], constraintGroup)
        mc.parent(thumbRig['constraints'], constraintGroup)
        mc.parent(jointConstraints, constraintGroup)


        # Special attributes

        indexCurl_attr = 'Index_Curl'
        middleCurl_attr = 'Middle_Curl'
        ringCurl_attr = 'Ring_Curl'
        pinkyCurl_attr = 'Pinky_Curl'
        thumbCurl_attr = 'Thumb_Curl'
        spread_attr = 'Spread'
        #cup_attr = 'Cup'

        allAttrs = [spread_attr, thumbCurl_attr, indexCurl_attr, middleCurl_attr, ringCurl_attr, pinkyCurl_attr ]


        for attr in allAttrs:
            mc.addAttr(handCtr.C, ln = attr, at = 'double', min = -5, max = 10, dv=0, k=1)

        # SDKs to drive finger curls
        curlAttrs = [thumbCurl_attr, indexCurl_attr, middleCurl_attr, ringCurl_attr, pinkyCurl_attr]
        fingerRigs = [thumbRig, indexFingerRig, middleFingerRig, ringFingerRig, pinkyFingerRig]

        for i in range(len(curlAttrs)):

            driver = '{}.{}'.format(handCtr.C, curlAttrs[i])
            driverValue1 = 0
            drivenValue1 = 0
            driverValue2 = 10
            drivenValue2 = -90

            for ctr in fingerRigs[i]['controls']:

                driven = '{}.rz'.format(ctr.Offsets[1])

                mc.setDrivenKeyframe(driven,
                                     currentDriver=driver,
                                     driverValue=driverValue1,
                                     value=drivenValue1,
                                     inTangentType='spline',
                                     outTangentType='spline')

                mc.setDrivenKeyframe(driven,
                                     currentDriver =driver,
                                     driverValue = driverValue2,
                                     value = drivenValue2 ,
                                     inTangentType='spline',
                                     outTangentType='spline')

                animCurve = mc.keyframe(driven, query=True, name=True)[0]
                mc.setAttr('{}.preInfinity'.format(animCurve), 1)


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
        mc.setAttr('{}.preInfinity'.format(animCurve), 1)