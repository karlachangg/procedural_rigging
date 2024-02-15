
import maya.cmds as mc
from ..base import control

def addFingerCurlDrivers( fingerControls, ctr = '', follow = '', prefix = '', ctrlParent = '', curlAxis = 'z'):

    '''

    :param fingerControls: dictionary of finger controls
    Ex:
    fingerControls = { 'finger1': ['index1_Ctr', 'index2_Ctr', 'index3_Ctr'],
                        'finger2': ['middle1_Ctr', 'middle2_Ctr', 'middle3_Ctr'],
                        'finger3': ['pinky1_Ctr', 'pinky2_Ctr', 'pinky3_Ctr']
                        }
    :param control: control on which to add curl attributes. If none provided, make one.
    '''

    # Special attributes


    if not ctr:

        # Create hand control with special attributes
        #curlControl = control.Control()

        fingersCtr = control.Control(prefix = prefix, translateTo = follow, scale = 1, parent = ctrlParent, shape='circle')
        mc.parentConstraint(follow, fingersCtr.Off, mo = 0)
        ctr = fingersCtr.C

    if 'x' in curlAxis:
        curlAttr = 'rx'
    elif 'y' in curlAxis:
        curlAttr = 'ry'
    elif 'z' in curlAxis:
        curlAttr = 'rz'

    curlDirection = 1
    if '-' in curlAxis:
        curlDirection = -1

    # Create list of finger curl attributes
    fingerCurl_attrs = []
    fingerKeys = list(fingerControls.keys())

    for key in fingerKeys:
        # Get the prefix of the finger

        fingerPrefix = key
        fingerCurlAttr = fingerPrefix + '_Curl'
        fingerCurl_attrs.append(fingerCurlAttr)

    #fingerCurl_attrs.append('Spread')


    #allAttrs = [spread_attr, thumbCurl_attr, indexCurl_attr, middleCurl_attr, ringCurl_attr, pinkyCurl_attr ]


    for attr in fingerCurl_attrs:
        mc.addAttr(ctr, ln = attr, at = 'double', min = -5, max = 10, dv=0, k=1)

    # SDKs to drive finger curls

    for i in range(len(fingerCurl_attrs)):

        driver = '{}.{}'.format(ctr, fingerCurl_attrs[i])
        driverValue1 = 0
        drivenValue1 = 0
        driverValue2 = 10
        drivenValue2 = 90 * curlDirection

        fingerKey = fingerKeys[i]
        finger = fingerControls[fingerKey]

        for each in finger:
            # Get offset group above finger control
            fingerOffset = mc.listRelatives(each, parent = 1)[0]

            driven = '{}.{}'.format(fingerOffset, curlAttr)



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

    '''
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
    
    '''





def getPrefix(name):

	edits = name.split('_')

	if len(edits) < 2:
		return name

	prefix = edits[0]
	return prefix