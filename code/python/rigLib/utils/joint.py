"""
joint utils @ utils
"""

import maya.cmds as mc

def listHierarchy( topJoint, withEndJoints = True ):

	"""
	list joint hierarchy starting with top joint

	@param topJoint: str, joint to get listed with its joint hierarchy
	@param withEndJoints: bool, list hierarchy including end joints
	@return: list( str), listed joints starting with top joint

	"""

	listedJoints = mc.listRelatives( topJoint, type = "joint", ad = True)
	listedJoints.append(topJoint)
	listedJoints.reverse()

	completeJoints = listedJoints[:]

	if not withEndJoints:

		#c stands for child, checking if a joint doesn't have a child

		completeJoints = [ j for j in listedJoints if mc.listRelatives( j, c = 1, type = 'joint') ]

	return completeJoints


def jointOrient( Joints ):


	mc.select( Joints[0])
	mc.joint(e = 1, oj = 'xzy', secondaryAxisOrient = 'zup', ch = 1, zso = 1)
	endJnt = Joints[-1]
	mc.select(endJnt )
	mc.xform( r = 1, ro = ( -90, -90, 0) )
	mc.makeIdentity( apply = 1,t = 0, r = 1, s = 0, n = 0, pn = 1)

def duplicateChain( joints, oldSuffix = 'jnt', newSuffix = 'jnt1', prefix = ''):

	# clear maya selection
	mc.select(cl=1)

	if not isinstance(joints, list ):
		joints = [joints]

	newJoints = []

	for jnt in joints:
		name = jnt.replace(oldSuffix, newSuffix)
		name = prefix + name
		newJoints.append(name)


	# make new joint chain

	for i in range(len(joints)):

		jnt = mc.duplicate( joints[i], n = newJoints[i], po = 1, ic = 0, un = 0)
		parent = mc.listRelatives(jnt, p = 1)
		if i == 0:

			if parent is not None:
				mc.parent(jnt, world = 1)

		else:
			mc.parent(jnt, newJoints[i-1])

	# clear maya selection
	mc.select(cl = 1)

	return newJoints



def segmentJointchain(startJoint, endJoint, numberOfSegments, prefix='', aimAxis='x'):

	insertJoints = []
	numberOfNewJoints = numberOfSegments - 1

	for i in range(numberOfNewJoints):
		name = '{}_{}_jnt'.format(prefix, i)
		insertJoints.append(name)

	wholeLength = mc.getAttr('{}.t{}'.format(endJoint, aimAxis))
	segmentLength = wholeLength / numberOfSegments

	if aimAxis == 'x':
		newPosition = [segmentLength, 0, 0]

	elif aimAxis == 'y':
		newPosition = [0, segmentLength, 0]

	elif aimAxis == 'z':
		newPosition = [0, 0, segmentLength]

	for i in range(numberOfNewJoints):

		if i == 0:
			jnt = mc.insertJoint(startJoint)
			newJnt = insertJoints[i]
			mc.rename(jnt, newJnt)
			mc.joint(newJnt, e=1, position=newPosition, r=True, co=True)


		else:

			jnt = mc.insertJoint(insertJoints[i - 1])
			newJnt = insertJoints[i]
			mc.rename(jnt, newJnt)
			mc.joint(newJnt, e=1, position=newPosition, r=True, co=True)


	jointList = [startJoint]

	for i in range(numberOfNewJoints):
		jointList.append(insertJoints[i])
	jointList.append(endJoint)

	return jointList

