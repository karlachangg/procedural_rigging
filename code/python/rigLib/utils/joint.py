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

def duplicateChain( joints, oldSuffix = 'jnt', newSuffix = 'jnt1'):

	# clear maya selection
	mc.select(cl=1)

	if not isinstance(joints, list ):
		joints = [joints]

	newJoints = []

	for jnt in joints:
		newJoints.append(jnt.replace(oldSuffix, newSuffix))


	# make new joint chain

	for i in range(len(joints)):

		jnt = mc.duplicate( joints[i], n = newJoints[i], po = 1, ic = 0, un = 0)

		if i == 0:
			mc.parent(jnt, world = 1)
		else:
			mc.parent(jnt, newJoints[i-1])

	# clear maya selection
	mc.select(cl = 1)

	return newJoints

