"""
transform @ utils

Functions to manipulate and create transforms
"""

import maya.cmds as mc

from . import name

def makeOffsetGrp( object, prefix = ''):

	"""
	make offset group for given object

	@param object: transform object to get offset group
	@param prefix: str, prefix to name new objects
	@return: str, name of new offset group  
	"""

	if not prefix:

		prefix = name.removeSuffix(object)

	offsetGrp = mc.group( n = prefix + 'offset_grp', em = 1)

	objectParents = mc.listRelatives( object, p = 1)

	if objectParents:

		mc.parent(offsetGrp, objectParents[0] )

	#match object transform

	mc.matchTransform(object, offsetGrp)

	#parent object under offset group

	mc.parent( object, offsetGrp)

	return offsetGrp


