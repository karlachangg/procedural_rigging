"""
name @ utils
Utilities to work with names and strings
"""

def removeSuffix(name): 

	"""
	Remove suffix from given name string

	@param name: given name string to process
	@return: str, name without suffix
	"""

	edits = name.split('_')
	
	if len(edits) < 2:

		return name
	
	suffix = '_' + edits[-1]
	nameNoSuffix = name[ :-len(suffix)]

	return nameNoSuffix

def removePrefix(name):

	"""
	Remove prefix from given name string

	@param name: given name string to process
	@return: str, name without prefix
	"""
	edits = name.split('_')

	if len(edits) < 2:
		return name

	prefixU = edits[0] + '_'
	namenoprefix = name[ len(prefixU):]

	return namenoprefix

def findAndReplace(name, find, replace):

	"""
	Remove 'u' attached to string of maya objects after maya object is converted to string

	@param name: given name string to process
	@return: str, name without 'u'
	"""
	newName = name.replace(find, replace)

	return newName

def addPrefix(name, prefix):

	"""
	Add prefix to given game string

	@param name: given name string to process
	@return: str, name with prefix
	"""
	newName = prefix + str(name)
	return newName

def getMayaStringName( name ):

	"""
	Remove [u''] part of maya names

	@param name: given name
	#return: str, name of maya object without stuff
	"""

	getName = findAndReplace(str(name), '[u\'', '' )
	finalName = findAndReplace( getName, '\']', '')
	return finalName


def getName(name):

	"""
	get the name of a string assuming its after the prefix
	:return:
	"""

	edits = name.split('_')

	if len(edits) < 2:
		return name

	name = edits[1]

	return name




