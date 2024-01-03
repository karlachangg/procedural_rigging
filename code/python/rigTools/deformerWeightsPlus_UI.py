
'''
Skincluster Weights Importer /Exporter Tool
ui window module
'''
from . import deformerWeightsPlus

import maya.cmds as mc
import maya.mel as mel
import os
#import json
from functools import partial

class deformerWeightsPlusUI():
	'''
	Class for animaiton importer/exporter UI window and methods
	'''

	def __init__(self):

		# Set Window variables
		self.win = 'deformerWeights_createWin'
		self.winTitle = 'Deformer Weights Import/Export'
		self.winWidth = 500
		self.winHeight = 200

		self.geoList = []
		self.swExt = '.xml'

	
	def buildUI(self):

		'''
		Build and show UI window
		'''

		mc.windowPref (self.win, width = self.winWidth, height = self.winHeight)

		if ( mc.window( self.win, exists = True)):
			mc.deleteUI (self.win)

		mc.window( self.win, rtf = True, width = self.winWidth, height = self.winHeight, title = self.winTitle, s = True)

		# First column layout for elements
		mainCL = mc.columnLayout(columnAttach = ('both', 5 ), columnWidth = self.winWidth, adjustableColumn = True)
		# Instructional text
		mc.text('Select objects with skincluster deformers to export or import deformer weights.', 
			align = 'center', w = self.winWidth, h= 40)
		mc.separator(style='in', height=20)

		# Set character name if it exists
		self.charName_TFBG = mc.textFieldButtonGrp( label='Character:', editable = 1, buttonLabel='Load Selected',
			columnAlign = [1, 'left'], buttonCommand= self.setCharacterName )

		# Input field for filepath and name to export

		self.filepath_TFBG = mc.textFieldButtonGrp( label='Filepath:', editable = 1, buttonLabel='Set',
			columnAlign = [1, 'left'], buttonCommand= self.setFilepath )

		#Add row layout for 2 items 
		tmpRowWidth = [self.winWidth*0.2, self.winWidth*0.8]
		mc.rowLayout(numberOfColumns=2, columnWidth2=tmpRowWidth, rowAttach = [1, 'top', 0])

		# Meshes title text
		mc.text('Meshes:', align='left', w= tmpRowWidth[0])

		# Second column layout for timeline elements
		mc.columnLayout(columnAttach = ('both', 5 ), columnWidth = tmpRowWidth[1], adjustableColumn = True)

		self.meshes_RBCOLL = mc.radioCollection()
		# Option 1 radio button 
		self.selected_RB = mc.radioButton('selectedMeshes', l ='Selected', onCommand = partial (self.toggleMeshOption, state= 0) )

		# Text field buttom group for selected meshes
		self.selectMesh_TFBG = mc.textFieldButtonGrp(label='', editable = True, buttonLabel='Load Selected',
													 columnAttach3 = ['left', 'left', 'left'],
													 columnWidth3 = [tmpRowWidth[1] * 0.1, tmpRowWidth[1]  * 0.5, tmpRowWidth[1] * 0.3],
													 adjustableColumn = 2 , buttonCommand=self.loadSelectedMesh, enableButton = 0,
													 changeCommand = self.validateEntry )

		# Option 2 radio button
		self.all_RB = mc.radioButton('allMeshes', l ='All', onCommand = partial (self.toggleMeshOption, state = 1)  )
		
		# Set parent back to main column layout
		mc.setParent(mainCL)

		mc.separator( style = 'none', height=5)
		
		# Save button
		save_BTN = mc.button(l='Save Skin Weights', w=self.winWidth, align='center', command= self.saveSkinWeights )
		mc.separator( style = 'none', height=5)

		# Load button
		load_BTN = mc.button(l='Load Skin Weights', w=self.winWidth, align='center', command= self.loadSkinWeights )

		# Set initial parameters
		self.setInitialState()
		
		# Show the UI window
		mc.showWindow(self.win)
		mc.window(self.win, edit = True, width = self.winWidth, height = self.winHeight)


	def setInitialState(self):

		mc.radioButton(self.selected_RB, edit=1, select =1)
		mc.textFieldButtonGrp(self.selectMesh_TFBG, edit=1, enableButton=1, editable=1)



	def setCharacterName(self, *args):

		pass

	def loadSelectedMesh(self, *args):

		# Get selected objects
		selected = mc.ls(sl=1)

		if not selected:
			raise Exception("No objects selected. Please select at least one skinned mesh.")

		for obj in selected:

			if not self.isValid(obj):
				raise Exception("Please select only skinned meshes.")

		# format list for text field
		meshObjects = formatList_to_string(selected)
		mc.textFieldButtonGrp(self.selectMesh_TFBG, edit = 1, text = meshObjects )
		self.setSelectedMeshes()



	def isValid(self, object):

		# check that object exists:
		if not mc.objExists(object):
			print("{} does not exist.".format(object))
			return False

		# check that object is a mesh object
		if not deformerWeightsPlus.isMesh(object):
			print("{} is not a mesh object.".format(object))
			return False

		# check that object has a skincluster deformer
		if not deformerWeightsPlus.findRelatedSkinCluster(object):
			print("{} does not have a skinCluster attached.".format(object))
			return False

		else:
			return True




	def validateEntry(self, *args):

		print("Validating...")

		# Get value from text field
		text = mc.textFieldButtonGrp(self.selectMesh_TFBG, q=1, text=1)

		if text:
			objects = formatString_to_list(text)

			for obj in objects:
				if not self.isValid(obj):
					print('Invalid Entry')
					mc.textFieldButtonGrp(self.selectMesh_TFBG, edit=1, text='')
					return

			# Validated
			self.setSelectedMeshes()

		else:
			# set geo List to empty
			self.geoList = []
			print('no loaded meshes')
			print('Geometry: {}'.format(self.geoList))










	def setSelectedMeshes(self):

		# Get value from text field
		text = mc.textFieldButtonGrp(self.selectMesh_TFBG, q = 1, text = 1 )

		if (text):
			# Set self.geoList to input
			objects = formatString_to_list(text)
			self.geoList = objects
			print('Geometry: {}'.format(self.geoList))

		else:
			# set geo List to empty
			self.geoList = []
			print('no loaded meshes')
			print('Geometry: {}'.format(self.geoList))


		'''

		skinnedGeo = self.loadSelectedMesh()

		self.geoList = skinnedGeo

		print('Geometry: {}'.format(self.geoList))
		'''


	def getAllSkinnedMeshes(self):

		skinClusters = mc.ls(type = 'skinCluster')
		skinnedGeo = []

		for cluster in skinClusters:
			shape = mc.skinCluster(cluster, q = 1, g = 1)[0]
			print(shape)
			geo = mc.listRelatives(shape, p = 1)[0]
			skinnedGeo.append(geo)

		return skinnedGeo


	def setAllSkinnedMeshes(self):

		skinnedGeo = self.getAllSkinnedMeshes()

		self.geoList = skinnedGeo

		print('Geometry: {}'.format(self.geoList))



	def getCharacterName(self, *args):

		# Get character name from input field (if any)
		characterName = mc.textFieldButtonGrp(self.charName_TFBG, q=True, text=True)

	def saveSkinWeights(self, *args):

		# Get file name from input field
		filepath = mc.textFieldButtonGrp(self.filepath_TFBG, q=True, text=True)

		for obj in self.geoList:
			wtFile = os.path.join(filepath, obj + self.swExt)
			sdw = deformerWeightsPlus.SkinDeformerWeights()
			sdw.saveWeightInfo(fpath = wtFile, meshes = [obj])


	def loadSkinWeights(self, *args):

		"""
		    load skin weights for character geometry objects
		    """

		# weight folders
		wtDir = mc.textFieldButtonGrp(self.filepath_TFBG, q=True, text=True)
		#wtDir = os.path.join(project.mainProjectPath, characterName, skinWeightsDir)
		wtFiles = os.listdir(wtDir)

		print('wtDir is: ' + wtDir)
		print(wtFiles)

		# load skin weights

		for wtFile in wtFiles:

			print(wtFile)
			extRes = os.path.splitext(wtFile)
			print(extRes)

			# check extension format
			if not extRes:
				continue

			# check skin weight file
			if not extRes[1] == self.swExt:
				continue

			# check geometry list
			if self.geoList and not extRes[0] in self.geoList:
				continue

			# check if object exists
			if not mc.objExists(extRes[0]):
				continue

			fullpathWtFile = os.path.join(wtDir, wtFile)
			sdw = deformerWeightsPlus.SkinDeformerWeights(path=fullpathWtFile)
			sdw.applyWeightInfo()



	def toggleMeshOption(self,   *args , state = 0 ):

		# "Selected mesh" option
		if state == 0:
			mc.textFieldButtonGrp(self.selectMesh_TFBG, edit  = 1, enableButton = 1, editable = 1 )
			self.setSelectedMeshes()

		# "All meshes" option
		elif state == 1:
			mc.textFieldButtonGrp(self.selectMesh_TFBG, edit = 1, enableButton = 0, editable = 0)
			self.setAllSkinnedMeshes()





	
	def setFilepath(self, *args):
		
		'''
		Open file browser to set filename for exporting
		'''

		folder = mc.fileDialog2(fileMode = 3, dialogStyle = 2, okCaption = 'Set')[0]
		mc.textFieldButtonGrp(self.filepath_TFBG, edit=1, text = folder)



	def getSelectedObject(self):
		'''
		Get selection from maya. Throw error if no object is selected, if more than one object is selected, 
		or if selected object is not transformable.

		@return: maya object name 
		'''

		# Get selected object
		selected = mc.ls(sl=1)

		if not selected:
			raise Exception("No object selected. Please select an object.")
		if len(selected) > 1:
			raise Exception("More than one object selected. Please select only one object.")

		if not mc.objectType(selected, isType = 'transform'):
			raise Exception("Invalid selection. Please select a transform object.")
		

		return selected[0]




def formatList_to_string(list):

	string = ''

	for i in range(len(list)):

		string += list[i]

		if i != len(list) - 1:
			string += ', '

	return string

def formatString_to_list(string):

	string = ''.join(string.split())
	if string[-1 ]== ',':
		string = string[:-1]

	list = string.split(",")
	return list
