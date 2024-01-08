
'''
Skincluster Weights Importer /Exporter Tool
ui window module
'''
from . import deformerWeightsPlus
from charRig import project

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
		self.charName_TFBG = mc.textFieldButtonGrp( label='Character (Optional):', editable = 1, buttonLabel='Load Selected',
			columnAlign = [1, 'left'], buttonCommand= self.loadSelectedCharacter, changeCommand = self.setFilepath_field )

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
													  )

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

		mc.radioButton(self.all_RB, edit=1, select = 1)
		mc.textFieldButtonGrp(self.selectMesh_TFBG, edit=1, enableButton=0, editable=0)

		char = self.getTopNode()
		if char:
			mc.textFieldButtonGrp(self.charName_TFBG, edit=1, text=char)
			self.setFilepath_field()

	def loadSelectedCharacter(self, *args):

		# Get selected objects
		selected = mc.ls(sl=1)

		if not selected:
			raise Exception("No objects selected.")

		if len(selected) > 1:
			raise Exception("More than one object selected. Please select only one object.")

		if not mc.objectType(selected, isType='transform'):
			raise Exception("Invalid selection. Please select a transform object.")

		text = formatList_to_string(selected)
		mc.textFieldButtonGrp(self.charName_TFBG, edit=1, text = text)
		self.setFilepath_field()

	def setFilepath_field(self, *args):

		characterName = mc.textFieldButtonGrp(self.charName_TFBG, q= 1, text = 1)
		wtFolder = os.path.join(project.mainProjectPath, characterName, project.skinWeightsDir )
		mc.textFieldButtonGrp(self.filepath_TFBG, edit=1, text = wtFolder)


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



	def isValid(self, object, action = ''):

		# check that object exists:
		if not mc.objExists(object):
			print("{} does not exist.".format(object))
			return False

		# check that object is a mesh object
		if not deformerWeightsPlus.isMesh(object):
			print("{} is not a mesh object.".format(object))
			return False

		if action == 'save':

			# check that object has a skincluster deformer
			if not deformerWeightsPlus.findRelatedSkinCluster(object):
				print("{} does not have a skinCluster attached.".format(object))
				return False
			else:
				return True

		if action == 'load':
			# check if object has a skincluster deformer
			if deformerWeightsPlus.findRelatedSkinCluster(object):
				print("{} has a skinCluster attached.".format(object))
				return False
			else:
				return True


		else:
			return True


	def getAllSkinnedMeshes(self):

		skinClusters = mc.ls(type = 'skinCluster')
		skinnedGeo = []

		for cluster in skinClusters:
			shape = mc.skinCluster(cluster, q = 1, g = 1)[0]
			print(shape)
			geo = mc.listRelatives(shape, p = 1)[0]
			skinnedGeo.append(geo)

		return skinnedGeo

	def getAllMeshes(self):

		shapes = mc.ls(type = 'mesh')
		meshes = []

		for each in shapes:
			geo = mc.listRelatives(each, parent = 1)[0]

			if not deformerWeightsPlus.findRelatedSkinCluster(geo):

				if geo not in meshes:
					meshes.append(geo)

		print('All non-skinned meshes: {}'.format(meshes))
		return meshes

	def getTopNode(self):
		topNode = ''

		# Get top node in scene if there is only one
		assemblies = mc.ls(assemblies = True)

		remove = ['persp', 'top', 'side', 'front']
		for each in remove:
			assemblies.remove(each)
		if len(assemblies) == 1:
			topNode = assemblies[0]

		return topNode


	def getGeometry(self, action =  ''):
		# Get geometry to export or import
		option = mc.radioCollection(self.meshes_RBCOLL, q=1, select=1)

		if action == 'save':

			if option == 'selectedMeshes':
				geometry = []
				# Get value from text field
				text = mc.textFieldButtonGrp(self.selectMesh_TFBG, q=1, text=1)
				if text:
					# Format valid input to string of objects
					objects = formatString_to_list(text)

					for obj in objects:

						if self.isValid(obj, action = 'save'):
							geometry.append(obj)

						else:
							print('Skipping...')
				else:
					print('Please enter a value for selected meshes.')


			elif option == 'allMeshes':
				geometry = self.getAllSkinnedMeshes()

			return geometry

		if action == 'load':

			if option == 'selectedMeshes':
				geometry = []
				# Get value from text field
				text = mc.textFieldButtonGrp(self.selectMesh_TFBG, q=1, text=1)
				# Format valid input to string of objects
				objects = formatString_to_list(text)

				for obj in objects:

					if self.isValid(obj, action = 'load'):
						geometry.append(obj)

					else:
						print('Skipping...')


			elif option == 'allMeshes':
				geometry = self.getAllMeshes()

			return geometry





	def saveSkinWeights(self, *args):

		# Get file name from input field
		filepath = mc.textFieldButtonGrp(self.filepath_TFBG, q=True, text=True)

		geometry = self.getGeometry(action = 'save')
		if not geometry:
			return

		print('Attempting to export weights for: {}'.format(geometry))

		# if filepath doesnt exist throw error
		if not os.path.isdir(filepath):
			raise Exception("Path: \"{}\" does not exist. Please enter a valid directory.".format(filepath))

		for obj in geometry:

			wtFile = os.path.join(filepath, obj + self.swExt)
			sdw = deformerWeightsPlus.SkinDeformerWeights()

			try:
				sdw.saveWeightInfo(fpath = wtFile, meshes = [obj])
			except:
				print('Cannot export weights for: {}. Skipping...'.format(obj))


	def loadSkinWeights(self, *args):

		"""
		    load skin weights for character geometry objects
		"""

		# weight folders
		wtDir = mc.textFieldButtonGrp(self.filepath_TFBG, q=True, text=True)
		wtFiles = os.listdir(wtDir)

		# get geometry to load weights onto
		geometry = self.getGeometry(action = 'load')
		if not geometry:
			return

		# load skin weights

		for wtFile in wtFiles:

			extRes = os.path.splitext(wtFile)

			# check extension format
			if not extRes:
				continue

			# check skin weight file
			if not extRes[1] == self.swExt:
				continue

			# check geometry list
			if geometry and not extRes[0] in geometry:
				continue

			# check if object exists
			if not mc.objExists(extRes[0]):
				continue

			fullpathWtFile = os.path.join(wtDir, wtFile)
			print("Loading: /'{}/' ...".format(fullpathWtFile))
			sdw = deformerWeightsPlus.SkinDeformerWeights(path=fullpathWtFile)
			sdw.applyWeightInfo()



	def toggleMeshOption(self,   *args , state = 0 ):

		# "Selected mesh" option
		if state == 0:
			mc.textFieldButtonGrp(self.selectMesh_TFBG, edit  = 1, enableButton = 1, editable = 1 )

		# "All meshes" option
		elif state == 1:
			mc.textFieldButtonGrp(self.selectMesh_TFBG, edit = 1, enableButton = 0, editable = 0)






	
	def setFilepath(self, *args):
		
		'''
		Open file browser to set filename for exporting
		'''

		folder = mc.fileDialog2(fileMode = 3, dialogStyle = 2, okCaption = 'Set')[0]
		mc.textFieldButtonGrp(self.filepath_TFBG, edit=1, text = folder)


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
