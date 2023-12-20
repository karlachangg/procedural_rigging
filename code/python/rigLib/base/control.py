"""
module for making rig controls
"""

import maya.cmds as mc

class Control():

	"""
	class for building rig control.
	"""

	def __init__(
				self, 
				prefix = 'new', 
				scale = 1.0,
				translateTo = '',
				rotateTo = '',
				parent = '',
				shape = 'circle',
				lockChannels = ['s', 'v'],
				offsets = ['null']
				):

		"""
		@param prefix: str, prefix to name new objects
		@param scale: float, scale value for size of control shapes
		@param translateTo: str, reference object for control orientation
		@param rotateTo: str, reference object for control orientation
		@param shape: str, control shape type
		@param lockChannels: list(str), list of channels on control to be locked and non-keyable
		@param offsets: list(str), list of offset groups on control from top to bottom
		@return: None
		"""
		ctrlObject = None
		circleNormal = [1, 0, 0]
		
		if shape in ['circle', 'circleX']:

			circleNormal = [1, 0, 0]

		elif shape == 'circleY':
			
			circleNormal = [0, 1, 0]
		
		elif shape == 'circleZ':
			
			circleNormal = [0, 0, 1]
		
		elif shape == 'sphere':
			
			ctrlObject = mc.circle( n = prefix + '_ctr', ch = False, normal = [1, 0, 0])[0]
			addShape = mc.circle( n = prefix + '_ctr2', ch = False, normal = [0, 0, 1])[0]
			mc.parent( mc.listRelatives( addShape, s = 1 ), ctrlObject, r = 1, s = 1 )
			mc.delete( addShape )
		
		elif shape == 'square':
			ctrlObject = create_square(prefix)
		
		elif shape == 'orb':
			ctrlObject = create_orb(prefix)

		elif shape == 'cube':
			ctrlObject = create_cube(prefix)

		elif shape == 'diamond':
			ctrlObject = create_diamond(prefix)

		if not ctrlObject:

			ctrlObject = mc.circle( n = prefix + '_ctr', ch = False, normal = circleNormal)[0]
		
		
		mc.xform( ctrlObject, s = (scale, scale, scale) )
		mc.makeIdentity( ctrlObject, apply=True )

		offsetGrps = []

		if offsets is not None:


			# Create offset group
			for grp in offsets:
				offset = mc.group( n = '{}_ctr_{}'.format(prefix, grp ), em = 1 )
				offsetGrps.append(offset)

			# Parent offset groups
			for x in range(1, len(offsetGrps)):
				mc.parent(offsetGrps[x], offsetGrps[x-1])

			# Parent control to lowest offset group
			mc.parent(ctrlObject, offsetGrps[-1])

		#ctrlOffsetDriver = mc.group( n = prefix + '_ctr_auto', em = 1 )
		#ctrlOffsetZero = mc.group( n = prefix + '_ctr_zero', em = 1 )
		#ctrlOffsetNull = mc.group( n = prefix + '_ctr_null', em = 1 )
		#mc.parent( ctrlObject, ctrlOffsetDriver )
		#mc.parent( ctrlOffsetDriver, ctrlOffsetZero )
		#mc.parent( ctrlOffsetZero, ctrlOffsetNull )
		

		# Change Colors
	
		ctrlShapes = mc.listRelatives( ctrlObject, s = 1 )
		[ mc.setAttr( s + '.ove', 1 ) for s in ctrlShapes ]

		if prefix.startswith( ('L_', 'l_')):

			[ mc.setAttr( s + '.ovc', 6 ) for s in ctrlShapes ]

		elif prefix.startswith( ('R_', 'r_') ):

			[ mc.setAttr( s + '.ovc', 13 ) for s in ctrlShapes ]

		else:

			[ mc.setAttr( s + '.ovc', 22 ) for s in ctrlShapes ]

		#translate control

		if mc.objExists( translateTo ):
		
			mc.delete( mc.pointConstraint ( translateTo, offsetGrps[0]))


		#rotate control

		if mc.objExists( rotateTo ):

			mc.delete( mc.orientConstraint ( rotateTo, offsetGrps[0]))

		#parent control

		if mc.objExists( parent ):

			if offsets is not None:
				mc.parent( offsetGrps[0], parent )
			else:
				mc.parent(ctrlObject, parent)

		
		#lock control channels

		singleAttributeLockList = []

		for lockChannel in lockChannels:

			if lockChannel in ['t', 'r', 's']:

				for axis in ['x', 'y', 'z']:

					attr = lockChannel + axis
					singleAttributeLockList.append(attr)

			else:

				singleAttributeLockList.append(lockChannel)

		for attr in singleAttributeLockList:

			mc.setAttr( ctrlObject + '.' + attr, l = 1, k = 0)


		# add public members

		self.C = ctrlObject

		if offsets is None:
			self.Off = ctrlObject

		elif len(offsets) > 1:
			self.Off = offsetGrps[0]
			self.Offsets = offsetGrps
		else:
			self.Off = offsetGrps[0]

#Control Shape Functions	
def create_circle():
	global control
	control = mc.circle( nr=[0,1,0])[0]


def create_square(prefix):
	global control
	control = mc.curve(d=1, p=[(-1,0,-1),(1,0,-1),(1,0,1),(-1,0,1), (-1,0,-1)], k=[0,1,2,3,4] , n = prefix + '_CTR')
	return control

def create_move_all(prefix):
	global control
	control = mc.circle(nr=[0,1,0])[0]

	arrow_list = []
	arrow_list.append(mc.curve(d=1, p=[(1.75625, 0, 0.115973),(1.75625, 0, -0.170979),(2.114939, 0, -0.170979),(2.114939, 0, -0.314454),(2.473628, 0, -0.0275029),(2.114939, 0, 0.259448),(2.114939, 0, 0.115973),(1.75625, 0, 0.115973)], k=[0,1,2,3,4,5,6,7]))
	arrow_list.append(mc.curve(d=1, p=[(0.143476, 0, -1.783753),(0.143476, 0, -2.142442),(0.286951, 0, -2.142442),(0, 0, -2.501131),(-0.286951, 0, -2.142442),(-0.143476, 0, -2.142442),(-0.143476, 0, -1.783753),(0.143476, 0, -1.783753)], k=[0,1,2,3,4,5,6,7]))
	arrow_list.append(mc.curve(d=1, p=[(-1.75625, 0, -0.170979),(-2.114939, 0, -0.170979),(-2.114939, 0, -0.314454),(-2.473628, 0, -0.0275029),(-2.114939, 0, 0.259448),(-2.114939, 0, 0.115973),(-1.75625, 0, 0.115973),(-1.75625, 0, -0.170979)], k=[0,1,2,3,4,5,6,7]))
	arrow_list.append(mc.curve(d=1, p=[(-0.143476, 0, 1.728747),(-0.143476, 0, 2.087436),(-0.286951, 0, 2.087436),(0, 0, 2.446125),(0.286951, 0, 2.087436),(0.143476, 0, 2.087436),(0.143476, 0, 1.728747),(-0.143476, 0, 1.728747)], k=[0,1,2,3,4,5,6,7]))

	mc.select(arrow_list)
	mc.pickWalk(d='Down')
	mc.select(control, tgl=True)
	mc.parent(r=True, s=True)
	mc.delete(arrow_list)
	mc.xform(control, cp=True)


def create_sun(prefix):
	global control
	control = mc.circle(s=16, nr=[0,1,0])[0]
	mc.select((control + '.cv[1]'), (control + '.cv[3]'), (control + '.cv[5]'), (control + '.cv[7]'), (control + '.cv[9]'), (control + '.cv[11]'), (control + '.cv[13]'), (control + '.cv[15]'), (control + '.cv[17]'), (control + '.cv[19]'), r=True)
	mc.scale(0.3, 0.3, 0.3, p=[0, 0, 0], r=True)
	mc.makeIdentity(control, apply=True, t=1, r=1, s=1, n=0)
	mc.xform(control, cp=True)
    
 
def create_pick(prefix):
	global control
	control = mc.circle(nr=[0,1,0])[0]
	
	mc.move(control + '.cv[5]', (0, 0, -1.108194) ,r=True)
	mc.move(control + '.cv[1]', (0, 0, 1.108194) ,r=True)
	mc.move(control + '.cv[6]', (-0.783612, 0, -0.783612) ,r=True)
	mc.move(control + '.cv[0]', (-0.783612, 0, 0.783612) ,r=True)
	mc.move(control + '.cv[7]', (-1.108194, 0, 0) ,r=True)

def create_frame(prefix):
	global control
	control = mc.curve(d=1, p=[(-1,0,-1),(-1,0,1),(1,0,1),(1,0,-1),(-1,0,-1),(-2,0,-2),(2,0,-2),(1,0,-1),(1,0,1),(2,0,2),(2,0,-2),(2,0,2),(-2,0,2),(-1,0,1),(-2,0,2),(-2,0,-2)], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15])


def create_triangle(prefix):
	global control
	control = mc.curve(d=1, p=[(-1,0,1),(1,0,1),(0,0,-1),(-1,0,1)], k=[0,1,2,3,])


def create_plus(prefix):
	global control
	control = mc.curve(d=1, p=[(-1,0,-3),(1,0,-3),(1,0,-1),(3,0,-1),(3,0,1),(1,0,1),(1,0,3),(-1,0,3),(-1,0,1),(-3,0,1),(-3,0,-1),(-1,0,-1),(-1,0,-3)], k=[0,1,2,3,4,5,6,7,8,9,10,11,12])
	mc.scale(control, .33, .33, .33)
	mc.makeIdentity(control, apply=True, t=True, r=True, s=True)

	
def create_single_arrow(prefix):
	global control
	control = mc.curve(d=1, p=[(0, 1.003235, 0),(0.668823, 0, 0),(0.334412, 0, 0),(0.334412, -0.167206, 0),(0.334412, -0.501617, 0),(0.334412, -1.003235, 0),(-0.334412, -1.003235, 0),(-0.334412, -0.501617, 0),(-0.334412, -0.167206, 0),(-0.334412, 0, 0),(-0.668823, 0, 0),(0, 1.003235, 0)], k=[0,1,2,3,4,5,6,7,8,9,10,11])


def create_curved_single_arrow(prefix):
	global control
	control = mc.curve(d=1, p=[(-0.251045, 0, 1.015808),(-0.761834, 0, 0.979696),(-0.486547, 0, 0.930468),(-0.570736, 0, 0.886448),(-0.72786, 0, 0.774834),(-0.909301, 0, 0.550655),(-1.023899, 0, 0.285854),(-1.063053, 0, 9.80765e-009),(-0.961797, 0, 8.87346e-009),(-0.926399, 0, 0.258619),(-0.822676, 0, 0.498232),(-0.658578, 0, 0.701014),(-0.516355, 0, 0.802034),(-0.440202, 0, 0.841857),(-0.498915, 0, 0.567734),(-0.251045, 0, 1.015808),], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15])
	mc.xform(cp=True)


def create_double_arrow(prefix):
	global control
	control = mc.curve(d=1, p=[(0, 1, 0),(1, 1, 0),(2, 1, 0),(3, 1, 0),(3, 2, 0),(4, 1, 0),(5, 0, 0),(4, -1, 0),(3, -2, 0),(3, -1, 0),(2, -1, 0),(1, -1, 0),(0, -1, 0),(-1, -1, 0),(-2, -1, 0),(-3, -1, 0),(-3, -2, 0),(-4, -1, 0),(-5, 0, 0),(-4, 1, 0),(-3, 2, 0),(-3, 1, 0),(-2, 1, 0),(-1, 1, 0),(0, 1, 0),], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24])
	mc.xform(cp=True)
	mc.scale(.2,.2,.2)
	mc.makeIdentity(apply=True, t=True, r=True, s=True)

 
def create_curved_double_arrow(prefix):
	global control
	control = mc.curve(d=1, p=[(-0.251045, 0, -1.015808), (-0.761834, 0, -0.979696), (-0.486547, 0, -0.930468), (-0.570736, 0, -0.886448), (-0.72786, 0, -0.774834), (-0.909301, 0, -0.550655), (-1.023899, 0, -0.285854), (-1.063053, 0, 9.80765e-009), (-1.023899, 0, 0.285854), (-0.909301, 0, 0.550655), (-0.72786, 0, 0.774834), (-0.570736, 0, 0.886448), (-0.486547, 0, 0.930468), (-0.761834, 0, 0.979696), (-0.251045, 0, 1.015808), (-0.498915, 0, 0.567734), (-0.440202, 0, 0.841857), (-0.516355, 0, 0.802034), (-0.658578, 0, 0.701014), (-0.822676, 0, 0.498232), (-0.926399, 0, 0.258619), (-0.961797, 0, 8.87346e-009), (-0.926399, 0, -0.258619), (-0.822676, 0, -0.498232), (-0.658578, 0, -0.701014), (-0.516355, 0, -0.802034), (-0.440202, 0, -0.841857), (-0.498915, 0, -0.567734), (-0.251045, 0, -1.015808)], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28])
	mc.makeIdentity(apply=True, t=True, r=True, s=True)
	mc.xform(cp=True)


def create_triple_arrow(prefix):
	global control
	control = mc.curve(d=1, p=[(-1, 1, 0),(-3, 1, 0),(-3, 2, 0),(-5, 0, 0),(-3, -2, 0),(-3, -1, 0),(-1, -1, 0),(1, -1, 0),(3, -1, 0),(3, -2, 0),(5, 0, 0),(3, 2, 0),(3, 1, 0),(1, 1, 0),(1, 3, 0),(2, 3, 0),(0, 5, 0),(-2, 3, 0),(-1, 3, 0),(-1, 1, 0),], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19])    	
	mc.xform(cp=True)
	mc.xform(t=[0,-1.5,0])
	mc.scale(.2,.2,.2)
	mc.makeIdentity(apply=True, t=True, r=True, s=True)


def create_quad_arrow(prefix):
	global control
	control = mc.curve(d=1, p=[(1, 0, 1),(3, 0, 1),(3, 0, 2),(5, 0, 0),(3, 0, -2),(3, 0, -1),(1, 0, -1),(1, 0, -3),(2, 0, -3),(0, 0, -5),(-2, 0, -3),(-1, 0, -3),(-1, 0, -1),(-3, 0, -1),(-3, 0, -2),(-5, 0, 0),(-3, 0, 2),(-3, 0, 1),(-1, 0, 1),(-1, 0, 3),(-2, 0, 3),(0, 0, 5),( 2, 0, 3),(1, 0, 3),(1, 0, 1),], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24])
	mc.xform(cp=True)
	mc.scale(.2,.2,.2)
	mc.makeIdentity(apply=True, t=True, r=True, s=True)

    
def create_cube(prefix):
	global control
	control = mc.curve(d=1, p=[(1, 1, 1),(1, 1, -1),(-1, 1, -1),(-1, 1, 1),(1, 1, 1),(1, -1, 1),(1, -1, -1),(1, 1, -1),(-1, 1, -1),(-1, -1, -1),(1, -1, -1),(-1, -1, -1),(-1, -1, 1),(-1, 1, 1),(-1, -1, 1),(1, -1, 1)], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15],  n = prefix + '_CTR')
	return control
	
def create_diamond(prefix):
	global control
	control = mc.curve(d=1, p=[(0, 1, 0),(-1, 0.00278996, 6.18172e-08),(0, 0, 1),(0, 1, 0),(1, 0.00278996, 0),(0, 0, 1),(1, 0.00278996, 0),(0, 0, -1),(0, 1, 0),(0, 0, -1),(-1, 0.00278996, 6.18172e-08),(0, -1, 0),(0, 0 ,-1),(1, 0.00278996, 0),(0, -1, 0),(0, 0, 1)],k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15],  n = prefix + '_CTR')
	return control

def create_cone(prefix):
	global control
	control = mc.curve(d=1, p=[(-0.5, -1, 0.866025),(0, 1, 0),(0.5, -1, 0.866025),(-0.5, -1, 0.866025),(-1, -1, -1.5885e-07),(0, 1, 0),(-1, -1, -1.5885e-07),(-0.5, -1, -0.866026),(0, 1, 0),(0.5, -1, -0.866025),(-0.5, -1, -0.866026),(0.5, -1, -0.866025),(0, 1, 0),(1, -1, 0), (0.5, -1, -0.866025),(1, -1, 0),(0.5, -1, 0.866025)  ], k=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])


def create_orb(prefix):
	global control
	control = mc.circle(nr=[0,1,0], n = prefix + '_CTR')[0]
	
	circle_list = []
	circle_list.append(mc.duplicate(rr=True))
	mc.xform(ro=[90,0,0])

	circle_list.append(mc.duplicate(rr=True))
	mc.xform(ro=[90,90,0])
	
	circle_list.append(mc.duplicate(rr=True))
	mc.xform(ro=[90,45,0])
	
	circle_list.append(mc.duplicate(rr=True))
	mc.xform(ro=[90,-45,0])

	for each in circle_list:
		mc.select(each)
		mc.makeIdentity(apply=True, t=True, r=True, s=True)
		mc.pickWalk(d='down')
		mc.select(control, tgl=True)
		mc.parent(r=True, s=True)
		mc.delete(each)

	mc.xform(control, cp=True)

	return control

	
def create_lever(prefix):
	line = mc.curve(d=1, p=[(0, -1, 0),(0, -2, 0 ),(0, -3, 0 ),(0, -4, 0),(0, -5, 0)], k=[0,1,2,3,4])
	create_orb()

	mc.select(line, r=True)
	mc.pickWalk(d='down')
	mc.select(control, tgl=True)
	mc.parent(r=True, s=True)

	mc.delete(line)
	mc.xform(control, rp=[0,-5,0], sp=[0,-5,0])
	mc.xform(control, t=[0,5,0])
	mc.scale(.2,.2,.2)
	mc.makeIdentity(control, apply=True, t=True, r=True, s=True)

