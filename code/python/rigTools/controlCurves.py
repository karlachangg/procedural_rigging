'''

import maya.cmds as mc

def saveControlCurves(filepath):

    # Get all control curves in scene

    controlCurves = mc.controller(ac = 1, q = 1)

    # Get control curve info and save in dictionary
    controlCurvesDict = {}

    # Save out to json


def loadControlCurves(filepath):

    # Load json from given filepath and save as a dictionary

    # For each control shape, delete the original shape in scene. Recreate it with given info, and parent under transform

'''

import json
import logging
import maya.cmds as cmds
import maya.api.OpenMaya as OpenMaya

logger = logging.getLogger(__name__)

def export_curves(controls=None, file_path=None):
    """Serializes the given curves into the control library.

    :param controls: Optional list of controls to export. If no controls are specified,
        the selected curves will be exported.
    :param file_path: File path to export to
    :return: The exported list of ControlShapes.
    """
    if controls is None:
        raise Exception("No control objects found in scene.")
    elif not controls:
        raise Exception("No control objects found in scene.")

    data = get_curve_data(controls)

    with open(file_path, "w") as fh:
        json.dump(data, fh, indent=4)
        logger.info("Exported controls to {}".format(file_path))
    return data

def import_curves(file_path=None, tag_as_controller=False):
    """Imports control shapes from disk onto their saved named transforms.

    :param file_path: Path to the control file.
    :param tag_as_controller: True to tag the curve transform as a controller
    :return: The new curve transforms
    """
    controls = load_curves(file_path)

    transforms = [
        curve.create(curve.transform, tag_as_controller) for curve in controls
    ]
    return transforms

def mirror_curves(controls = None, mirrorAxis = 'x'):
    # get left side controls
    leftControls = []

    if mirrorAxis == 'x':
        x, y, z = -1, 1, 1
    elif mirrorAxis == 'y':
        x, y, z = 1, -1, 1
    elif mirrorAxis == 'z':
        x, y, z = 1, 1, -1


    for control in controls:
        if control[0:2] == 'l_':
            leftControls.append(control)

    # Create temp controls to mirror
    temp_controls = []
    temp_grp = cmds.group(n='mirror_temp', em = 1)
    for control in leftControls:
        temp_control = cmds.duplicate(control, n = 'temp_' + control, rc = 1, rr = 1)[0]
        cmds.setAttr(temp_control + '.tx', lock=0)
        cmds.setAttr(temp_control + '.ty', lock=0)
        cmds.setAttr(temp_control + '.tz', lock=0)
        cmds.setAttr(temp_control + '.rx', lock=0)
        cmds.setAttr(temp_control + '.ry', lock=0)
        cmds.setAttr(temp_control + '.rz', lock=0)
        cmds.setAttr(temp_control + '.sx', lock=0)
        cmds.setAttr(temp_control + '.sy', lock=0)
        cmds.setAttr(temp_control + '.sz', lock=0)

        temp_controls.append(temp_control)

        cmds.delete(cmds.listRelatives(temp_control, c = 1, typ = 'transform'))
        cmds.parent(temp_control, temp_grp)

    cmds.scale(-1, 1, 1, temp_grp, r=1)
    cmds.makeIdentity(temp_grp, apply = True)




    for i in range(len(temp_controls)):

        rightControl = 'r_' + leftControls[i][2:]
        rightShapes = get_shapes(rightControl, intermediate=False)
        right_color = get_color(rightShapes[0])
        cmds.delete(rightShapes)


        # Get the temp control shapes
        shapes = get_shapes(temp_controls[i], intermediate=False)


        cmds.parent(temp_controls[i], rightControl)
        cmds.makeIdentity(temp_controls[i], apply = True)
        cmds.parent(temp_controls[i], temp_grp)



        for shape in shapes:
            CurveShapeObject = CurveShape(transform = temp_controls[i], shape = shape)
            #CurveShapeObject.scale_by(-1, -1, -1, local=True)
            #newTransform = 'new_' + controlw
            rightShape = 'r_' + shape[2:]
            MirrorCurve = CurveShape(transform = rightControl,
                                       shape = rightShape,
                                       cvs=  CurveShapeObject.cvs,
                                       degree= CurveShapeObject.degree,
                                       form= CurveShapeObject.form,
                                       knots=CurveShapeObject.knots,
                                       color = right_color)
            #MirrorCurve.scale_by(x, y, z, local=True )
            MirrorCurve.create(relative=True)


    cmds.delete(temp_grp)







    # create right side curves

def get_curve_data(controls=None):
    """Get the serializable data of the given controls.

    :param controls: Controls to serialize
    :return: List of ControlShape objects
    """
    if controls is None:
        raise Exception("No control objects found in scene.")
    elif not controls:
        raise Exception("No control objects found in scene.")

    #data = [CurveShape(transform=control) for control in controls]
    data = {}


    for control in controls:


        # Get shape nodes under control
        shapes = get_shapes(control, intermediate= False)
        data[control] = {}

        for shape in shapes:
            data[control][shape] = {}
            CurveShapeObject = CurveShape(transform = control, shape = shape)

            #print(CurveShapeObject.cvs)


            data[control][shape] = {
                'cvs' : CurveShapeObject.cvs,
                'degree' : CurveShapeObject.degree,
                'form' : CurveShapeObject.form,
                'knots': CurveShapeObject.knots,
                'color': CurveShapeObject.color
            }

    # Prune empty curves
    #data = [x for x in data if x.cvs]
    return data

def load_curves(file_path=None):
    """Load the CurveShape objects from disk.

    :param file_path:
    :return:
    """

    with open(file_path, "r") as fh:
        data = json.load(fh)

    logger.info("Loaded controls {}".format(file_path))
    curves = []

    for control in data:

        # First wipe out any shapes under existing controls

        if cmds.objExists(control):
            existingShapes = get_shapes(control, intermediate=False)
            for shape in existingShapes:
                cmds.delete(shape)

        # Get shapes from json to create
        shapes = list(data[control].keys())

        for shape in shapes:

            curveShapeObj = CurveShape(transform= control,
                                       shape = shape,
                     cvs= data[control][shape]['cvs'],
                     degree= data[control][shape]['degree'],
                     form= data[control][shape]['form'],
                     knots= data[control][shape]['knots'],
                     color= data[control][shape]['color'])

            curves.append(curveShapeObj)
    #curves = [CurveShape(control) for control in data]

    return curves

def get_shapes(node, intermediate=False):
    """Get the shape node of a transform

    This is useful if you don't want to have to check if a node is a shape node
    or transform.  You can pass in a shape node or transform and the function
    will return the shape node.

    :param node:  node The name of the node.
    :param intermediate:  intermediate True to get the intermediate shape
    :return: The name of the shape node.
    """
    shapes = []
    if cmds.objectType(node, isAType="transform"):

        nodeShapes = cmds.listRelatives(node, shapes=True, path=True)

        # return empty list of no shape nodes are found
        if not nodeShapes:
            return shapes

        for shape in nodeShapes:
            is_intermediate = cmds.getAttr("{}.intermediateObject".format(shape))
            # if we want to get intermediate shapes, return list only if the shape is intermediate
            if (
                intermediate
                and is_intermediate
                and cmds.listConnections(shape, source=False)
            ):
                shapes.append(shape)


            elif not intermediate and not is_intermediate:
                shapes.append(shape)

        if nodeShapes:
            return shapes

    elif cmds.nodeType(node) in ["mesh", "nurbsCurve", "nurbsSurface"]:
        is_intermediate = cmds.getAttr("{}.intermediateObject".format(node))
        if is_intermediate and not intermediate:
            node = cmds.listRelatives(node, parent=True, path=True)[0]
            return get_shapes(node)
        else:
            return node
    return None

def get_knots(curve):
    """Gets the list of knots of a curve so it can be recreated.

    :param curve: Curve to query.
    :return: A list of knot values that can be passed into the curve creation command.
    """
    curve = get_shapes(curve)
    info = cmds.createNode("curveInfo")
    cmds.connectAttr("{0}.worldSpace".format(curve), "{0}.inputCurve".format(info))
    knots = cmds.getAttr("{0}.knots[*]".format(info))
    knots = [int(x) for x in knots]
    cmds.delete(info)
    return knots

def get_color(curve):

    if cmds.getAttr("{}.overrideEnabled".format(curve)):
        if cmds.getAttr("{}.overrideRGBColors".format(curve)):
            color = cmds.getAttr("{}.overrideColorRGB".format(curve))[0]
        else:
            color = cmds.getAttr("{}.overrideColor".format(curve))
    else:
        color = None
    return color


class CurveShape():
    """Represents the data required to build a nurbs curve shape"""

    def __init__(self,
                 transform=None,
                 shape = None,
                 cvs=None,
                 degree=3,
                 form=0,
                 knots=None,
                 color=None):

        self.cvs = cvs
        self.degree = degree
        self.form = form
        self.knots = knots
        self.color = color
        self.transform_matrix = OpenMaya.MTransformationMatrix()
        self.transform = transform
        self.shape = shape
        if transform and cmds.objExists(transform) and not cvs:
            self._set_from_curve(transform)
            #self.transform_xform = cmds.xform(control)

    def _set_from_curve(self, transform):
        """Store the parameters from an existing curve in the CurveShape object.

        :param transform: Transform
        """
        #shape = get_shapes(transform)
        #print(shape)

        if self.shape and cmds.nodeType(self.shape) == "nurbsCurve":
            create_attr = "{}.create".format(self.shape)
            connection = cmds.listConnections(create_attr, plugs=True, d=False)
            if connection:
                cmds.disconnectAttr(connection[0], create_attr)
            self.transform = transform
            self.cvs = cmds.getAttr("{}.cv[*]".format(self.shape))
            self.degree = cmds.getAttr("{}.degree".format(self.shape))
            self.form = cmds.getAttr("{}.form".format(self.shape))
            self.knots = get_knots(self.shape)
            self.color = get_color(self.shape)

            if connection:
                cmds.connectAttr(connection[0], create_attr)

    def create(self, transform=None, as_controller=True, relative = True):
        """Create a curve.

        :param transform: Name of the transform to create the curve shape under.
            If the transform does not exist, it will be created.
        :param as_controller: True to mark the curve transform as a controller.
        :return: The transform of the new curve shapes.
        """
        transform = transform or self.transform
        if not cmds.objExists(transform):
            transform = cmds.createNode("transform", name=transform)
        # if shape node exists, delete it
        if cmds.objExists(self.shape):
            cmds.delete(self.shape)

        periodic = self.form == 2
        points = self._get_transformed_points()
        points = points + points[: self.degree] if periodic else points

        curve = cmds.curve(degree=self.degree, p=points, per=periodic, k=self.knots)
        shape = get_shapes(curve)[0]
        if self.color is not None:
            cmds.setAttr("{}.overrideEnabled".format(shape), True)
            if isinstance(self.color, int):
                cmds.setAttr("{}.overrideColor".format(shape), self.color)
            else:
                cmds.setAttr("{}.overrideRGBColors".format(shape), True)
                cmds.setAttr("{}.overrideColorRGB".format(shape), *self.color)
        cmds.parent(shape, transform, r=relative, s=True)
        shape = cmds.rename(shape, self.shape)
        cmds.delete(curve)
        if as_controller:
            cmds.controller(transform)
        logger.info("Created curve {} for transform {}".format(shape, transform))
        return transform

    def _get_transformed_points(self):
        matrix = self.transform_matrix.asMatrix()
        #print(self.cvs)
        points = [OpenMaya.MPoint(*x) * matrix for x in self.cvs]
        points = [(p.x, p.y, p.z) for p in points]
        return points

    def translate_by(self, x, y, z, local=True):
        """Translate the curve cvs by the given values

        :param x: Translate X
        :param y: Translate Y
        :param z: Translate Z
        :param local: True for local space, False for world
        """
        space = OpenMaya.MSpace.kObject if local else OpenMaya.MSpace.kWorld
        self.transform_matrix.translateBy(OpenMaya.MVector(x, y, z), space)

    def set_translation(self, x, y, z, local=True):
        """Set the absolute translation of the curve shape.

        :param x: Translate X
        :param y: Translate Y
        :param z: Translate Z
        :param local: True for local space, False for world
        """
        space = OpenMaya.MSpace.kObject if local else OpenMaya.MSpace.kWorld
        self.transform_matrix.setTranslation(OpenMaya.MVector(x, y, z), space)

    def rotate_by(self, x, y, z, local=True):
        """Rotate the curve cvs by the given euler rotation values

        :param x: Rotate X
        :param y: Rotate Y
        :param z: Rotate Z
        :param local: True for local space, False for world
        """
        x, y, z = [v * 0.0174533 for v in [x, y, z]]
        space = OpenMaya.MSpace.kObject if local else OpenMaya.MSpace.kWorld
        self.transform_matrix.rotateBy(OpenMaya.MEulerRotation(x, y, z), space)

    def set_rotation(self, x, y, z):
        """Set the absolute rotation of the curve shape in euler rotations.

        :param x: Rotate X
        :param y: Rotate Y
        :param z: Rotate Z
        """
        x, y, z = [v * 0.0174533 for v in [x, y, z]]
        self.transform_matrix.setRotation(OpenMaya.MEulerRotation(x, y, z))

    def scale_by(self, x, y, z, local=True):
        """Scale the curve cvs by the given amount.

        :param x: Scale X
        :param y: Scale Y
        :param z: Scale Z
        :param local: True for local space, False for world
        """

        space = OpenMaya.MSpace.kObject if local else OpenMaya.MSpace.kWorld
        self.transform_matrix.scaleBy([x, y, z], space)

    def mirror(self, x, y, z ):
        cmds.select(self.transform)
        tempGrp = cmds.group(n = 'mirror_temp')
        cmds.scale(-1, 1, 1, tempGrp, r = 1 )
        cmds.parent(self.transform, w = 1)

    def set_scale(self, x, y, z, local=True):
        """Set the absolute scale of the curve shape.

        :param x: Scale X
        :param y: Scale Y
        :param z: Scale Z
        :param local: True for local space, False for world
        """
        space = OpenMaya.MSpace.kObject if local else OpenMaya.MSpace.kWorld
        self.transform_matrix.setScale([x, y, z], space)

def ExportCurves(file_path):

    # Get controls in scene, we are using the controller tag
    '''
    curve_transforms = [cmds.listRelatives(i, p=1, type='transform')[0] for i
    in cmds.ls(type='nurbsCurve', o=1, r=1, ni=1)]
    selectList = []

    for c in curve_transforms:
        selectList.append(c)
    '''
    controlCurves = cmds.controller(ac = 1, q = 1)

    #file_path = "/Users/karlachang/Documents/maya/biped/rig/build/curves.json"
    export_curves(controlCurves, file_path)

def ImportCurves(file_path):
    #file_path = "/Users/karlachang/Documents/maya/biped/rig/build/curves.json"
    import_curves(file_path, True)

#ExportCurves()
#ImportCurves()
def MirrorCurves_LeftToRight():

    controlCurves = cmds.controller(ac=1, q=1)
    mirror_curves(controlCurves)

def MirrorSelected_LeftToRight(mirrorAxis = 'x'):

    selected = cmds.ls(sl = 1)
    mirror_curves(selected, mirrorAxis)
