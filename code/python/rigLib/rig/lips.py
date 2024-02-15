import maya.cmds as mc
from ..base import control
from ..utils import name
from . import ribbon



def buildLips(lipDriverJoints, topCurve, topSurf, bottomCurve, bottomSurf, topBindJoints, bottomBindJoints, prefix='', aimAxis='x'):

    # Create major controls
    majorControls = buildControls(lipDriverJoints)

    # Create lip minor controls at top lip bind joints
    topTweakControls = buildTweakControls(topBindJoints, topCurve, topSurf, prefix = 'top')

    # Create lip minor controls at bottom lip bind joints
    bottomTweakControls = buildTweakControls(bottomBindJoints, bottomCurve, bottomSurf, prefix = 'bottom')

    # Drive tweak controls with major controls

    # top lip 1
    constraint = mc.parentConstraint(majorControls[0].C,  topTweakControls[0].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 2
    constraint = mc.parentConstraint(majorControls[0].C, majorControls[1].C, topTweakControls[1].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 3
    constraint = mc.parentConstraint(majorControls[1].C, topTweakControls[2].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 4
    constraint = mc.parentConstraint(majorControls[1].C, majorControls[2].C, topTweakControls[3].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 5
    constraint = mc.parentConstraint(majorControls[1].C, majorControls[2].C, topTweakControls[4].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 6
    constraint = mc.parentConstraint(majorControls[2].C, topTweakControls[5].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 7
    constraint = mc.parentConstraint(majorControls[2].C, majorControls[3].C, topTweakControls[6].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 8
    constraint = mc.parentConstraint(majorControls[2].C, majorControls[3].C, topTweakControls[7].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 9
    constraint = mc.parentConstraint(majorControls[3].C, topTweakControls[8].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 10
    constraint = mc.parentConstraint(majorControls[3].C, majorControls[4].C, topTweakControls[9].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # top lip 11
    constraint = mc.parentConstraint(majorControls[4].C, topTweakControls[10].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)




    # bottom lip 1
    constraint = mc.parentConstraint(majorControls[0].C, majorControls[5].C, bottomTweakControls[0].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 2
    constraint = mc.parentConstraint(majorControls[5].C, bottomTweakControls[1].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 3
    constraint = mc.parentConstraint(majorControls[5].C, majorControls[6].C, bottomTweakControls[2].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 4
    constraint = mc.parentConstraint(majorControls[5].C, majorControls[6].C, bottomTweakControls[3].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 5
    constraint = mc.parentConstraint(majorControls[6].C, bottomTweakControls[4].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 6
    constraint = mc.parentConstraint(majorControls[6].C, majorControls[7].C, bottomTweakControls[5].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 7
    constraint = mc.parentConstraint(majorControls[6].C, majorControls[7].C, bottomTweakControls[6].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 8
    constraint = mc.parentConstraint(majorControls[7].C, bottomTweakControls[7].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # bottom lip 9
    constraint = mc.parentConstraint(majorControls[7].C, majorControls[4].C, bottomTweakControls[8].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)




    # Bind curves and ribbons to the driver joints
    topLipDriverJoints = lipDriverJoints[0:5]
    bottomLipDriverJoints = [lipDriverJoints[0], lipDriverJoints[5], lipDriverJoints[6], lipDriverJoints[7], lipDriverJoints[4] ]

    mc.skinCluster(topLipDriverJoints, topCurve, bm = 0, mi = 2)
    mc.skinCluster(topLipDriverJoints, topSurf, bm=0, mi=2)

    mc.skinCluster(bottomLipDriverJoints, bottomCurve, bm=0, mi=2)
    mc.skinCluster(bottomLipDriverJoints, bottomSurf, bm=0, mi=2)



def buildControls(joints):


    # Create lip major controls at each lip driver
    controls = []
    ghostControls = []
    controlsGrp = mc.group(em = 1, n = 'LipControls')
    ghostControlsGrp = mc.group(em = 1, n = 'LipGhostControls')

    for driverJoint in joints:

        ctr = control.Control(prefix= name.removeSuffix(driverJoint), translateTo=driverJoint, rotateTo=driverJoint,
                               scale= 1.0, shape= 'cube', offsets= ['grp', 'zero', 'auto'], parent = controlsGrp,
                              color = 'yellow')

        controls.append(ctr)

        # Create a duplicate control hierarchy but delete shape node
        ghostCtr = control.Control(prefix= name.removeSuffix(driverJoint) + 'Ghost', translateTo=driverJoint, rotateTo=driverJoint,
                               scale= 1.0, shape= 'cube', offsets= ['grp', 'zero', 'auto'], parent = ghostControlsGrp)
        # delete control shape node
        mc.delete(mc.listRelatives(ghostCtr.C, c= 1, s = 1))

        ghostControls.append(ghostCtr)

        # direct connect t, r, and s from controls to ghost controls
        mc.connectAttr('{}.translate'.format(ctr.C), '{}.translate'.format(ghostCtr.C))
        mc.connectAttr('{}.rotate'.format(ctr.C), '{}.rotate'.format(ghostCtr.C))
        mc.connectAttr('{}.scale'.format(ctr.C), '{}.scale'.format(ghostCtr.C))

        # parent driver joints to ghost controls
        mc.parent(driverJoint, ghostCtr.C)

    # Connect lip major control hierarchy (drive middle controls with top, bottom and corners

    # right corner and middle top
    constraint = mc.parentConstraint(controls[0].C, controls[2].C, controls[1].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # middle top and left corner
    constraint = mc.parentConstraint(controls[2].C, controls[4].C, controls[3].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # right corner and middle bottom
    constraint = mc.parentConstraint(controls[0].C, controls[6].C, controls[5].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    # middle bottom and left corner
    constraint = mc.parentConstraint(controls[6].C, controls[4].C, controls[7].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)


    # Do exact same setup but on the duplicated setup
    constraint = mc.parentConstraint(ghostControls[0].C, ghostControls[2].C, ghostControls[1].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    constraint = mc.parentConstraint(ghostControls[2].C, ghostControls[4].C, ghostControls[3].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    constraint = mc.parentConstraint(ghostControls[0].C, ghostControls[6].C, ghostControls[5].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    constraint = mc.parentConstraint(ghostControls[6].C, ghostControls[4].C, ghostControls[7].Off, mo=1)[0]
    mc.setAttr('{}.interpType'.format(constraint), 2)

    return controls




def buildTweakControls(joints, curve, surf, prefix = '' ):

    # Create lip major controls at each lip driver
    controls = []
    ghostControls = []

    ghostControlDrivenOffsets = []

    controlsGrp = mc.group(em=1, n= prefix + 'LipTweakControls')
    ghostControlsGrp = mc.group(em=1, n= prefix + 'LipTweakGhostControls')

    for driverJoint in joints:
        ctr = control.Control(prefix=name.removeSuffix(driverJoint), translateTo=driverJoint, rotateTo=driverJoint,
                              scale=1.0, shape = 'diamond', offsets=['grp', 'zero', 'auto'], parent=controlsGrp,
                              color = 'cyan' )

        controls.append(ctr)


        # Create a duplicate control hierarchy but delete shape node
        ghostCtr = control.Control(prefix=name.removeSuffix(driverJoint) + 'Ghost', translateTo=driverJoint,
                                   rotateTo=driverJoint, scale = 0.5,
                                   offsets=['grp', 'zero', 'auto'], parent=ghostControlsGrp)
        # delete control shape node
        mc.delete(mc.listRelatives(ghostCtr.C, c=1, s=1))

        ghostControls.append(ghostCtr)
        ghostControlDrivenOffsets.append(ghostCtr.Off)


        # direct connect t, r, and s from controls to ghost controls
        mc.connectAttr('{}.translate'.format(ctr.C), '{}.translate'.format(ghostCtr.C))
        mc.connectAttr('{}.rotate'.format(ctr.C), '{}.rotate'.format(ghostCtr.C))
        mc.connectAttr('{}.scale'.format(ctr.C), '{}.scale'.format(ghostCtr.C))


        # direct connect t, r, and s from controls to ghost controls
        mc.connectAttr('{}.translate'.format(ctr.Offsets[1]), '{}.translate'.format(ghostCtr.Offsets[1]))
        mc.connectAttr('{}.rotate'.format(ctr.Offsets[1]), '{}.rotate'.format(ghostCtr.Offsets[1]))
        mc.connectAttr('{}.scale'.format(ctr.Offsets[1]), '{}.scale'.format(ghostCtr.Offsets[1]))

        # direct connect t, r, and s from controls to ghost controls
        mc.connectAttr('{}.translate'.format(ctr.Offsets[2]), '{}.translate'.format(ghostCtr.Offsets[2]))
        mc.connectAttr('{}.rotate'.format(ctr.Offsets[2]), '{}.rotate'.format(ghostCtr.Offsets[2]))
        mc.connectAttr('{}.scale'.format(ctr.Offsets[2]), '{}.scale'.format(ghostCtr.Offsets[2]))

        # parent driver joints to ghost controls
        mc.parent(driverJoint, ghostCtr.C)

    # Attach duplicated minor control setup offset groups to lip curves and lip ribbons using the motion path scripts
    ribbon.buildRibbon(curve, surf, ghostControlDrivenOffsets)




    return controls





