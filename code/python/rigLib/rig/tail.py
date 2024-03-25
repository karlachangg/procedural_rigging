"""
tail FK/IK @ rig
"""

import maya.cmds as mc

from . import bendyLimb
from . import fkChain

from ..base import module
from ..base import control

from ..utils import joint
from ..utils import name

class Tail():

    def __init__(self,
            tailJoints,
            prefix = 'tail',
            bendy = False,
            smallestScalePercent = 0.5,
            rigScale = 1.0,
            baseRig = None,
            ):
        """
        :param armJoints: list(str), shoulder - elbow - wrist
        :param scapulaJoint: str, scapula position joint
        :param prefix: str, prefix to name new objects
        :param rigScale: float, scale factor for size of controls
        :param baseRig: instance of base.module.Base class
        :return: dictionary with rig module objects
        """
        self.tailJoints = tailJoints
        self.prefix = prefix
        self.bendy = bendy
        self.smallestScalePercent = smallestScalePercent
        self.rigScale = rigScale
        self.baseRig = baseRig

        # make rig module

        self.rigmodule = module.Module(prefix = self.prefix, baseObj = self.baseRig)

        self.rigParts = {'bodyAttachGrp': ''}

    def build(self):

        # Make FK rig
        fkRig = self.buildFK()

        # Define rigParts properties
        self.rigParts['fkControls'] = fkRig['controls']

        # Create body attach group
        bodyAttachGrp = mc.group(n='{}_bodyAttachGrp'.format(self.prefix), em=1)
        mc.delete(mc.parentConstraint(self.tailJoints[0], bodyAttachGrp, mo = 0))

        mc.parentConstraint(bodyAttachGrp, fkRig['fkBodyAttach'], mo = 1)

        # rigParts dictionary
        self.rigParts = {
            'bodyAttachGrp': bodyAttachGrp
        }


    def buildFK(self):

        # duplicate tail joints to make FK joints
        fkJoints = joint.duplicateChain(self.tailJoints, 'jnt', 'FK_jnt')
        mc.parent(fkJoints[0], self.rigmodule.jointsGrp)

        # make controls

        tailFKRig = fkChain.build(fkJoints[:-1], rigScale=self.rigScale * 1.5, parent = self.rigmodule.controlsGrp,
                                       smallestScalePercent= 0.25, offsets=['null', 'zero', 'auto'])

        tailBaseCtr = tailFKRig['controls'][0]

        # Create body attach group
        bodyAttachGrp_FK = mc.group(n='{}_FK_bodyAttachGrp'.format(self.prefix), em=1)
        bodyAttachOrient = mc.group(n='{}_bodyAttachOrient'.format(self.prefix), em=1)
        hipsOrient = mc.group(n='{}_hipsOrient'.format(self.prefix), em=1)
        worldOrient = mc.group(n='{}_worldOrient'.format(self.prefix), em=1)

        mc.parent(bodyAttachOrient, bodyAttachGrp_FK)
        mc.parent(hipsOrient, bodyAttachGrp_FK)
        mc.parent(worldOrient, self.rigmodule.partsGrp)
        mc.parent(bodyAttachGrp_FK, self.rigmodule.partsGrp)

        # Position bodyAttachGroup at tail base
        mc.delete(mc.parentConstraint(tailBaseCtr.C, bodyAttachGrp_FK, mo=0))

        # Position worldOrient group to match tail base starting orientation
        mc.delete(mc.parentConstraint(tailBaseCtr.C, worldOrient, mo=0))

        # Parent constrain tail base to body attach orient group
        mc.parentConstraint(bodyAttachOrient, tailBaseCtr.Off, mo=1)

        # Set up tail orientation space switch
        attr = "Tail_Orient"
        mc.addAttr(tailBaseCtr.C, ln=attr, at='enum', enumName='hips:world', k=1)
        mc.setAttr('{}.{}'.format(tailBaseCtr.C, attr), cb=1)

        # orient constraint head control
        tailOrientConstraint = mc.orientConstraint(hipsOrient, worldOrient, bodyAttachOrient, mo=1)[0]
        weights = mc.orientConstraint(tailOrientConstraint, q=1, weightAliasList=1)

        # set driven key for hips orient
        mc.setAttr('{}.{}'.format(tailBaseCtr.C, attr), 0)
        mc.setAttr('{}.{}'.format(tailOrientConstraint, weights[0]), 1)
        mc.setAttr('{}.{}'.format(tailOrientConstraint, weights[1]), 0)

        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(tailOrientConstraint, weights[i]),
                                 cd='{}.{}'.format(tailBaseCtr.C, attr))

        # set driven key for global orient
        mc.setAttr('{}.{}'.format(tailBaseCtr.C, attr), 1)
        mc.setAttr('{}.{}'.format(tailOrientConstraint, weights[0]), 0)
        mc.setAttr('{}.{}'.format(tailOrientConstraint, weights[1]), 1)

        for i in range(len(weights)):
            mc.setDrivenKeyframe('{}.{}'.format(tailOrientConstraint, weights[i]),
                                 cd='{}.{}'.format(tailBaseCtr.C, attr))

        # set default to follow hips
        mc.setAttr('{}.{}'.format(tailBaseCtr.C, attr), 0)


        return {'joints': fkJoints, 'controls': tailFKRig['controls'], 'fkBodyAttach': bodyAttachGrp_FK}
