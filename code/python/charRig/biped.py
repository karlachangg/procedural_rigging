"""
biped rig setup
main module
"""

#from rigLib.base import control
#from rigLib.base import module

#from . import project
#from . import char_deform
from . import char

from rigLib.rig import spine
from rigLib.rig import neck
from rigLib.rig import ikChain
from rigLib.rig import legFKIK

from rigLib.utils import joint

import maya.cmds as mc


class Biped(char.Character):

    def __init__(self, characterName):
        char.Character.__init__(self, characterName)

    def build(self):

        self.setup()
        self.deform()
        self.makeControlSetup(self.baseRig)


    def makeControlSetup(self, baseRig):
        """
           make control setup
           """

        # spine
        spineJoints = ['spine_0_jnt', 'spine_1_jnt', 'spine_2_jnt', 'spine_3_jnt', 'spine_4_jnt', 'spine_5_jnt',
                       'spine_6_jnt']
        rootJoint = 'root_jnt'
        chestJoint = 'chest_jnt'

        spineRig = spine.Spine(
            spineJoints = spineJoints,
            rootJnt = rootJoint,
            chestJnt = chestJoint,
            spineCurve='spine_curve',
            prefix = 'spine',
            rigScale = self.sceneScale,
            baseRig = baseRig
        )

        spineRig.build()


        # neck
        neckJoints = ['neck_1_jnt', 'neck_2_jnt', 'neck_3_jnt', 'neck_4_jnt']
        headJoint = 'head_jnt'

        neckRig = neck.build(
            neckJoints=neckJoints,
            headJnt=headJoint,
            neckCurve='neck_curve',
            prefix=  'neck',
            rigScale=self.sceneScale,
            baseRig=baseRig
        )

        # attach neck to spine
        mc.parentConstraint(spineRig.rigParts['chestAttachGrp'], neckRig['baseAttachGrp'], mo=1)

        # left leg
        legJoints = ['hip_jnt', 'knee_jnt', 'ankle_jnt']
        toeJoint = 'toe_jnt'
        hipPivotJoint = 'hip_piv_jnt'

        leftLegRig = legFKIK.Leg(
            legJoints = legJoints,
            toeJoint = toeJoint,
            hipPivotJoint = hipPivotJoint,
            pvLocator = '',
            prefix = 'leg',
            side = 'l',
            rigScale = self.sceneScale,
            baseRig = baseRig
            )

        leftLegRig.build()

        rightLegRig = legFKIK.Leg(
            legJoints = legJoints,
            toeJoint = toeJoint,
            hipPivotJoint = hipPivotJoint,
            pvLocator = '',
            prefix = 'leg',
            side = 'r',
            rigScale = self.sceneScale,
            baseRig = baseRig
            )

        rightLegRig.build()














