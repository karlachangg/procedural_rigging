"""
rhino rig setup
main module
"""


from . import quadruped
from . import project

from rigLib.rig import fkChain

import maya.cmds as mc

class Rhino(quadruped.Quadruped):

    def __init__(self, characterName, sceneScale):
        quadruped.Quadruped.__init__(self, characterName, sceneScale)

    def build(self):

        self.setup()
        self.makeControlSetup(self.baseRig)
        self.buildExtraControls()
        self.deform()

    def buildExtraControls(self):


        # Left ear

        earJoints = ['l_ear1_jnt', 'l_ear2_jnt']


        leftEarFKRig = fkChain.build(earJoints, rigScale = self.sceneScale, parent = self.neckRig.rigParts['headCtr'].C,
                                  offsets=['null', 'zero', 'auto'])


