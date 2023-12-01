"""
ikChain @ rig


# if i want to make a "spine" chain (tail) this is how to call it:
tailJoints = joint.listHierarchy('tail0_jnt')

chainRig = ikChain.build(
                        chainJoints=tailJoints,
                        chainCurve= 'tail_curve',
                        prefix = 'tail',
                        rigScale= self.sceneScale,
                        smallestScalePercent= 0.4,
                        fkParenting= True,
                        baseRig = baseRig
                        )
# attach the chain (tail) to the pelvis

"""

import maya.cmds as mc

from ..base import module
from ..base import control

def build(
        chainJoints,
        chainCurve,
        prefix = 'tail',
        rigScale = 1.0,
        smallestScalePercent = 0.5,
        fkParenting = True,
        baseRig = None
        ):
    """

    :param chainJoints: list (str), list of chain joints
    :param chainCurve: str, name of chain cubic curve
    :param prefix: str, prefix to name new objects
    :param rigScale: float, scale factor for size of controls
    :param smallestScalePercent: float, scale of smallest control at the end of the chain compared to rig scale
    :param fkParenting: bool, parent each control to the previous one to make an FK chain
    :param baseRig: instance of base.module.Base class
    :return: dictionary with rig module objects
    """

    # make rig module

    rigmodule = module.Module(prefix=prefix, baseObj=baseRig)

    # make locators to drive spine curve clusters

    chainCurveCVs = mc.ls('{}.cv[*]'.format(chainCurve), fl=1)
    numChainCVs = len(chainCurveCVs)
    chainCurveShape = mc.listRelatives(chainCurve, ad=True, shapes=True)[0]
    driverLocators = []
    driverLocatorOffsets = []
    locGrp = mc.group(n='{}_cv_drivers'.format(prefix), em=True)

    for i in range(numChainCVs):
        cvXform = mc.xform(chainCurveCVs[i], q=True, t=True)
        loc = mc.spaceLocator(n='{}_cv{}_loc'.format(prefix, i + 1))[0]
        driverLocators.append(loc)
        # get locator shape node
        locShape = mc.listRelatives(loc, ad=1, shapes=True)[0]
        offset = mc.group(n='{}_cv{}_loc_offset'.format(prefix, i + 1))
        driverLocatorOffsets.append(offset)
        mc.parent(offset, locGrp)
        mc.xform(offset, t=cvXform)
        mc.connectAttr('{}.worldPosition[0]'.format(locShape), chainCurveShape + '.controlPoints[%d]' % (i))

    mc.parent(locGrp, rigmodule.partsGrp)
    mc.select(cl=1)

    # make attach groups
    baseAttachGrp = mc.group(n= '{}_BaseAttach_grp'.format(prefix), em=1, p=rigmodule.partsGrp)
    mc.delete(mc.pointConstraint(chainJoints[0], baseAttachGrp))

    # make controls

    chainControls = []
    controlScaleIncrement = (1.0 - smallestScalePercent) / numChainCVs
    mainCtrScaleFactor = 5.0

    for i in range(numChainCVs):

        ctrScale = rigScale * mainCtrScaleFactor * (1.0 - (i * controlScaleIncrement))
        ctr = control.Control(prefix = '{}_{}'.format(prefix, i+1), translateTo=driverLocators[i],
                              scale = ctrScale, parent = rigmodule.controlsGrp, shape = 'sphere')
        chainControls.append(ctr)

    # parent controls

    if fkParenting:

        for i in range(len(numChainCVs)):

            if i==0:
                continue

            mc.parent(chainControls[i].Off, chainControls[i-1].C )

    # attach driver locators

    for i in range(numChainCVs):
        mc.parentConstraint(chainControls[i].C, driverLocatorOffsets[i], mo=1)

    # attach controls
    mc.parentConstraint(baseAttachGrp, chainControls[0].Off, mo=1)

    # make IK handle

    chainIk = mc.ikHandle(n='{}_ikHandle'.format(prefix), sol='ikSplineSolver', sj=chainJoints[0], ee=chainJoints[-1],
                          c=chainCurve, ccv=0, parentCurve=0)[0]
    mc.hide(chainIk)
    mc.parent(chainIk, rigmodule.noXformGrp)
    mc.parent(chainCurve, rigmodule.noXformGrp)

    # add twist attribute
    twistAttr = 'twist'
    mc.addAttr(chainControls[-1].C, ln = twistAttr, k=1)
    mc.connectAttr('{}.{}'.format(chainControls[-1].C, twistAttr), '{}.twist'.format(chainIk))

    return {'module': rigmodule, 'baseAttachGrp': baseAttachGrp}