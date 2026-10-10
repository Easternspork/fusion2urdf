# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:17:17 2019
Modified on Sun Jan 17 2021

@author: syuntoku
@author: spacemaster85
"""

import adsk, re, traceback
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit):
        """
        Attributes
        ----------
        name: str
            name of the joint
        type: str
            type of the joint(ex: rev)
        xyz: [x, y, z]
            coordinate of the joint
        axis: [x, y, z]
            coordinate of axis of the joint
        parent: str
            parent link
        child: str
            child link
        joint_xml: str
            generated xml describing about the joint
        tran_xml: str
            generated xml describing about the transmission
        """
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis  # for 'revolute' and 'continuous'
        self.upper_limit = upper_limit  # for 'revolute' and 'prismatic'
        self.lower_limit = lower_limit  # for 'revolute' and 'prismatic'

    def make_joint_xml(self):
        """
        Generate the joint_xml and hold it by self.joint_xml
        """
        joint = Element('joint')
        joint.attrib = {'name':self.name, 'type':self.type}

        origin = SubElement(joint, 'origin')
        origin.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link':self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link':self.child}
        if self.type == 'revolute' or self.type == 'continuous' or self.type == 'prismatic':
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz':' '.join([str(_) for _ in self.axis])}
        if self.type == 'revolute' or self.type == 'prismatic':
            limit = SubElement(joint, 'limit')
            limit.attrib = {'upper': str(self.upper_limit), 'lower': str(self.lower_limit),
                            'effort': '100', 'velocity': '100'}

        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """
        Generate the tran_xml and hold it by self.tran_xml


        Notes
        -----------
        mechanicalTransmission: 1
        type: transmission interface/SimpleTransmission
        hardwareInterface: PositionJointInterface
        """

        tran = Element('transmission')
        tran.attrib = {'name':self.name + '_tran'}

        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'

        joint = SubElement(tran, 'joint')
        joint.attrib = {'name':self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'

        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name':self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'

        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg):
    """
    joints_dict holds parent, axis and xyz informatino of the joints


    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status

    Returns
    ----------
    joints_dict:
        {name: {type, axis, upper_limit, lower_limit, parent, child, xyz}}
    msg: str
        Tell the status
    """

    joint_type_list = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical',
    'PinSlot', 'Planner', 'Ball']  # these are the names in urdf

    joints_dict = {}
    skipped_joints = []

    # Collect joints from root and all sub-assemblies (sub-assembly joints are
    # not included in root.joints when defined inside a nested component).
    # Sub-assembly joints must be converted into the root assembly context
    # (proxy), otherwise their occurrences resolve to names local to the
    # sub-component that don't match any root-level link.
    # As-built joints are collected too. Hidden joints are not exported.
    all_joints = list(root.joints) + list(root.asBuiltJoints)
    for occ in root.allOccurrences:
        for j in list(occ.component.joints) + list(occ.component.asBuiltJoints):
            proxy = j.createForAssemblyContext(occ)
            all_joints.append(proxy if proxy else j)

    for joint in all_joints:
        if not joint.isLightBulbOn:
            skipped_joints.append('%s (joint hidden)' % joint.name)
        else:
            joint_dict = {}
            joint_type = joint_type_list[joint.jointMotion.jointType]
            joint_dict['type'] = joint_type
    
            # switch by the type of the joint
            joint_dict['axis'] = [0, 0, 0]
            joint_dict['upper_limit'] = 0.0
            joint_dict['lower_limit'] = 0.0
    
            # support  "Revolute", "Rigid" and "Slider"
            if joint_type == 'revolute':
                joint_dict['axis'] = [round(i, 6) for i in \
                    joint.jointMotion.rotationAxisVector.asArray()] ## In Fusion, exported axis is normalized.
                max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
                min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled
                if max_enabled and min_enabled:
                    joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue, 6)
                    joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue, 6)
                elif max_enabled and not min_enabled:
                    msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                    break
                elif not max_enabled and min_enabled:
                    msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                    break
                else:  # if there is no angle limit
                    joint_dict['type'] = 'continuous'
    
            elif joint_type == 'prismatic':
                joint_dict['axis'] = [round(i, 6) for i in \
                    joint.jointMotion.slideDirectionVector.asArray()]  # Also normalized
                max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
                min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled
                if max_enabled and min_enabled:
                    joint_dict['upper_limit'] = round(joint.jointMotion.slideLimits.maximumValue/100, 6)
                    joint_dict['lower_limit'] = round(joint.jointMotion.slideLimits.minimumValue/100, 6)
                elif max_enabled and not min_enabled:
                    msg = joint.name + 'is not set its lower limit. Please set it and try again.'
                    break
                elif not max_enabled and min_enabled:
                    msg = joint.name + 'is not set its upper limit. Please set it and try again.'
                    break
            elif joint_type == 'fixed':
                pass
            
    
            def get_parent(occ): 
            # function to find the root component of the joint. This is necessary for the correct component name in the urdf file
                if occ.assemblyContext != None:
                    #print(occ.name)
                    occ = get_parent(occ.assemblyContext)
                return occ
    
            if joint.occurrenceTwo != None and joint.occurrenceOne != None and joint.occurrenceOne.isLightBulbOn:
                parent_occ = utils.link_occurrence_for(joint.occurrenceTwo)
                joint_dict['parent'] = utils.link_name(parent_occ)
                if joint_dict['parent'] == 'base_link':
                    base_link = parent_occ
                child_occ = utils.link_occurrence_for(joint.occurrenceOne)
                joint_dict['child'] = utils.link_name(child_occ)
                if not parent_occ.isVisible or not child_occ.isVisible:
                    skipped_joints.append('%s (connects a hidden component)' % joint.name)
                    continue
                if joint_dict['parent'] == joint_dict['child']:
                    skipped_joints.append('%s (both sides in same link %s)' % (joint.name, joint_dict['child']))
                    continue
            else:
                reason = 'occ1=None' if joint.occurrenceOne is None else \
                         'occ2=None' if joint.occurrenceTwo is None else \
                         'occ1 hidden'
                skipped_joints.append('%s (%s)' % (joint.name, reason))
                continue
            
            def getJointOriginWorldCoordinates(joint :adsk.fusion.Joint):
            # Function to transform the joint origin coordinates which are in the component context into world coordinates
            # Thanky you for the help in the fusion forum
            # https://forums.autodesk.com/t5/fusion-360-api-and-scripts/how-to-get-the-joint-origin-in-world-context/m-p/10011971/highlight/false#M12401
                def getMatrixFromRoot(root_occ) -> adsk.core.Matrix3D:
                    mat = adsk.core.Matrix3D.create()
    
                    occ = adsk.fusion.Occurrence.cast(root_occ)
                    if not occ:
                        return mat # root

                    def getParentOccs(joint):
                        occs_list = []
            
                        if joint != None:    
                            occs_list.append(joint)
             
                        if joint.assemblyContext != None:
                            occs_list = occs_list + getParentOccs(joint.assemblyContext)
            
                        return occs_list
                    
                    occs = getParentOccs(root_occ)
                    mat3ds = [occ.transform for occ in occs if occ!= None]
                    #mat3ds.reverse()
                    for mat3d in mat3ds:
                        mat.transformBy(mat3d)
                    return mat
    
                # mat :adsk.core.Matrix3D = getMatrixFromRoot(joint.occurrenceOne)
                # ori1 :adsk.core.Point3D = joint.geometryOrOriginOne.origin.copy()
                # ori1.transformBy(mat)
    
                mat = getMatrixFromRoot(joint.occurrenceTwo)
                ori2 :adsk.core.Point3D = joint.geometryOrOriginTwo.origin.copy()
                ori2.transformBy(mat)
                return  ori2 #ori1,
    
            try:
                #xyz_of_joint = getJointOriginWorldCoordinates(joint)
                if isinstance(joint, adsk.fusion.AsBuiltJoint):
                    xyz_of_joint = joint.geometry.origin
                else:
                    xyz_of_joint = joint.geometryOrOriginTwo.origin
                joint_dict['xyz'] = [round(i / 100.0, 6) for i in xyz_of_joint.asArray()]  # converted to meter
                #print(f"xyz : {joint_dict['xyz']}")
    
            except:
                print('Failed:\n{}'.format(traceback.format_exc()))
                try:
                    if type(joint.geometryOrOriginTwo)==adsk.fusion.JointOrigin:
                        data = joint.geometryOrOriginTwo.geometry.origin.asArray()
                    else:
                        data = joint.geometryOrOriginTwo.origin.asArray()
                    joint_dict['xyz'] = [round(i / 100.0, 6) for i in data]  # converted to meter
                except:
                    msg = joint.name + " doesn't have joint origin. Please set it and run again."
                    break
    
            joint_name = joint.name
            if joint_name in joints_dict and joint.assemblyContext:
                # same joint in another instance of a sub-assembly
                joint_name = re.sub('[ :()]', '_', joint.assemblyContext.name) + '_' + joint_name
            if joint_name in joints_dict:
                skipped_joints.append('%s (duplicate joint name, renamed)' % joint_name)
                n = 2
                while '%s_%d' % (joint_name, n) in joints_dict:
                    n += 1
                joint_name = '%s_%d' % (joint_name, n)
            joints_dict[joint_name] = joint_dict
    found = len(joints_dict)
    orient_joints_from_base(joints_dict, skipped_joints)
    skipped_joints.append('(info) %d joints in design (%d root, %d as-built), %d read, %d exported'
                          % (len(all_joints), root.joints.count, root.asBuiltJoints.count, found, len(joints_dict)))
    return joints_dict, skipped_joints, msg


def orient_joints_from_base(joints_dict, skipped_joints):
    """
    URDF needs a tree rooted at base_link. Fusion's component1/component2 order
    doesn't guarantee that, so walk outward from base_link and flip any joint
    whose parent/child are reversed. Joints not connected to base_link, and
    extra joints that would close a loop, are dropped and reported.
    """
    visited = {'base_link'}
    frontier = ['base_link']
    kept = set()
    while frontier:
        link = frontier.pop(0)
        for name, jd in joints_dict.items():
            if name in kept:
                continue
            if jd['parent'] == link and jd['child'] not in visited:
                pass
            elif jd['child'] == link and jd['parent'] not in visited:
                jd['parent'], jd['child'] = jd['child'], jd['parent']
                jd['axis'] = [-a for a in jd['axis']]  # reversed relative motion
            else:
                continue
            kept.add(name)
            visited.add(jd['child'])
            frontier.append(jd['child'])
    for name in list(joints_dict):
        if name not in kept:
            jd = joints_dict.pop(name)
            skipped_joints.append('%s (%s -> %s not connected to base_link or closes a loop)'
                                  % (name, jd['parent'], jd['child']))