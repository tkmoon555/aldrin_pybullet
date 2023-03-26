from __future__ import print_function

import urdf_parser_py.urdf as urdf

import PyKDL as kdl

def treeFromFile(filename):
    """
    Construct a PyKDL.Tree from an URDF file.
    :param filename: URDF file path
    """

    with open(filename) as urdf_file:
        return treeFromUrdfModel(urdf.URDF.from_xml_string(urdf_file.read()))

def treeFromParam(param):
    """
    Construct a PyKDL.Tree from an URDF in a ROS parameter.
    :param param: Parameter name, ``str``
    """

    return treeFromUrdfModel(urdf.URDF.from_parameter_server(param))

def treeFromString(xml):
    """
    Construct a PyKDL.Tree from an URDF xml string.
    :param xml: URDF xml string, ``str``
    """

    return treeFromUrdfModel(urdf.URDF.from_xml_string(xml))

def _toKdlPose(pose):
    # URDF might have RPY OR XYZ unspecified. Both default to zeros
    rpy = pose.rpy if pose and pose.rpy and len(pose.rpy) == 3 else [0, 0, 0]
    xyz = pose.xyz if pose and pose.xyz and len(pose.xyz) == 3 else [0, 0, 0]

    return kdl.Frame(
          kdl.Rotation.RPY(*rpy),
          kdl.Vector(*xyz))


def _toKdlInertia(i):
    # kdl specifies the inertia in the reference frame of the link, the urdf
    # specifies the inertia in the inertia reference frame
    origin = _toKdlPose(i.origin)
    inertia = i.inertia
    return origin.M * kdl.RigidBodyInertia(
            i.mass, origin.p,
            kdl.RotationalInertia(inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz));

def _toKdlJoint(jnt):

    fixed = lambda j,F: kdl.Joint(
        j.name,
        getattr(kdl.Joint, 'Fixed') if hasattr(kdl.Joint, 'Fixed') else getattr(kdl.Joint, 'None'))
    rotational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.RotAxis)
    translational = lambda j,F: kdl.Joint(j.name, F.p, F.M * kdl.Vector(*j.axis), kdl.Joint.TransAxis)

    type_map = {
            'fixed': fixed,
            'revolute': rotational,
            'continuous': rotational,
            'prismatic': translational,
            'floating': fixed,
            'planar': fixed,
            'unknown': fixed,
            }

    return type_map[jnt.type](jnt, _toKdlPose(jnt.origin))

def _add_children_to_tree(robot_model, root, tree):


    # constructs the optional inertia
    inert = kdl.RigidBodyInertia(0)
    if root.inertial:
        inert = _toKdlInertia(root.inertial)

    # constructs the kdl joint
    (parent_joint_name, parent_link_name) = robot_model.parent_map[root.name]
    parent_joint = robot_model.joint_map[parent_joint_name]

    # construct the kdl segment
    sgm = kdl.Segment(
        root.name,
        _toKdlJoint(parent_joint),
        _toKdlPose(parent_joint.origin),
        inert)

    # add segment to tree
    if not tree.addSegment(sgm, parent_link_name):
        return False

    if root.name not in robot_model.child_map:
        return True

    children = [robot_model.link_map[l] for (j,l) in robot_model.child_map[root.name]]

    # recurslively add all children
    for child in children:
        if not _add_children_to_tree(robot_model, child, tree):
            return False

    return True

def treeFromUrdfModel(robot_model, quiet=False):
    """
    Construct a PyKDL.Tree from an URDF model from urdf_parser_python.
    :param robot_model: URDF xml string, ``str``
    :param quiet: If true suppress messages to stdout, ``bool``
    """

    root = robot_model.link_map[robot_model.get_root()]

    if root.inertial and not quiet:
        print("The root link %s has an inertia specified in the URDF, but KDL does not support a root link with an inertia.  As a workaround, you can add an extra dummy link to your URDF." % root.name);

    ok = True
    tree = kdl.Tree(root.name)

    #  add all children
    for (joint,child) in robot_model.child_map[root.name]:
        if not _add_children_to_tree(robot_model, robot_model.link_map[child], tree):
            ok = False
            break

    return (ok, tree)





"""
if __name__ == '__main__':
    import numpy as np
    # Load the URDF file
    filename = '../../config/models/simplebot_v10/model.urdf'
    # Create a kinematic tree from the URDF robot model    
    (ok, tree) = treeFromFile(filename)
    print(tree)
    # Create a kinematic chain from the URDF robot model
    chain = tree.getChain("body_v1:1", "arm_v4:3")
    print(chain)

    #create some initial angles for the joints
    q0=5.9037
    q1=17.203

    #put the initial angles in a list
    qList=[q0,q1]

    #start looking at the forward kinematics
    print("Forward kinematics")

    # create a Joint Array containing all of the initial angles
    jointAngles=kdl.JntArray(len(qList))
    for i,q in enumerate(qList):
        jointAngles[i]=np.deg2rad(q)

    #calculate the forward kinematics of the chain
    fk=kdl.ChainFkSolverPos_recursive(chain)

    #calculate the position of the end effector
    finalFrame=kdl.Frame()
    fk.JntToCart(jointAngles,finalFrame)

    print ("End-effector position: ",finalFrame.p)

    x=0.02
    y=-0.0090514
    z=-0.0708146
    desiredFrame=kdl.Frame(kdl.Vector(x,y,z))

    print ("Inverse Kinematics")

    #calculate the velocity IK of the chain
    vik=kdl.ChainIkSolverVel_pinv(chain)

    #calculate the position IK of the chain
    #ik=kdl.ChainIkSolverPos_NR(chain,fk,vik,100,1e-3)
    ik =  kdl.ChainIkSolverPos_LMA(chain)

    #print out the desired position
    print("Desired Position: ", desiredFrame.p)

    #calculate the output angles
    q_out=kdl.JntArray(len(qList))
    ik.CartToJnt(jointAngles,desiredFrame,q_out)

    #put the angles into the fk solver to get the end effector position
    checkFrame=kdl.Frame()
    fk.JntToCart(q_out,checkFrame)
    print('Actual Position: ',checkFrame.p)
"""