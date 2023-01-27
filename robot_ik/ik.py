import numpy as np

from geometry_msgs.msg import Twist
import geometry_msgs

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from urdf_parser_py.urdf import URDF
# from kdl_parser import kdl_tree_from_urdf_model
# from urdf_parser_py.urdf import URDF

# from turtlesim.srv import Spawn

import PyKDL as kdl

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
    #fixed = lambda j,F: kdl.Joint(j.name, kdl.Joint.None)
#    fixed = lambda j,F: kdl.Joint(j.name, getattr(kdl.Joint, 'None'))
    fixed = lambda j,F: kdl.Joint(j.name, getattr(kdl.Joint, 'Fixed') if hasattr(kdl.Joint, 'Fixed') else getattr(kdl.Joint, 'None'))
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

    return True;


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



def treeFromFile(filename):
    """
    Construct a PyKDL.Tree from an URDF file.
    :param filename: URDF file path
    """
 
    with open(filename) as urdf_file:
        return treeFromUrdfModel(URDF.from_xml_string(urdf_file.read()))



########### added 
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(geometry_msgs.msg.Transform, 'topic',
                                    5)
    def publish(self, msg):
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info(
                f'Publishing {msg.translation.x}, ' +
                            f'{msg.translation.y}, ' +
                            f'{msg.translation.z}, ' +
                            ' Rotation ' +
                            f'{msg.rotation.x}, ' +
                            f'{msg.rotation.y}, ' +
                            f'{msg.rotation.z}, ' +
                            f'{msg.rotation.w}, ' 
                )
########### added end
        
        
class FrameListener(Node):

    def __init__(self):
        super().__init__('vive_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'LeftHand').get_parameter_value().string_value
          
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.03, self.on_timer)

    ##############added 
        self.msg = geometry_msgs.msg.Transform()  
        self.pub = MinimalPublisher()
    ############# added end

        self.first_frame_flag = 0
    
    ###############IK
        self.urdf_file = '/home/robot/ros2_ws/src/robot_ik/robot_ik/aubo_i3.urdf'
        self.status, self.tree = treeFromFile(self.urdf_file)
        self.chain = self.tree.getChain('base_link', 'wrist3_Link')

        self.fk_ee = kdl.ChainFkSolverPos_recursive(self.chain) # 递归运动学正解求解器
        self.ik_ee = kdl.ChainIkSolverPos_LMA(self.chain)

        self.kdl_input = kdl.JntArray(self.chain.getNrOfJoints())
        # initialize the joints' angle as input
        for i in range(self.chain.getNrOfJoints()):
            self.kdl_input[i] = 0
        self.initFrame = kdl.Frame()

        self.targetFrame = kdl.Frame()

        self.fk_ee.JntToCart(self.kdl_input, self.initFrame) # q_in: joint angle, frame_out: end joint pose matrix and position

        self.initPos = self.initFrame.p #init position of end joint
        self.initRot = self.initFrame.M # init rotation matrix of end joint coordinate system 

        # self.targetPos = self.targetFrame.p #new position of end joint
        # self.targetRot = self.targetFrame.M 

        self.kdl_output = kdl.JntArray(self.chain.getNrOfJoints())
    ##################

    def on_timer(self): 

        # self.get_logger().info(f'timer called.')
        # while not rclpy._shutdown():            
        try:
            t = self.tf_buffer.lookup_transform(
                'world' ,
                'LeftHand',
                rclpy.time.Time())    
            if self.first_frame_flag == 0:
                self.first_frame_flag = 1
                self.pre_x = t.transform.translation.x
                self.pre_y = t.transform.translation.y
                self.pre_z = t.transform.translation.z

                self.targetFrame.p  = self.initPos #initialization once
                self.targetFrame.M  = self.initRot

            delta_x = t.transform.translation.x - self.pre_x
            delta_y = t.transform.translation.y - self.pre_y
            delta_z = t.transform.translation.z - self.pre_z

            # # obtain the new pose of end joint
            self.targetFrame.p [0] += delta_x
            self.targetFrame.p [1] += delta_y
            self.targetFrame.p [2] += delta_z
            self.targetFrame.M = kdl.Rotation.Quaternion(t.transform.rotation.x, 
                                                            t.transform.rotation.y,
                                                            t.transform.rotation.z,
                                                            t.transform.rotation.w)

            print("Input angle: ", self.kdl_input)
            self.ik_ee.CartToJnt(self.kdl_input, self.targetFrame, self.kdl_output)
            self.kdl_input = self.kdl_output
            # print("Target Frame: ", self.targetFrame)            
            print("Output angle: ", self.kdl_output)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform LeftHand to  world: {ex}')
            return


def main():
    rclpy.init()
    node = FrameListener()
   
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    
