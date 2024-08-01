from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
import launch_ros.actions
import launch_ros.descriptions
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # namespace_param_name = "namespace"
    # namespace = 'camera'
    # namespace_launch_arg = DeclareLaunchArgument(namespace_param_name, default_value='camera')

    # tf_prefix_param_name = "tf_prefix"
    # tf_prefix = LaunchConfiguration(tf_prefix_param_name)
    # tf_prefix_launch_arg = DeclareLaunchArgument(tf_prefix_param_name, default_value='')

    # container = launch_ros.actions.ComposableNodeContainer(
    #         name='container',
    #         namespace=namespace,
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             # Driver
    #             launch_ros.descriptions.ComposableNode(
    #                 package='openni2_camera',
    #                 plugin='openni2_wrapper::OpenNI2Driver',
    #                 name='driver',
    #                 namespace=namespace,
    #                 parameters=[{'depth_registration': False},
    #                             {'use_device_time': True},
    #                             {'rgb_frame_id': namespace+"_rgb_optical_frame"},
    #                             {'depth_frame_id': namespace +"_depth_optical_frame"},
    #                             {'ir_frame_id': namespace+"_ir_optical_frame"},],
    #                 remappings=[('depth/image', 'depth_registered/image_raw')],
    #             ),
    #             # Create XYZ point cloud
    #             launch_ros.descriptions.ComposableNode(
    #                 package='depth_image_proc',
    #                 plugin='depth_image_proc::PointCloudXyzNode',
    #                 name='points_xyz',
    #                 namespace=namespace,
    #                 parameters=[{'queue_size': 10}],
    #                 remappings=[('image_rect', 'depth/image_raw'),
    #                             ('camera_info', 'depth/camera_info'),
    #                             ('points', 'depth/points')],
    #             ),
    #         ],
    #         output='screen',
    # )


    return LaunchDescription([
        Node(
            package='real_tf_broadcaster',
            executable='real_tf_broadcaster',
            name='real_tf_broadcaster',
        ),
        Node(
            package='pointcloud_practice',
            executable='pointcloud_practice',
            name='pointcloud_practice',
            output='screen',
            parameters=[
                {'MIN_CLUSTER_SIZE' : 10},
                {'MAX_CLUSTER_SIZE' : 500},
                {'TABLE_HEIGHT' : -10.0},
                {'TOLERANCE' : 0.01},
                {'REMOVE_FLOOR' : False},
                {'LIFETIME' : 2_000_000_000}, # in nano seconds
            ],
        ),
        Node(
            package='pointcloud_pipe',
            name='pointcloud_pipe',
            executable='pipe'
        ),
        Node(
            package='image_parsing',
            name='image_parsing',
            executable='classifier',
        ),
        # namespace_launch_arg,
        # tf_prefix_launch_arg,
        # container,
    ])