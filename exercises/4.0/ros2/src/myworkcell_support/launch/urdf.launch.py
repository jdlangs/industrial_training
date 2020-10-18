import os
import yaml
import launch
import launch_ros
from ament_index_python import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def generate_launch_description():
    xacro_file = get_package_file('myworkcell_support', 'urdf/workcell.urdf.xacro')
    urdf_file = run_xacro(xacro_file)

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_name='robot_state_publisher',
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_file],
        ),
        #launch_ros.actions.Node(
            #node_name='joint_state_publisher_gui',
            #package='joint_state_publisher_gui',
            #node_executable='joint_state_publisher_gui',
            #output='screen',
        #),
        launch_ros.actions.Node(
            package='fake_joint_driver',
            node_executable='fake_joint_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'robot_description': load_file(urdf_file)},
                get_package_file("myworkcell_moveit_config", "config/fake_controllers.yaml"),
            ],
        ),
        launch_ros.actions.Node(
            node_name='rviz',
            package='rviz2',
            node_executable='rviz2',
            output='screen',
        )
    ])
