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
    moveit_config_file = get_package_file('myworkcell_moveit_config', 'config/moveit.yaml')
    xacro_file = get_package_file('myworkcell_support', 'urdf/workcell.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('myworkcell_moveit_config', 'config/myworkcell.srdf')
    kinematics_file = get_package_file('myworkcell_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('myworkcell_moveit_config', 'config/ompl_planning.yaml')

    moveit_config = load_yaml(moveit_config_file)
    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_name='fake_ar_publisher_node',
            package='fake_ar_publisher',
            node_executable='fake_ar_publisher_node',
            output='screen',
        ),
        launch_ros.actions.Node(
            node_name='vision_node',
            package='myworkcell_core',
            node_executable='vision_node',
            output='screen',
        ),
        launch_ros.actions.Node(
            node_name='myworkcell_node',
            package='myworkcell_core',
            node_executable='myworkcell_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'base_frame': 'world',
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    'ompl': ompl_config,
                },
                moveit_config,
            ],
        ),
    ])
