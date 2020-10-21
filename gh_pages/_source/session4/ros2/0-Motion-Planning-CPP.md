# Motion Planning using C++
>In this exercise, we'll explore MoveIt's C++ interface to programatically move a robot. 


## Motivation
Now that we’ve got a working MoveIt! configuration for your workcell and we’ve played a bit in RViz with the planning tools, let’s perform planning and motion in code. This exercise will introduce you to the basic C++ interface for interacting with the MoveIt! node in your own program. There are lots of ways to use MoveIt!, but for simple applications this is the most straight forward.

## Reference Example
[Move Group Interface tutorial](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#setup)

## 3. Further Information and Resources
 * [MoveIt! Tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
 * [MoveIt! home-page](http://moveit.ros.org/)

## Scan-N-Plan Application: Problem Statement
In this exercise, your goal is to modify the `myworkcell_core` node to:

 1. Move the robot’s tool frame to the center of the part location as reported by the service call to your vision node.

## Scan-N-Plan Application: Guidance

### Using MoveItCpp

 1. Edit your `myworkcell_node.cpp` file. In the `ScanNPlan` class's `start` method, use the response from the `LocalizePart` service to initialize a new `move_target` variable:

    ``` c++
    geometry_msgs::msg::Pose move_target;
    move_target.header.frame_id = base_frame;
    move_target.pose = response->pose;
    ```

    * make sure to place this code _after_ the call to the vision_node's service.

 1. Create a `MoveItCpp` object and a `PlanningComponent` object which will be used for performing the motion planning

     1. Add these lines in the `start` function:

        ```
        auto moveit_cpp = std::make_shared<moveit::planning_interface::MoveItCpp>(this->shared_from_this());
        moveit::planning_interface::PlanningComponent arm("manipulator", moveit_cpp);

        arm.setGoal(move_target, "tool0");
        const auto plan_solution = arm.plan();
        ```

     1. Add required dependencies in your `CMakeLists.txt`:

        ```
        find_package(moveit_msgs REQUIRED)
        find_package(moveit_ros_planning_interface REQUIRED)
        find_package(Boost REQUIRED COMPONENTS system)

        ...

        ament_target_dependencies(myworkcell_node
          rclcpp
          moveit_msgs
          moveit_ros_planning_interface
        )
        target_link_libraries(myworkcell_node Boost::system)
        ```

        and in `package.xml`:

        ```
        <depend>moveit_msgs</depend>
        <depend>moveit_ros_planning_interface</depend>
        ```

     1. Add includes at the top of file for the needed MoveIt components:

        ```
        #include <moveit/moveit_cpp/moveit_cpp.h>
        #include <moveit/moveit_cpp/planning_component.h>
        ```

 1. Currently, MoveIt2 uses many parameters that are not declared ahead of time. To enable this, we have to construct our ROS2 node with an option to automatically declare parameters when they are set in a launch file. Modify the `ScanNPlan` constructor to start with the following:

    ```
    ScanNPlan() : Node("scan_n_plan", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
      if (! this->has_parameter("base_frame"))
      {
        this->declare_parameter("base_frame", "world");
      }
    ```

    Note that the previous `declare_parameter` will now throw an exception if the parameter gets automatically declared which is why we first check if the parameter already exists now.

### Execution

 1. Currently MoveIt2 doesn't support executing planned trajectories directly through the MoveIt interface. Instead, we will configure the node to manually send the planned trajectory to another node for execution. In the constructor, add a publisher for a joint trajectory:

    ```
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "fake_joint_trajectory_controller/joint_trajectory", rclcpp::QoS(1));
    ```

    Add the `Publisher` as a member of the class alongside the already-existing service client variable:

    ```
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    ```

 1. Add the following lines after the motion planning code in the `start` function:

    ```
    if (plan_solution)
    {
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
      trajectory_pub_->publish(robot_trajectory.joint_trajectory);
    }
    ```

### Launch files and testing

 1. Open your `workcell.launch.py` and replace it with the following contents:

    ```
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
    ```

    * Note that this uses the same set of helper functions that we previously added to `urdf.launch.py`.
    * All of the extra complexity is used to build the set of parameters for the `myworkcell_node` node.

 1. Modify `urdf.launch.py` to remove the GUI-based `joint_state_publisher`. It is replaced by a `fake_joint_driver` node which is our stand in for the robot and is the node that will receive the joint trajectory that `myworkcell_node` publishes.

    ```diff
    -launch_ros.actions.Node(
    -    node_name='joint_state_publisher_gui',
    -    package='joint_state_publisher_gui',
    -    node_executable='joint_state_publisher_gui',
    -    output='screen',
    -),

    +launch_ros.actions.Node(
    +    package='fake_joint_driver',
    +    node_executable='fake_joint_driver_node',
    +    output='screen',
    +    parameters=[
    +        {'robot_description': load_file(urdf_file)},
    +        get_package_file("myworkcell_moveit_config", "config/fake_controllers.yaml"),
    +    ],
    +),
    ```

 1. Now let's test the system!

    ``` bash
    colcon build
    ros2 launch myworkcell_support urdf.launch.py
    ros2 launch myworkcell_support workcell.launch.py
    ```
