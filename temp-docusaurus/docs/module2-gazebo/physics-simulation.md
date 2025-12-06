# Simulating Physics in Gazebo

Gazebo is a powerful 3D robot simulator that is widely used in the robotics community. It allows for testing algorithms, designing robots, and performing regression testing without the need for physical hardware. A core strength of Gazebo is its realistic physics engine, which enables accurate simulation of robot dynamics and interactions with the environment.

## 1.1 Introduction to Gazebo

### What is Gazebo? Role in Robotics Development

Gazebo is an open-source robotics simulator that offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. It provides robust physics simulation, high-quality graphics, and convenient programmatic and graphical interfaces.

Its primary role in robotics development includes:

*   **Algorithm Testing:** Developers can test control algorithms, navigation stacks, and perception systems in a simulated environment before deploying them to real hardware. This speeds up development and reduces the risk of damaging physical robots.
*   **Robot Design and Prototyping:** Gazebo allows engineers to virtually assemble and test robot designs, iterate on mechanical components, and evaluate sensor placement.
*   **Regression Testing:** Automated tests can be run in Gazebo to ensure that changes to software or hardware models do not introduce regressions.
*   **Education and Research:** It provides an accessible platform for students and researchers to learn about robotics and experiment with new ideas.

### Key Features: Physics Engine, Sensor Models, Plugins

*   **Physics Engine:** Gazebo supports several high-performance physics engines, including ODE (Open Dynamics Engine), Bullet, DART, and Simbody. These engines handle rigid body dynamics, collision detection, and friction, providing realistic interactions.
*   **Sensor Models:** Gazebo can simulate a wide array of sensors, including cameras (monocular, stereo, depth), LiDAR (laser range finders), IMUs (inertial measurement units), GPS, contact sensors, and more. These models can often incorporate noise and other real-world effects.
*   **Plugins:** Gazebo's functionality is highly extensible through a plugin architecture. Users can write custom plugins in C++ to:
    *   Control robot behavior (e.g., motor controllers).
    *   Implement custom sensor logic.
    *   Interface with external software (like [ROS 2](pathname:///docs/module1-ros2/introduction)).
    *   Create custom world elements or environmental effects.

### Gazebo vs. other simulators

While Gazebo is prevalent, other simulators exist, each with its strengths:

*   **Unity (with Unity Robotics Hub):** Offers superior graphical rendering and a powerful C# scripting environment, making it excellent for high-fidelity visualization, synthetic data generation for AI, and game development. Its physics engine (PhysX) is also robust.
*   **CoppeliaSim (formerly V-REP):** Known for its versatility and large collection of robot models. It has a powerful built-in scripting API and supports multiple physics engines.
*   **Webots:** An open-source, fast, and stable simulator widely used in academic settings, particularly for mobile robotics and humanoids.
*   **PyBullet:** A Python module for robotics, games, and VR, using the Bullet physics engine. Good for rapid prototyping and reinforcement learning.

Gazebo stands out for its deep integration with ROS 2, its comprehensive sensor simulation capabilities, and its open-source nature with a large community.

## 1.2 Gazebo Installation and Setup

Gazebo is typically installed as part of the ROS 2 environment. If you followed the ROS 2 installation guide in Module 1, you likely already have a compatible Gazebo version (e.g., Gazebo Garden with Humble or Iron).

### Installation on Ubuntu (ROS 2 Integrated)

Assuming you have ROS 2 Humble installed on Ubuntu, Gazebo Garden (the default for Humble) can be installed as follows:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs # Installs Gazebo Garden along with ROS 2 integration packages
```

### Basic GUI interface and controls

Once installed, you can launch Gazebo:

```bash
gazebo # Launches the Gazebo GUI without a world
```
Or, typically with a world file:
```bash
gazebo --verbose worlds/empty.world # Launches Gazebo with an empty world
```

The Gazebo GUI provides:

*   **3D Viewport:** Displays the simulated world and robots. You can navigate using your mouse (left-click + drag to orbit, right-click + drag to pan, scroll wheel to zoom).
*   **Scene Tree:** On the left, it shows all entities in the simulation (models, lights, sensors). You can select objects here.
*   **Toolbar:** Icons for adding simple shapes (box, sphere, cylinder), lights, or interacting with the simulation (play/pause, step).
*   **Inspector:** On the right, shows properties of selected objects (position, rotation, physics parameters).

### Starting Gazebo with an empty world

To start Gazebo with an empty environment, you can use the command:

```bash
gazebo worlds/empty.world
```
This loads a default empty world, providing a clean slate to insert models or create new environments.

## 1.3 Creating a Simple Simulation World

Gazebo worlds are defined using the Simulation Description Format (SDF). SDF is an XML format designed to describe environments, objects, and robots for simulators like Gazebo.

### SDF (Simulation Description Format) overview

SDF is more general than URDF (which is primarily for robots). An SDF file can describe:

*   **`<world>`:** The top-level element for defining a simulation environment.
*   **`<model>`:** Represents a collection of links and joints, typically a robot or an object in the world.
*   **`<link>`:** A rigid body, similar to URDF.
*   **`<joint>`:** Connects two links, similar to URDF.
*   **`<collision>`:** Defines collision geometry.
*   **`<visual>`:** Defines visual geometry.
*   **`<light>`:** Defines light sources.
*   **`<gravity>`:** Sets the world's gravity.
*   **`<physics>`:** Configures the physics engine.
*   **`<include>`:** Allows for modularity by including other SDF files or models from the Gazebo Model Database.

### Defining a world: ground plane, lights, simple shapes

A basic world SDF (`simple_world.sdf`) might look like this:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="simple_world">
    <!-- Gravity -->
    <gravity>0 0 -9.8</gravity>

    <!-- Physics Engine -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- A simple sun light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- An obstacle: a simple box -->
    <model name="my_box">
      <pose>1 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry><box><size>1 1 1</size></box></geometry>
          <material><ambient>0.0 1.0 0.0 1</ambient><diffuse>0.0 1.0 0.0 1</diffuse></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia ixx="0.166667" ixy="0" ixz="0" iyy="0.166667" iyz="0" izz="0.166667"/>
        </inertial>
      </link>
    </model>
    
  </world>
</sdf>
```

### Adding basic models from Gazebo's model database

Gazebo comes with a rich online model database (`https://app.gazebosim.org/`) where you can find pre-built models of common objects (chairs, tables, plants, robots, etc.). You can include them directly in your SDF:

```xml
    <include>
      <uri>model://coke_can</uri>
      <pose>-1 0 0.1 0 0 0</pose>
    </include>
```
Gazebo will automatically download these models if they are not present locally.

### Step-by-step example: a flat world with a few obstacles

To create and run `simple_world.sdf`:

1.  Save the XML content above as `simple_world.sdf` in a directory (e.g., `my_gazebo_worlds`).
2.  Launch Gazebo with your world file:
    ```bash
    gazebo simple_world.sdf
    ```
    You should see a ground plane, a sun, and a green box obstacle.

## 1.4 Integrating URDF Models into Gazebo

While [URDF](pathname:///docs/module1-ros2/understanding-urdf) describes the robot, Gazebo needs additional information (like physics properties or specific plugins) to simulate it correctly. This is done by embedding `<gazebo>` tags directly into your URDF (or XACRO) file.

### Using `<gazebo>` tags in URDF/XACRO for Gazebo-specific properties

You can add `<gazebo>` tags at the `<link>` and `<joint>` level in your URDF to specify Gazebo-specific properties.

Example for a link:
```xml
  <link name="base_link">
    <!-- ... URDF visual, collision, inertial tags ... -->
    <gazebo reference="base_link">
      <material>Gazebo/Orange</material>
      <mu1>0.2</mu1> <!-- Friction coefficient 1 -->
      <mu2>0.2</mu2> <!-- Friction coefficient 2 -->
      <kp>1000000.0</kp> <!-- Contact stiffness -->
      <kd>1.0</kd> <!-- Contact damping -->
    </gazebo>
  </link>
```

Example for a joint (useful for adding joint motor properties or limits):
```xml
  <joint name="shoulder_joint" type="revolute">
    <!-- ... URDF joint properties ... -->
    <gazebo reference="shoulder_joint">
      <provide_feedback>true</provide_feedback> <!-- Enable joint force/torque feedback -->
    </gazebo>
  </joint>
```

### Adding Gazebo plugins (e.g., `libgazebo_ros_diff_drive.so` for wheeled robots)

Gazebo plugins are crucial for making your simulated robot interactive and connecting it to ROS 2. For instance, to control a differential drive robot with ROS 2 `cmd_vel` messages, you'd use a `gazebo_ros_diff_drive` plugin.

```xml
  <robot name="my_diff_drive_robot">
    <!-- ... links and joints ... -->

    <gazebo>
      <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <ros>
            <namespace>/</namespace>
        </ros>
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_radius>0.1</wheel_radius>
        <publish_odom>true</publish_odom>
        <publish_wheel_tf>true</publish_wheel_tf>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>base_footprint</robot_base_frame>
        <command_topic>cmd_vel</command_topic>
        <odometry_topic>odom</odometry_topic>
        <update_rate>30</update_rate>
      </plugin>
    </gazebo>
  </robot>
```
This plugin reads `Twist` messages from the `cmd_vel` topic, converts them into wheel commands, simulates the robot's motion, and publishes odometry data to `odom`.

### Launching a robot model in Gazebo with ROS 2 interfaces

To bring your robot into a Gazebo simulation and enable ROS 2 communication, you typically use a ROS 2 launch file. This launch file will:

1.  Start Gazebo with your chosen world.
2.  Load your robot's URDF/XACRO model and publish it to the `/robot_description` topic using `robot_state_publisher`.
3.  Spawn your robot into the Gazebo world using the `spawn_entity.py` script.
4.  Optionally, start `joint_state_publisher_gui` or other controllers.

Example ROS 2 launch file (`my_robot_launch.py`):
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the launch directory
    pkg_name = 'my_robot_description' # Replace with your robot's package name
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # Path to your XACRO file
    xacro_file = os.path.join(pkg_share_dir, 'urdf', 'my_robot.urdf.xacro')
    
    # Process the xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(pkg_share_dir, 'worlds', 'simple_world.sdf')}.items(),
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Spawn Entity Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])
```
This launch file provides a robust way to bring up your simulated robot in Gazebo, ready for interaction via ROS 2.
