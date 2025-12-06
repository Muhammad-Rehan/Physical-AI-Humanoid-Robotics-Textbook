# Understanding URDF (Unified Robot Description Format)

The Unified Robot Description Format (URDF) is an XML format for describing all aspects of a robot. It's a foundational tool in ROS 2 for representing your robot's physical structure, visual appearance, and collision properties. A URDF file allows ROS 2 to understand your robot's kinematic and dynamic properties, which is essential for [simulation](pathname:///docs/module2-gazebo/physics-simulation), visualization, motion planning, and more.

## 4.1 Understanding URDF (Unified Robot Description Format)

A URDF file defines a robot as a tree-like structure of `links` connected by `joints`.

### Links and Joints: Physical and Kinematic Structure

*   **Links:** These represent the rigid bodies of your robot. Each link has a name and can include definitions for:
    *   **Inertial Properties:** Mass, center of mass, and inertia tensor, crucial for realistic physics simulation.
    *   **Visual Properties:** How the link should appear, including geometry (e.g., box, cylinder, mesh) and material (color, texture).
    *   **Collision Properties:** How the link interacts with its environment and other links for collision detection, often a simplified version of the visual geometry to save computational resources.

*   **Joints:** These define the kinematic and dynamic properties of the connections between links. Each joint connects a `parent` link to a `child` link and has a `type` that specifies its degrees of freedom:
    *   **`revolute`:** A rotating joint (e.g., a wheel, a shoulder). Requires an `axis` of rotation and `limit`s (upper/lower position, velocity, effort).
    *   **`continuous`:** A revolute joint with no position limits (e.g., a continuously spinning wheel).
    *   **`prismatic`:** A sliding joint (e.g., a linear actuator). Requires an `axis` of translation and `limit`s.
    *   **`fixed`:** A rigid connection between two links. Effectively fuses the child link to the parent, removing any degrees of freedom. Useful for mounting sensors or tools.
    *   **`planar`:** Allows motion in a plane (2 prismatic, 1 revolute).
    *   **`floating`:** Allows full 6-DOF motion (3 prismatic, 3 revolute). Typically used for the base link of a mobile robot in a simulator.

### Coordinate Frames and Transformations

In a URDF, every link implicitly defines its own coordinate frame. Joints define the transformation from the parent link's coordinate frame to the child link's coordinate frame.

*   **`origin` tag:** Within a `<joint>` or `<link>`'s visual/collision element, the `<origin>` tag specifies the position (`xyz`) and orientation (`rpy` - roll, pitch, yaw) of the child frame relative to the parent frame, or the geometry relative to the link's origin. It's crucial for correctly assembling the robot's structure.

### Visual and Collision Models

*   **Visual Models:** These are what you see when you visualize your robot in tools like `RViz` or simulation environments like Gazebo. They can be simple primitive shapes (box, cylinder, sphere) or complex 3D meshes (e.g., `.stl`, `.dae`).
    ```xml
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    ```

*   **Collision Models:** These define the shape used for collision detection. They are often simplified versions of the visual models (e.g., using a bounding box or sphere instead of a complex mesh) to improve performance during physics simulation and collision checking.
    ```xml
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    ```

## 4.2 Creating a Simple URDF Model

Let's create a very simple URDF for a two-link arm, consisting of a base, a shoulder joint, an upper arm, an elbow joint, and a forearm.

### Defining a Basic Robot Structure (e.g., a 2-DOF arm)

Our robot will have:
*   A `base_link` (fixed to the world, conceptually)
*   A `shoulder_link` connected to `base_link` by a `shoulder_joint` (revolute)
*   An `elbow_link` connected to `shoulder_link` by an `elbow_joint` (revolute)

**`my_arm.urdf`**
```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/> <!-- Position relative to base_link -->
    <axis xyz="0 0 1"/> <!-- Rotates around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Shoulder Link (Upper Arm) -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Elbow Joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/> <!-- Position relative to shoulder_link (end of upper arm) -->
    <axis xyz="0 0 1"/> <!-- Rotates around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Elbow Link (Forearm) -->
  <link name="elbow_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <origin xyz="0 0 0.075" rpy="0 0 0"/>
      <inertia ixx="0.00005" ixy="0.0" ixz="0.0" iyy="0.00005" iyz="0.0" izz="0.00005"/>
    </inertial>
  </link>

</robot>
```

### Adding Visual Meshes and Collision Geometries

In the example above, we used primitive shapes (`box`, `cylinder`). For more complex robots, you would typically use 3D meshes:

```xml
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot_description/meshes/link_visual.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot_description/meshes/link_collision.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
```
The `package://` URI scheme is used to locate files relative to a ROS 2 package.

## 4.3 XACRO: URDF Macros for Reusability

Writing long URDF files can be tedious and error-prone. XACRO (XML Macros) is an XML macro language that allows you to use constants, mathematical expressions, and macros to generate URDF. This significantly improves readability, reusability, and maintainability.

To use XACRO, your file extension should typically be `.urdf.xacro`. You process it with `xacro` to produce a standard `.urdf` file.

### Parameterizing URDF Models

You can define properties like dimensions or masses as variables.

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:property name="arm_length" value="0.5" />
  <xacro:property name="arm_radius" value="0.05" />

  <link name="base_link"/>

  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-${pi/2}" upper="${pi/2}" effort="10" velocity="1"/>
  </joint>

  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder radius="${arm_radius}" length="${arm_length}"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="${0.1 * arm_length}"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```
Here, `${arm_length}` and `${arm_radius}` are variables, and `${pi/2}` shows a mathematical expression.

### Including other XACRO files

You can create modular XACRO files and include them, making it easy to define standard components (e.g., a gripper, a sensor) once and reuse them across different robot models.

```xml
<?xml version="1.0"?>
<robot name="my_complex_robot" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:include filename="$(find my_robot_description)/urdf/common_macros.xacro" />
  <xacro:include filename="$(find my_robot_description)/urdf/arm.xacro" />
  <xacro:include filename="$(find my_robot_description)/urdf/gripper.xacro" />

  <link name="world"/>

  <joint name="base_joint" type="fixed">
    <parent link="world"/>
    <child link="robot_base"/>
  </joint>

  <link name="robot_base">
    <!-- ... base visual/collision/inertial ... -->
  </link>

  <!-- Now instantiate the arm and gripper using the included macros -->
  <xacro:arm_macro parent_link="robot_base" />
  <xacro:gripper_macro parent_link="arm_tool_link" />

</robot>
```

## 4.4 Visualizing URDF Models

Once you have a URDF (or XACRO) file, the primary tool for visualizing it in ROS 2 is `RViz2`.

### Using `RViz2` to Display Robot Models

`RViz2` is a 3D visualization tool for ROS 2. It allows you to display sensor data, robot models, and planning outputs.

1.  **Start `RViz2`:**
    ```bash
    rviz2
    ```
2.  **Add `RobotModel` Display:**
    *   In the `RViz2` interface, click "Add" in the "Displays" panel.
    *   Select `RobotModel` from the list and click "OK".
3.  **Configure `RobotModel`:**
    *   In the `RobotModel` properties, set the `Description Topic` to `/robot_description`.
    *   Set the `Fixed Frame` to `base_link` (or the name of your robot's base link).
4.  **Publish the URDF:**
    You need a node that publishes your robot's URDF to the `/robot_description` topic. This is typically done by the `robot_state_publisher` node.
    ```bash
    # Ensure your URDF is in a package and the path is correct
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="`cat $(find your_robot_description)/urdf/your_robot.urdf`"
    ```
    If you're using XACRO, you'll first need to process it:
    ```bash
    xacro $(find your_robot_description)/urdf/your_robot.urdf.xacro > install/your_robot_description/share/your_robot_description/urdf/your_robot.urdf
    # Then run robot_state_publisher as above
    ```

### Joint State Publishers

To see your robot move in `RViz2`, you also need to publish joint states. The `joint_state_publisher_gui` is a convenient tool for manually controlling joint positions.

1.  **Run `joint_state_publisher_gui`:**
    ```bash
    ros2 run joint_state_publisher_gui joint_state_publisher_gui
    ```
    This will open a GUI that allows you to manipulate sliders for each revolute and prismatic joint in your URDF. As you move the sliders, `joint_state_publisher_gui` publishes messages to the `/joint_states` topic, and `robot_state_publisher` (which subscribes to `/joint_states`) updates the robot model in `RViz2`.
