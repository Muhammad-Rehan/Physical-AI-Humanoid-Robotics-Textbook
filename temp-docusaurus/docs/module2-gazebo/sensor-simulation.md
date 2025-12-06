# Simulating Sensors (LiDAR, Depth Cameras)

Accurate sensor simulation is critical for developing and testing robotic perception, navigation, and control algorithms. Gazebo provides powerful capabilities to model a wide range of sensors, including LiDAR and depth cameras, allowing developers to work with realistic data without the need for physical hardware.

## 2.1 Introduction to Sensor Simulation

### Why simulate sensors? Testing algorithms without hardware

Simulating sensors offers numerous advantages in robotics development:

*   **Cost-Effectiveness:** Avoids the expense and potential damage to real sensors and robots during early development and testing phases.
*   **Safety:** Allows testing of dangerous scenarios (e.g., collisions, extreme environments) without risk.
*   **Reproducibility:** Simulations are deterministic, enabling exact reproduction of test cases for debugging and regression testing.
*   **Accessibility:** Provides access to a wide range of virtual sensors and environments, even if physical hardware is unavailable.
*   **Rapid Iteration:** Speeds up the development cycle by allowing quick changes and re-testing of algorithms.
*   **Synthetic Data Generation:** Can generate large datasets for training machine learning models for perception tasks, especially useful for rare or hard-to-capture scenarios.

### Fidelity vs. performance

A key consideration in sensor simulation is the trade-off between fidelity (how accurately the simulation models reality) and performance (how fast the simulation runs).

*   **High Fidelity:** Aims to replicate real-world sensor behavior as closely as possible, including noise, distortions, lighting effects, and environmental interactions. This often comes at a high computational cost.
*   **High Performance:** Prioritizes speed and efficiency, often by simplifying sensor models or reducing the complexity of the simulated environment. This is crucial for real-time simulations or large-scale data generation.

Choosing the right balance depends on the specific application. For early algorithm prototyping, lower fidelity might suffice. For validating deployment-ready systems, higher fidelity becomes essential.

## 2.2 Simulating LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors measure distances to objects by illuminating them with laser light and measuring the reflection time. In robotics, they are widely used for mapping, localization, and obstacle avoidance.

### Adding a LiDAR sensor to an SDF/URDF model

To add a LiDAR sensor to your robot model in Gazebo, you typically modify your URDF/XACRO file. You'll define a new `link` for the LiDAR and a `joint` to attach it to your robot. Then, within the `<link>`'s `<gazebo>` tag, you'll specify the sensor details using a `<sensor>` element.

Here's an example of adding a 2D LiDAR (Hokuyo UTM-30LX equivalent) to a robot's `base_link`:

```xml
  <link name="hokuyo_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-05" ixy="0" ixz="0" iyy="1e-05" iyz="0" izz="1e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot_description/meshes/hokuyo.dae"/> <!-- Optional: 3D model of the sensor -->
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.05"/> <!-- Simplified collision box -->
      </geometry>
    </collision>
  </link>

  <joint name="hokuyo_joint" type="fixed">
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/> <!-- Position relative to base_link -->
    <parent link="base_link"/>
    <child link="hokuyo_link"/>
  </joint>

  <gazebo reference="hokuyo_link">
    <sensor name="laser_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30</update_rate> <!-- Sensor update rate in Hz -->
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>      <!-- Number of laser beams -->
            <resolution>1</resolution>  <!-- Resolution (unused in ROS 2 LaserScan) -->
            <min_angle>-1.57</min_angle> <!-- Minimum horizontal angle (radians) -->
            <max_angle>1.57</max_angle>  <!-- Maximum horizontal angle (radians) -->
          </horizontal>
          <vertical>
            <samples>1</samples>       <!-- For 2D LiDAR, usually 1 vertical sample -->
            <min_angle>0</min_angle>
            <max_angle>0</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.10</min>            <!-- Minimum range (meters) -->
          <max>10.0</max>            <!-- Maximum range (meters) -->
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>       <!-- Standard deviation of Gaussian noise -->
        </noise>
      </ray>
      <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <argument>~/out</argument>
          <namespace>/</namespace>
          <remapping>~/out:=scan</remapping> <!-- Remap Gazebo's default topic to 'scan' -->
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>hokuyo_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### Configuring LiDAR parameters: range, samples, update rate

In the example above, key parameters are configured:

*   **`<update_rate>`**: How often the sensor publishes data (e.g., 30 Hz).
*   **`<samples>`**: Number of individual laser beams in the horizontal scan (e.g., 720).
*   **`<min_angle>`, `<max_angle>`**: The field of view of the laser scan in radians.
*   **`<min>`, `<max>`**: The minimum and maximum detection range of the laser in meters.
*   **`<noise>`**: Parameters for simulating sensor noise (e.g., Gaussian noise with a standard deviation).

**[ROS 2](pathname:///docs/module1-ros2/introduction) interface:** publishing `sensor_msgs/LaserScan` messages

The `libgazebo_ros_ray_sensor.so` plugin is configured to publish messages of type `sensor_msgs/msg/LaserScan` to the `/scan` topic. This is the standard ROS 2 message type for 2D LiDAR data, containing an array of range values, angles, and intensity information.

### Visualizing LiDAR data in RViz2

After launching your robot in Gazebo with the LiDAR sensor configured, you can visualize the data in `RViz2`:

1.  Launch `RViz2`.
2.  Add a `LaserScan` display.
3.  Set the `Topic` to `/scan` (or whatever you remapped it to).
4.  Set the `Fixed Frame` to `hokuyo_link` (or the frame where your sensor is attached).
You should see the simulated laser points in the 3D environment, allowing you to debug your sensor configuration and algorithms.

## 2.3 Simulating Depth Cameras (RGB-D)

Depth cameras (e.g., Intel RealSense, Microsoft Kinect) provide both a color image (RGB) and a depth map (distance to objects) for each pixel. They are invaluable for object detection, 3D reconstruction, and navigation.

### Adding a depth camera (e.g., RealSense, Kinect) to an SDF/URDF model

Similar to LiDAR, a depth camera is added as a `link` and `joint` in your URDF, with a `<sensor>` element in its `<gazebo>` tag. The `type` for a depth camera is `depth_camera` or `camera` with specific plugins.

Here's an example:

```xml
  <link name="camera_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1e-05" ixy="0" ixz="0" iyy="1e-05" iyz="0" izz="1e-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </collision>
  </link>

  <joint name="camera_joint" type="fixed">
    <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="camera_link"/>
  </joint>

  <gazebo reference="camera_link">
    <sensor name="depth_camera_sensor" type="depth">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30.0</update_rate>
      <camera name="rgb_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- ~60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
        <ros>
          <argument>camera_name/image_raw:=rgb/image_raw</argument>
          <argument>camera_name/image_depth:=depth/image_raw</argument>
          <argument>camera_name/camera_info:=rgb/camera_info</argument>
          <argument>camera_name/depth/camera_info:=depth/camera_info</argument>
          <namespace>/</namespace>
          <remapping>~/depth_camera/points:=points</remapping>
        </ros>
        <point_cloud_cutout_min_z>0.0</point_cloud_cutout_min_z>
        <point_cloud_cutout_max_z>1.0</point_cloud_cutout_max_z>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### Configuring camera parameters: FOV, resolution, noise

Key configuration parameters for depth cameras include:

*   **`<horizontal_fov>`**: Horizontal Field of View (in radians).
*   **`<image>`**:
    *   **`<width>`, `<height>`**: Resolution of the image (e.g., 640x480).
    *   **`<format>`**: Pixel format (e.g., `R8G8B8` for color, `L_INT16` for depth).
*   **`<clip>`**:
    *   **`<near>`, `<far>`**: Near and far clipping planes, defining the valid depth range.
*   **`<noise>`**: Parameters for simulating noise in the depth data.

### [ROS 2](pathname:///docs/module1-ros2/introduction) interfaces:** `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, `sensor_msgs/PointCloud2`

The `libgazebo_ros_depth_camera.so` plugin publishes various ROS 2 messages:

*   `/rgb/image_raw` (`sensor_msgs/msg/Image`): The color image stream.
*   `/depth/image_raw` (`sensor_msgs/msg/Image`): The depth image stream (typically 16-bit monochrome).
*   `/rgb/camera_info`, `/depth/camera_info` (`sensor_msgs/msg/CameraInfo`): Camera calibration parameters.
*   `/points` (`sensor_msgs/msg/PointCloud2`): A 3D point cloud generated from the depth image, representing the scene in 3D coordinates.

### Visualizing RGB and Depth images in RViz2/image_view

*   **RGB Image:**
    1.  Launch `RViz2`.
    2.  Add an `Image` display.
    3.  Set the `Topic` to `/rgb/image_raw`.
*   **Depth Image:**
    1.  Launch `RViz2`.
    2.  Add an `Image` display.
    3.  Set the `Topic` to `/depth/image_raw`. (Note: You might need to adjust min/max display values to see the depth patterns).
*   **Point Cloud:**
    1.  Launch `RViz2`.
    2.  Add a `PointCloud2` display.
    3.  Set the `Topic` to `/points`.
    4.  Set the `Fixed Frame` to `camera_link`.

Alternatively, for viewing images directly, you can use `rqt_image_view`:
```bash
ros2 run rqt_image_view rqt_image_view
```
Then select the desired image topic from the dropdown.

## 2.4 Other Sensor Types

Gazebo can simulate many other sensors, each with its specific configuration and ROS 2 interface:

*   **IMU (Inertial Measurement Unit):** Provides linear acceleration and angular velocity (`sensor_msgs/msg/Imu`). Plugins like `libgazebo_ros_imu_sensor.so`.
*   **Contact Sensors:** Detect physical contact with objects (`gazebo_msgs/msg/ContactState`). Plugins like `libgazebo_ros_bumper.so`.
*   **GPS (Global Positioning System):** Provides latitude, longitude, and altitude (`sensor_msgs/msg/NavSatFix`). Plugins like `libgazebo_ros_gps_sensor.so`.

Each sensor type requires careful configuration in the SDF/URDF and understanding of its corresponding ROS 2 message type to effectively use the simulated data in your robotics applications.
