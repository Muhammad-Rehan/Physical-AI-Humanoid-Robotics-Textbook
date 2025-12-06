# Setting up Your ROS 2 Environment

Before diving into development, you need a functional ROS 2 environment. This section covers installation and basic tools.

## Installation (Ubuntu, Windows, macOS)

ROS 2 officially supports Ubuntu Linux (LTS versions), Windows 10/11, and macOS. The recommended installation method typically involves using binary packages.

*   **Ubuntu (Recommended for most robotics development):**
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install software-properties-common
    sudo add-apt-repository universe

    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install ros-humble-desktop # Replace 'humble' with your desired ROS 2 distribution
    ```
    Remember to source your ROS 2 environment after installation:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    For persistent sourcing, add this line to your `~/.bashrc` file.

*   **Windows (via Chocolatey or source):**
    Installation on Windows often involves using Chocolatey for binary installs or building from source for more control. Refer to the official ROS 2 documentation for detailed instructions: [https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)

*   **macOS (via Homebrew or source):**
    macOS also supports ROS 2, typically installed via Homebrew or from source. Detailed instructions are available in the official documentation: [https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html](https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html)

## Workspace Creation and Management (`colcon`)

`colcon` is the build tool used in ROS 2. It organizes and builds your ROS 2 packages within a workspace.

1.  **Create a workspace directory:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  **Initialize `colcon` workspace (optional, but good practice):**
    ```bash
    colcon build
    ```
    This will create `install`, `log`, and `build` directories.
3.  **Source your workspace:**
    After building, you need to source your workspace's setup files to make its packages available:
    ```bash
    source install/setup.bash
    ```
    Always source your base ROS 2 installation first, then your workspace.

## Basic Command-line Tools

ROS 2 provides a rich set of command-line tools for introspection, debugging, and interaction with the running ROS 2 graph.

*   **`ros2 run`**: Executes a ROS 2 node.
    ```bash
    ros2 run <package_name> <executable_name>
    ```
    Example: `ros2 run demo_nodes_cpp talker`

*   **`ros2 topic`**: Interacts with ROS 2 topics.
    *   `ros2 topic list`: Lists active topics.
    *   `ros2 topic echo <topic_name>`: Displays messages published on a topic.
    *   `ros2 topic info <topic_name>`: Shows information about a topic, including publisher/subscriber count and message type.
    *   `ros2 topic pub <topic_name> <message_type> <args>`: Publishes a single message to a topic.

*   **`ros2 node`**: Interacts with ROS 2 nodes.
    *   `ros2 node list`: Lists active nodes.
    *   `ros2 node info <node_name>`: Displays information about a specific node.

*   **`ros2 service`**: Interacts with ROS 2 services.
    *   `ros2 service list`: Lists active services.
    *   `ros2 service call <service_name> <service_type> <args>`: Calls a service.

*   **`ros2 param`**: Interacts with node parameters.
    *   `ros2 param list`: Lists parameters for a node.
    *   `ros2 param get <node_name> <param_name>`: Gets the value of a parameter.
    *   `ros2 param set <node_name> <param_name> <value>`: Sets the value of a parameter.

These tools are invaluable for monitoring and debugging your ROS 2 applications.
