# Python Agents to ROS Controllers

This chapter focuses on integrating intelligent Python agents with ROS 2 to control robotic systems. We'll explore how to design and implement Python code that can interact with ROS 2 topics, services, and actions to command robots, process sensor data, and execute complex behaviors.

## 3.1 Basics of `rclpy`

`rclpy` is the Python client library for ROS 2, providing an intuitive interface to all core ROS 2 functionalities. It allows Python developers to write ROS 2 nodes, publishers, subscribers, service clients and servers, and action clients and servers.

### Initializing and Shutting Down ROS 2 in Python

Every ROS 2 Python application must initialize the `rclpy` library and create a node.

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize rclpy library
    rclpy.init(args=args)

    # Create a node
    node = Node('my_python_node')
    node.get_logger().info('My Python Node started!')

    # Keep the node alive (e.g., waiting for messages or service calls)
    rclpy.spin(node)

    # Destroy the node once spin() returns
    node.destroy_node()

    # Shutdown rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating Nodes, Publishers, Subscribers, Service Clients/Servers, Action Clients/Servers

The examples below provide a quick overview of how to create each communication primitive. For detailed explanations, refer to the previous chapter and ROS 2 documentation.

#### Node
A `Node` is the fundamental unit. All ROS 2 communication happens through methods of a `Node` object.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_custom_node')
        self.get_logger().info('My Custom Node has been initialized!')

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Publisher
To send data to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the message type

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Topic name: chatter, Queue size: 10
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from Python: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber
To receive data from a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter', # Subscribe to the 'chatter' topic
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client and Server
For request-reply communication. Assuming `example_interfaces.srv.AddTwoInts` is available.

**Service Server:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server "add_two_ints" is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d -> sum: %d' % (request.a, request.b, response.sum))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client:**
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        # Spin until the future is complete, getting the result
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info(
                'Result of add_two_ints: for %d + %d = %d' %
                (self.req.a, self.req.b, self.future.result().sum))
        else:
            self.get_logger().error('Service call failed %r' % (self.future.exception(),))
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 3:
        print('Usage: ros2 run <package_name> minimal_client_async <arg1> <arg2>')
        return
    
    minimal_client = MinimalClientAsync()
    minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.2 Python Agents to ROS Controllers

Integrating Python-based AI agents with ROS 2 allows for sophisticated control over robotic platforms. This typically involves an AI agent (e.g., a reinforcement learning agent, a planning algorithm, or a behavioral state machine) making decisions and then translating those decisions into ROS 2 commands.

### Writing Simple Control Loops in Python

A common pattern for controlling robots is to implement a control loop within a ROS 2 node. This loop reads sensor data (via subscribers), processes it to make a decision, and then sends commands to the robot (via publishers or service calls).

Consider a simple agent that tries to keep a robot moving forward while avoiding obstacles detected by a hypothetical `/scan` topic.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # For sending velocity commands
from sensor_msgs.msg import LaserScan # For receiving laser scan data

class SimpleNavigationAgent(Node):
    def __init__(self):
        super().__init__('simple_navigation_agent')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription # prevent unused variable warning
        self.timer = self.create_timer(0.1, self.control_loop) # Control loop runs at 10 Hz

        self.last_scan_data = None
        self.linear_speed = 0.2
        self.angular_speed = 0.0

    def scan_callback(self, msg):
        self.last_scan_data = msg

    def control_loop(self):
        twist = Twist()
        if self.last_scan_data is not None:
            # Simple obstacle avoidance logic: if something is too close in front, turn
            # Assuming scan.ranges[0] is the front distance
            front_distance = self.last_scan_data.ranges[len(self.last_scan_data.ranges) // 2] # Approximate front
            
            if front_distance < 0.5: # Obstacle too close
                self.get_logger().info(f'Obstacle detected at {front_distance:.2f}m. Turning.')
                twist.linear.x = 0.0 # Stop linear motion
                twist.angular.z = 0.5 # Turn right
            else:
                self.get_logger().info(f'Path clear. Moving forward. Front distance: {front_distance:.2f}m')
                twist.linear.x = self.linear_speed # Move forward
                twist.angular.z = 0.0 # No turning
        else:
            self.get_logger().info('No scan data yet. Robot stopped.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    agent = SimpleNavigationAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
This example shows:
*   A `MinimalAgent` node publishing `Twist` messages (linear and angular velocities) to the `cmd_vel` topic, which is a standard topic for controlling mobile robots.
*   It subscribes to a `LaserScan` topic to get data from a LiDAR sensor.
*   A `control_loop` timer callback processes the sensor data and decides on the robot's next movement, publishing it to `cmd_vel`.

### Interfacing with Simulated Robot Joint States and Commands

For more complex robots like manipulators or humanoid robots, control often involves sending commands to individual joints and receiving feedback on their current states. This typically relies on understanding the robot's physical structure, as described in the [Understanding URDF](understanding-urdf) chapter.

*   **Joint State Publishers:** Robots typically publish their current joint angles and velocities on a `/joint_states` topic (message type `sensor_msgs/JointState`). Your Python agent can subscribe to this topic to understand the robot's current configuration.
*   **Joint Trajectory Controllers:** For smooth and coordinated motion, robots often expose action interfaces (e.g., `FollowJointTrajectory`) that allow you to send a sequence of target joint positions, velocities, and accelerations over time. Your agent would act as an action client to these interfaces.

This approach allows agents to command robots at a higher level of abstraction than raw motor commands, focusing on desired poses or trajectories.

## 3.3 Advanced `rclpy` Features

### Timers and Callbacks
Timers allow you to schedule functions to be called periodically. This is essential for control loops, data logging, or any task that needs to execute at a fixed rate.

```python
# Example: Timer callback already shown in the publisher and navigation agent examples.
# self.timer = self.create_timer(period_in_seconds, self.callback_function)
```

### Parameters
ROS 2 nodes can expose parameters that can be dynamically changed at runtime, allowing for flexible configuration without recompiling code.

```python
import rclpy
from rclpy.node import Node

class ParameterExample(Node):
    def __init__(self):
        super().__init__('parameter_example')
        # Declare a parameter with a default value
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('publish_frequency', 1.0)

        # Get parameter value
        param_value = self.get_parameter('my_parameter').get_parameter_value().string_value
        self.get_logger().info(f'Initial parameter value: {param_value}')

        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.get_logger().info(f'Publish frequency: {freq} Hz')

        # You can also set a callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'my_parameter':
                self.get_logger().info(f'Parameter "my_parameter" changed to: {param.value}')
            if param.name == 'publish_frequency':
                self.get_logger().info(f'Parameter "publish_frequency" changed to: {param.value} Hz')
        return rclpy.ParameterResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ParameterExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
You can change parameters using the `ros2 param set` command:
`ros2 param set /parameter_example my_parameter "new_value"`
`ros2 param set /parameter_example publish_frequency 2.0`

### Logging
`rclpy` provides a standard way to log messages with different severity levels (DEBUG, INFO, WARN, ERROR, FATAL).

```python
import rclpy
from rclpy.node import Node

class LoggerExample(Node):
    def __init__(self):
        super().__init__('logger_example')
        self.get_logger().debug('This is a debug message.')
        self.get_logger().info('This is an info message.')
        self.get_logger().warn('This is a warning message.')
        self.get_logger().error('This is an error message.')
        self.get_logger().fatal('This is a fatal message.')

def main(args=None):
    rclpy.init(args=args)
    node = LoggerExample()
    rclpy.spin_once(node, timeout_sec=1) # Spin once to process logs
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Logging levels can be configured at runtime.
