# Introduction to ROS 2

## 1.1 What is ROS 2?

The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS 2 is the second generation of this framework, re-architected to address the limitations of ROS 1 in areas such as real-time performance, security, and multi-robot systems.

### Evolution from ROS 1 to ROS 2

ROS 1 revolutionized robotics development by providing a standardized, open-source framework. However, as robotics applications grew more sophisticated and moved into production environments, several limitations became apparent:

*   **Real-time Performance:** ROS 1 was not designed with strict real-time requirements in mind, making it challenging for applications demanding precise timing and control.
*   **Inter-process Communication:** Its communication layer relied on TCP/IP, which could be inefficient for high-bandwidth data and did not offer built-in quality-of-service (QoS) guarantees.
*   **Security:** ROS 1 lacked built-in security features, making it vulnerable in networked and critical applications.
*   **Multi-robot Support:** Managing multiple robots with unique namespaces and coordinating their actions was cumbersome.
*   **Windows and macOS Support:** ROS 1 primarily supported Linux, limiting its adoption in certain development and deployment environments.

ROS 2 was developed from the ground up to overcome these challenges while retaining the core philosophy and many successful concepts of ROS 1. It offers a more robust, scalable, and secure platform suitable for a broader range of applications, from research prototypes to industrial deployments.

### Key Improvements: DDS, Real-time, Security, Multi-robot Support

ROS 2's fundamental improvements stem from several key architectural changes:

*   **DDS (Data Distribution Service) as the Middleware:** Instead of a custom TCP/IP-based communication system, ROS 2 leverages DDS, an international standard for publish-subscribe communication. DDS provides:
    *   **Quality of Service (QoS):** Developers can specify policies for reliability, durability, latency, and more, allowing fine-grained control over data delivery. This is crucial for real-time and safety-critical applications.
    *   **Discovery:** DDS handles automatic discovery of participants, simplifying network configuration.
    *   **Platform Independence:** DDS implementations are available across various operating systems.
*   **Enhanced Real-time Capabilities:** With DDS's QoS features and careful design, ROS 2 enables applications to achieve deterministic behavior, essential for precise robot control and safety.
*   **Built-in Security:** ROS 2 incorporates Security Best Practices directly into its communication layer, utilizing encryption, authentication, and access control mechanisms based on the DDS Security Specification. This ensures that only authorized nodes can communicate and that data remains confidential and unaltered.
*   **Improved Multi-robot Support:** DDS's distributed nature naturally supports multiple robots operating in the same network without complex remapping. Features like namespaces and domain IDs simplify the management of independent robotic systems.
*   **Cross-platform Compatibility:** ROS 2 offers robust support for Linux, Windows, and macOS, expanding its usability for developers and deployments.

### Target Audience and Use Cases

ROS 2 is designed for a broad spectrum of users and applications:

*   **Robotics Researchers:** Provides a powerful and flexible platform for exploring new algorithms in areas like AI, perception, navigation, and human-robot interaction.
*   **Robotics Engineers:** Offers production-grade features for developing and deploying commercial robotic systems in manufacturing, logistics, healthcare, and other industries.
*   **Hobbyists and Educators:** While more complex than ROS 1 in some aspects, ROS 2's modern architecture and improved tools make it an excellent platform for learning advanced robotics.

Use cases for ROS 2 are extensive and include:

*   Autonomous vehicles and drones
*   Industrial robotic arms
*   Service robots (e.g., cleaning robots, delivery robots)
*   Humanoid and bipedal robots
*   Medical robots
*   Space robotics

## 1.2 ROS 2 Architecture Overview

Understanding the architecture of ROS 2 is crucial for effective development. It revolves around a decentralized graph of executable nodes that communicate with each other using various mechanisms.

### Nodes, Topics, Services, Actions

These are the primary communication primitives in ROS 2:

*   **Nodes:** The fundamental computational units in ROS 2. A node is essentially an executable program that performs a specific task (e.g., controlling a motor, processing camera data, planning a path).
*   **Topics:** A publish-subscribe mechanism for asynchronous, one-to-many data streaming. Nodes publish messages to topics, and any node subscribed to that topic receives those messages. This is ideal for continuous data flows like sensor readings or motor commands.
*   **Services:** A request-reply mechanism for synchronous, one-to-one communication. A client node sends a request to a service server node, and the server processes the request and sends back a response. This is suitable for discrete tasks like triggering an action or querying a specific piece of information.
*   **Actions:** A higher-level communication type built on topics and services, designed for long-running, goal-oriented tasks that provide periodic feedback and can be preempted. An action client sends a goal to an action server, which executes the task, provides feedback on its progress, and ultimately returns a result. Examples include "drive to a location" or "pick up an object."

To illustrate these concepts, let's look at simple Python examples. For more advanced integration, refer to the [Python Agents to ROS Controllers](python-agents-to-controllers) chapter.

### Example: Simple ROS 2 Publisher (Python)

This node will continuously publish "Hello ROS 2!" messages to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS client library for Python
    minimal_publisher = MinimalPublisher() # Create a node
    rclpy.spin(minimal_publisher) # Keep the node alive
    minimal_publisher.destroy_node() # Destroy the node once spin() returns
    rclpy.shutdown() # Shutdown the ROS client library

if __name__ == '__main__':
    main()
```

### Example: Simple ROS 2 Subscriber (Python)

This node will subscribe to the topic from the publisher above and print received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
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

### Example: Simple ROS 2 Service (Python)

This example includes both a service server (adds two integers) and a client that calls it.

First, define the service in a `.srv` file (e.g., `AddTwoInts.srv` in `srv/` directory of your package):
```
int64 a
int64 b
---
int64 sum
```

Then, the service server and client Python code:

**Service Server:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Assuming you have this service defined

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
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
from example_interfaces.srv import AddTwoInts # Assuming you have this service defined

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

These examples provide a basic understanding of how to implement nodes, publishers, subscribers, and services in ROS 2 using Python.

### DDS (Data Distribution Service) as the Middleware

At the heart of ROS 2's communication is the Data Distribution Service (DDS). DDS is a vendor-neutral, open standard middleware that enables real-time, high-performance, and scalable data exchange in distributed systems. ROS 2 uses DDS to handle the low-level communication details, allowing developers to focus on robot logic.

Key aspects of DDS within ROS 2:

*   **Decentralized:** There is no central master node (unlike `roscore` in ROS 1). Nodes discover each other directly via DDS.
*   **Quality of Service (QoS):** DDS offers a rich set of QoS policies that can be applied to topics, services, and actions. These policies govern aspects like:
    *   **Reliability:** Whether messages are guaranteed to arrive (reliable) or if some loss is acceptable (best effort).
    *   **Durability:** Whether late-joining subscribers receive historical messages.
    *   **Liveliness:** How publishers and subscribers detect each other's presence.
    *   **Latency Budget:** The maximum acceptable delay for message delivery.
    *   **Deadline:** The expected rate at which data is updated.

Developers can configure these QoS settings to match the specific requirements of their robotic application, ensuring optimal performance and behavior.


