# Cognitive Planning with LLMs

Large Language Models (LLMs) are transforming various domains, and robotics is no exception. Their ability to understand and generate human-like text, coupled with their vast encapsulated knowledge, makes them powerful tools for cognitive planning, enabling robots to interpret high-level commands and generate sequences of actions to achieve complex goals.

## 2.1 Introduction to Large Language Models (LLMs) in Robotics

### Motivation: High-level reasoning, task decomposition, common-sense knowledge

Traditional robotics heavily relies on explicitly programmed rules and finite state machines for task execution. This approach becomes brittle and difficult to scale for open-ended tasks in unstructured environments. LLMs offer a paradigm shift:

*   **High-level Reasoning:** LLMs can process abstract human instructions (e.g., "make coffee," "clean the table") and translate them into logical steps, leveraging their extensive pre-trained knowledge base.
*   **Task Decomposition:** Complex tasks can be broken down into a series of simpler, executable sub-tasks, a process that is often challenging to hardcode. LLMs can dynamically decompose tasks based on context.
*   **Common-Sense Knowledge:** LLMs implicitly encode a vast amount of common-sense knowledge about the world, which is crucial for handling unexpected situations or making inferences that are not explicitly programmed. This allows robots to move beyond rigid scripts.
*   **Adaptability:** LLMs can be prompted to adapt their planning based on new information, environmental changes, or user feedback, leading to more flexible robot behavior.

### Challenges: Grounding LLM outputs in physical reality, safety, computational cost

While promising, integrating LLMs into robotics presents significant challenges:

*   **Grounding LLM Outputs in Physical Reality:** LLMs operate in a linguistic space, not a physical one. Their outputs (textual plans) need to be "grounded" in the robot's physical capabilities and the real-world environment. This means translating abstract instructions into specific motor commands, navigation goals, or manipulation sequences. The robot's perception of the world must align with the LLM's understanding.
*   **Safety:** LLMs can sometimes generate unexpected, incorrect, or even unsafe instructions. Ensuring that an LLM-driven robot operates safely requires robust validation layers, human oversight, and fail-safe mechanisms.
*   **Computational Cost:** Running large LLMs, especially in real-time on edge devices, can be computationally expensive and may require significant hardware resources. Optimization and efficient inference techniques are critical.
*   **Ambiguity and Hallucination:** LLMs can hallucinate information or produce ambiguous instructions that are difficult for a robot to interpret reliably. Careful prompt engineering and feedback loops are necessary.
*   **Limited Real-time Feedback:** Standard LLMs do not inherently operate in real-time feedback loops with the physical world. Mechanisms are needed to allow the robot's current state and perception to influence the LLM's ongoing planning.

## 2.2 LLMs for Task Decomposition and High-Level Planning

One of the most powerful applications of LLMs in robotics is their ability to decompose high-level, abstract goals into a sequence of actionable steps.

### Using LLMs to break down complex natural language goals into a sequence of executable robot actions

Consider the goal: "Make me breakfast." This seemingly simple instruction involves many sub-tasks: go to the kitchen, identify breakfast items, retrieve ingredients, operate appliances, prepare food, serve it, and clean up. An LLM can help structure this process.

The general approach involves:

1.  **Receive High-Level Goal:** The user provides a natural language instruction (e.g., "Clean the room").
2.  **Prompt LLM for Plan:** The LLM is prompted with the goal, along with information about the robot's capabilities (available tools, actions) and the current state of the environment.
3.  **Generate Plan Steps:** The LLM outputs a textual plan, breaking down the complex goal into a sequence of simpler steps. These steps might be abstract (e.g., "Go to the table") or more specific (e.g., "Pick up the red cup").
4.  **Execute Steps Sequentially:** Each step of the plan is then translated into robot-executable actions.

### Example: "Clean the room" -> "Go to table", "Pick up cup", "Put cup in sink"

Let's imagine a robot is tasked with "Clean the room." A well-engineered prompt could elicit a plan similar to this:

**Prompt to LLM:**
```
You are a helpful robot assistant. Your goal is to generate a step-by-step plan for a robot to perform a task.
The robot can perform the following actions:
- navigate_to(location): Move to a specified location.
- pick_up(object): Pick up an object.
- place_object(object, location): Place an object at a specified location.
- detect_objects(): Scan the environment for objects.

Current environment state:
- The room contains a table, a couch, and a sink.
- On the table: a red cup, a book.
- On the couch: a blue pillow.

Task: Clean the room.

Generate a numbered list of actions the robot should take to clean the room.
```

**LLM Response (Example):**
```
1. navigate_to(table)
2. pick_up(red cup)
3. navigate_to(sink)
4. place_object(red cup, sink)
5. navigate_to(table)
6. pick_up(book)
7. navigate_to(shelf) # Assuming 'shelf' is a valid location for books
8. place_object(book, shelf)
9. navigate_to(couch)
10. pick_up(blue pillow)
11. place_object(blue pillow, couch_original_position) # Or a designated pillow area
12. navigate_to(charging_station) # Go to idle or charge
```

This plan then needs to be parsed and executed by the robot's control system.

## 2.3 Prompt Engineering for Robot Control

The quality of an LLM's plan heavily depends on the quality of the prompt. Effective prompt engineering is key to guiding the LLM to generate safe, feasible, and relevant robot actions.

### Designing effective prompts to guide LLMs towards generating safe and feasible plans

Key strategies for prompt engineering:

*   **Define Robot Capabilities:** Clearly state the actions the robot can perform, including their arguments and expected outcomes.
*   **Provide Contextual Information:** Include relevant details about the robot's current state, the environment, and the goal. The more specific and up-to-date this information, the better.
*   **Specify Output Format:** Instruct the LLM to provide the plan in a structured, easily parsable format (e.g., a numbered list of function calls).
*   **Include Constraints and Safety Guidelines:** Explicitly tell the LLM about safety constraints (e.g., "Do not approach humans too closely," "Avoid dropping objects").
*   **Few-Shot Learning:** Provide examples of desired task decompositions to guide the LLM's behavior.
*   **Iterative Refinement:** If the initial plan is not satisfactory, provide feedback to the LLM and ask it to refine the plan.

### Providing a prompt engineering example showing how to translate "Clean the room" into a sequence of actions

(This has been demonstrated in the previous section's example)

## 2.4 Grounding LLM Plans into Robot Executable Actions

Once an LLM generates a textual plan, the robot needs to convert these abstract steps into concrete, low-level commands that its hardware can execute. This is the "grounding" problem.

### Mapping LLM-generated high-level actions to specific robot APIs (ROS 2 services, actions, topics)

Each action in the LLM's plan (e.g., `navigate_to(location)`, `pick_up(object)`) must correspond to a specific robot API:

*   **`navigate_to(location)`:** Might map to a [ROS 2](pathname:///docs/module1-ros2/introduction) Action Goal for the [Nav2](pathname:///docs/module3-isaac/vslam-navigation) stack (`NavigateToPose`). The `location` would be translated into a `geometry_msgs/PoseStamped`.
*   **`pick_up(object)`:** Could map to a [ROS 2](pathname:///docs/module1-ros2/introduction) Service call to a manipulation controller (e.g., a custom service like `gripper_control/srv/PickUpObject`). The `object` would need to be localized using perception and its pose passed as a parameter.
*   **`place_object(object, location)`:** Similar to `pick_up`, this would invoke a manipulation service or action, with `object` and `location` translated into target poses.
*   **`detect_objects()`:** Might trigger a perception system (e.g., an Isaac ROS DNN node) via a ROS 2 Service call, which then publishes detected objects to a topic.

This mapping layer acts as an interpreter, translating the LLM's linguistic instructions into the robot's operational language.

### Handling ambiguities and failure cases: asking for clarification, replanning

Real-world robot operation is prone to uncertainties. The grounding layer must handle:

*   **Ambiguity:** If the LLM's instruction is vague (e.g., "get the thing"), the robot might need to ask for clarification from the user or use its perception to infer intent.
*   **Failure Cases:** If a planned action fails (e.g., `pick_up(object)` fails because the object is too heavy or not reachable), the robot needs mechanisms to:
    *   **Report Failure:** Inform the user.
    *   **Replanning:** Ask the LLM to generate an alternative plan, potentially with updated environmental information or new constraints.
    *   **Error Recovery:** Execute predefined error recovery routines.

This iterative feedback loop, where the robot's real-world experiences inform and refine the LLM's cognitive planning, is crucial for developing truly intelligent and robust robotic systems.
