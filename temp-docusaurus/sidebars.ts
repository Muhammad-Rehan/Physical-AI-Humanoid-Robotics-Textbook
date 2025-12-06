import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro/introduction'],
    },
    {
      type: 'category',
      label: 'Getting Started',
      items: ['getting-started/ros2-setup'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2',
      items: [
        'module1-ros2/introduction',
        'module1-ros2/python-agents-to-controllers',
        'module1-ros2/understanding-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation',
      items: [
        'module2-gazebo/physics-simulation',
        'module2-gazebo/sensor-simulation',
        'module2-gazebo/unity-rendering',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac & VSLAM',
      items: [
        'module3-isaac/isaac-sim',
        'module3-isaac/vslam-navigation',
        'module3-isaac/path-planning-nav2',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module4-vla/voice-to-action',
        'module4-vla/cognitive-planning-llm',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['module4-vla/capstone-project'],
    },
    {
      type: 'category',
      label: 'Resources',
      items: ['resources/resources'],
    },
  ],
};

export default sidebars;
