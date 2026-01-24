/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Welcome',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/introduction',
        'module-1/ros2-fundamentals',
        'module-1/nodes-topics-services',
        'module-1/python-rclpy',
        'module-1/urdf-robot-description',
        'module-1/module-1-project',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/introduction',
        'module-2/physics-simulation',
        'module-2/gazebo-basics',
        'module-2/unity-integration',
        'module-2/sensor-simulation',
        'module-2/module-2-project',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/introduction',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/introduction',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware/overview',
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'resources/setup-guides',
      ],
    },
  ],
};

module.exports = sidebars;
