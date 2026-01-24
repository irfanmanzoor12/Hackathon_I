"""
Chapter 1 Content Indexing Script
Indexes Physical AI & Humanoid Robotics content into Qdrant
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings
from app.database.qdrant import upsert_documents, init_qdrant
from app.chat.rag_engine import rag_engine


# Chapter 1 Content - Physical AI & Humanoid Robotics
CHAPTER_1_CONTENT = [
    # Week 1: Introduction to Physical AI
    {
        "content": """Physical AI represents artificial intelligence systems that exist in and interact with the physical world.
        Unlike traditional AI that operates purely in digital spaces, Physical AI must understand physics, navigate real environments,
        and manipulate objects. This requires embodied intelligence - AI integrated with robotic systems that can perceive,
        reason about, and act upon the physical world.""",
        "metadata": {
            "section": "Introduction to Physical AI",
            "week": "1",
            "topic": "Embodied Intelligence"
        }
    },
    {
        "content": """Humanoid robotics involves creating robots with human-like form factors, enabling them to operate in
        environments designed for humans. Key challenges include bipedal locomotion, dexterous manipulation, human-robot interaction,
        and adaptive behavior in unstructured environments.""",
        "metadata": {
            "section": "Introduction to Physical AI",
            "week": "1",
            "topic": "Humanoid Robotics"
        }
    },

    # Weeks 2-4: ROS 2 Fundamentals
    {
        "content": """ROS 2 (Robot Operating System 2) is the middleware framework that provides communication infrastructure
        for robotics applications. It uses a distributed architecture where processes communicate via nodes. Nodes are independent
        processes that perform specific tasks and communicate using topics (publish-subscribe), services (request-response),
        and actions (long-running tasks with feedback).""",
        "metadata": {
            "section": "ROS 2 Fundamentals",
            "week": "2-4",
            "topic": "ROS 2 Architecture"
        }
    },
    {
        "content": """ROS 2 topics enable asynchronous, one-to-many communication using the publish-subscribe pattern.
        Publishers send messages on named topics, and subscribers receive them. Common message types include sensor_msgs
        (LiDAR, cameras), geometry_msgs (poses, velocities), and custom messages defined in .msg files.""",
        "metadata": {
            "section": "ROS 2 Fundamentals",
            "week": "2-4",
            "topic": "ROS 2 Topics"
        }
    },
    {
        "content": """URDF (Unified Robot Description Format) is an XML format for describing robot kinematics and dynamics.
        It defines links (rigid bodies), joints (connections between links), sensors, and visual/collision geometry.
        URDF files are essential for simulation and visualization in Gazebo and RViz.""",
        "metadata": {
            "section": "ROS 2 Fundamentals",
            "week": "2-4",
            "topic": "URDF"
        }
    },
    {
        "content": """rclpy is the Python client library for ROS 2. It provides APIs for creating nodes, publishers, subscribers,
        services, and actions. Example: rclpy.init() initializes the ROS 2 context, Node() creates a node,
        create_publisher() creates a publisher, and spin() keeps the node running.""",
        "metadata": {
            "section": "ROS 2 Fundamentals",
            "week": "2-4",
            "topic": "rclpy Programming"
        }
    },

    # Weeks 5-7: Digital Twins (Gazebo & Unity)
    {
        "content": """Gazebo is an open-source physics simulator for robotics. It simulates rigid body dynamics, sensor models
        (LiDAR, cameras, IMU, depth), and environmental conditions. Gazebo integrates with ROS 2 via gazebo_ros packages,
        allowing robots to be tested in simulation before physical deployment.""",
        "metadata": {
            "section": "Digital Twins",
            "week": "5-7",
            "topic": "Gazebo Simulation"
        }
    },
    {
        "content": """Unity is a real-time 3D development platform increasingly used for robotics simulation. Unity Robotics Hub
        provides ROS integration, enabling high-fidelity rendering, synthetic data generation for ML training, and realistic
        physics simulation. Unity excels at visual simulation and can generate photorealistic synthetic datasets.""",
        "metadata": {
            "section": "Digital Twins",
            "week": "5-7",
            "topic": "Unity Simulation"
        }
    },
    {
        "content": """LiDAR (Light Detection and Ranging) sensors emit laser pulses and measure return time to create 3D point clouds.
        In simulation, LiDAR is modeled with ray casting. Common ROS message type: sensor_msgs/PointCloud2.
        Used for SLAM, obstacle detection, and navigation.""",
        "metadata": {
            "section": "Digital Twins",
            "week": "5-7",
            "topic": "Sensor Simulation - LiDAR"
        }
    },
    {
        "content": """Depth cameras (like Intel RealSense) provide RGB-D data: color images plus per-pixel depth information.
        They enable 3D perception, object detection, and manipulation planning. ROS message types: sensor_msgs/Image (RGB),
        sensor_msgs/Image (depth), sensor_msgs/PointCloud2 (3D points).""",
        "metadata": {
            "section": "Digital Twins",
            "week": "5-7",
            "topic": "Sensor Simulation - Depth Cameras"
        }
    },
    {
        "content": """IMU (Inertial Measurement Unit) sensors measure acceleration and angular velocity. They're crucial for
        robot pose estimation and balance control in humanoid robots. ROS message type: sensor_msgs/Imu.
        Simulated IMUs add realistic noise models to match physical sensors.""",
        "metadata": {
            "section": "Digital Twins",
            "week": "5-7",
            "topic": "Sensor Simulation - IMU"
        }
    },

    # Weeks 8-10: NVIDIA Isaac
    {
        "content": """NVIDIA Isaac Sim is a scalable robotics simulation platform built on NVIDIA Omniverse.
        It provides photorealistic rendering, accurate physics simulation, and synthetic data generation for AI training.
        Isaac Sim supports ROS 2 integration and can simulate complex environments with multiple robots.""",
        "metadata": {
            "section": "NVIDIA Isaac",
            "week": "8-10",
            "topic": "Isaac Sim"
        }
    },
    {
        "content": """Isaac ROS provides GPU-accelerated ROS 2 packages for perception, navigation, and manipulation.
        Key packages include: isaac_ros_visual_slam (visual odometry), isaac_ros_nvblox (3D reconstruction),
        isaac_ros_dnn_inference (deep learning inference). These leverage NVIDIA GPUs for real-time performance.""",
        "metadata": {
            "section": "NVIDIA Isaac",
            "week": "8-10",
            "topic": "Isaac ROS"
        }
    },
    {
        "content": """Nav2 (Navigation 2) is the ROS 2 navigation stack. It provides autonomous navigation capabilities including
        path planning (using A*, Dijkstra, or other algorithms), obstacle avoidance, localization, and behavior trees.
        Nav2 integrates with costmaps that represent obstacle information from sensors.""",
        "metadata": {
            "section": "NVIDIA Isaac",
            "week": "8-10",
            "topic": "Nav2 Navigation"
        }
    },
    {
        "content": """Perception pipelines in robotics involve sensor data processing, object detection, pose estimation,
        and scene understanding. NVIDIA Isaac provides GPU-accelerated perception using deep learning models.
        Common tasks: object detection (YOLO, Faster R-CNN), segmentation, 3D pose estimation.""",
        "metadata": {
            "section": "NVIDIA Isaac",
            "week": "8-10",
            "topic": "AI Perception"
        }
    },

    # Weeks 11-13: Vision-Language-Action (VLA)
    {
        "content": """Vision-Language-Action (VLA) models combine computer vision, natural language processing, and robotic control.
        They enable robots to understand multimodal instructions like "Pick up the red cup on the table" and execute corresponding
        actions. VLA models bridge high-level human commands with low-level motor control.""",
        "metadata": {
            "section": "VLA Integration",
            "week": "11-13",
            "topic": "VLA Models"
        }
    },
    {
        "content": """OpenAI Whisper is a robust speech recognition model that converts voice commands to text.
        In robotics, Whisper enables voice-controlled interfaces. Integration: audio input → Whisper API → text transcript →
        robot command parser → action execution. Supports multilingual recognition.""",
        "metadata": {
            "section": "VLA Integration",
            "week": "11-13",
            "topic": "Voice Commands - Whisper"
        }
    },
    {
        "content": """Cognitive planning in robotics involves task decomposition, sequential reasoning, and adaptive execution.
        Modern approaches use Large Language Models (LLMs) like GPT-4 for high-level planning: breaking complex tasks into
        subtasks, handling exceptions, and reasoning about object affordances and spatial relationships.""",
        "metadata": {
            "section": "VLA Integration",
            "week": "11-13",
            "topic": "Cognitive Planning"
        }
    },
    {
        "content": """The capstone humanoid project integrates all quarter components: ROS 2 communication, Gazebo simulation,
        NVIDIA Isaac perception, and VLA-based control. Students build a complete pipeline where a humanoid robot receives
        voice commands, plans actions using LLMs, navigates environments, and manipulates objects - demonstrating end-to-end
        Physical AI capabilities.""",
        "metadata": {
            "section": "VLA Integration",
            "week": "11-13",
            "topic": "Capstone Project"
        }
    },

    # Additional Technical Concepts
    {
        "content": """SLAM (Simultaneous Localization and Mapping) is the problem of building a map of an unknown environment
        while simultaneously tracking the robot's location within it. Key algorithms: EKF-SLAM, FastSLAM, ORB-SLAM.
        In ROS 2, SLAM is typically implemented using packages like slam_toolbox or Cartographer.""",
        "metadata": {
            "section": "Advanced Topics",
            "week": "8-10",
            "topic": "SLAM"
        }
    },
    {
        "content": """Kinematics involves computing robot motion without considering forces. Forward kinematics: given joint angles,
        compute end-effector pose. Inverse kinematics: given desired end-effector pose, compute required joint angles.
        Essential for manipulation and locomotion control.""",
        "metadata": {
            "section": "Advanced Topics",
            "week": "2-4",
            "topic": "Robot Kinematics"
        }
    },
    {
        "content": """Control theory for robotics includes PID control, model predictive control (MPC), and adaptive control.
        PID controllers are commonly used for joint control. MPC is used for trajectory optimization.
        ROS 2 provides ros2_control framework for implementing robot controllers.""",
        "metadata": {
            "section": "Advanced Topics",
            "week": "2-4",
            "topic": "Robot Control"
        }
    },
]


async def generate_embeddings(texts: list[str]) -> list[list[float]]:
    """Generate embeddings using HYBRID system (Local or OpenAI)"""
    print(f"  Generating embeddings for {len(texts)} sections...")
    print(f"  Using: {settings.EMBEDDING_PROVIDER.upper()} embeddings")

    embeddings = []
    for text in texts:
        embedding = await rag_engine.generate_embedding(text)
        embeddings.append(embedding)

    return embeddings


async def index_chapter_content():
    """Index all Chapter 1 content into Qdrant"""
    print("=" * 60)
    print("Chapter 1 Content Indexing")
    print("=" * 60)

    # Initialize Qdrant
    print("\nInitializing Qdrant connection...")
    await init_qdrant()

    # Prepare content
    documents = []
    for item in CHAPTER_1_CONTENT:
        doc = {
            "content": item["content"],
            "section": item["metadata"]["section"],
            "week": item["metadata"]["week"],
            "topic": item["metadata"]["topic"],
            "metadata": item["metadata"]
        }
        documents.append(doc)

    print(f"\nPrepared {len(documents)} content sections")

    # Generate embeddings
    print("\nGenerating embeddings...")
    texts = [doc["content"] for doc in documents]
    embeddings = await generate_embeddings(texts)

    # Upsert to Qdrant
    print("\nUploading to Qdrant...")
    upsert_documents(documents, embeddings)

    print("\n" + "=" * 60)
    print("SUCCESS: Chapter 1 Content Indexed!")
    print("=" * 60)
    print(f"\nTotal sections indexed: {len(documents)}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"Vector dimension: {settings.VECTOR_DIMENSION}")
    print("\nReady for RAG queries!")


if __name__ == "__main__":
    try:
        asyncio.run(index_chapter_content())
    except KeyboardInterrupt:
        print("\n\nIndexing interrupted by user")
    except Exception as e:
        print(f"\n\nError during indexing: {str(e)}")
        raise
