import React from 'react';
import './App.css';
import RAGChatWidget from './components/RAGChatWidget/RAGChatWidget';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1>Chapter 1: Physical AI & Humanoid Robotics</h1>
        <p>Test page for RAG Chatbot Widget</p>
      </header>

      <main className="App-content">
        <section className="chapter-content">
          <h2>Introduction to Physical AI</h2>
          <p>
            Physical AI represents artificial intelligence systems that exist in and interact with the physical world.
            Unlike traditional AI that operates purely in digital spaces, Physical AI must understand physics, navigate real environments,
            and manipulate objects. This requires embodied intelligence - AI integrated with robotic systems that can perceive,
            reason about, and act upon the physical world.
          </p>

          <h2>ROS 2 Fundamentals</h2>
          <p>
            ROS 2 (Robot Operating System 2) is the middleware framework that provides communication infrastructure
            for robotics applications. It uses a distributed architecture where processes communicate via nodes. Nodes are independent
            processes that perform specific tasks and communicate using topics (publish-subscribe), services (request-response),
            and actions (long-running tasks with feedback).
          </p>

          <h2>Gazebo Simulation</h2>
          <p>
            Gazebo is an open-source physics simulator for robotics. It simulates rigid body dynamics, sensor models
            (LiDAR, cameras, IMU, depth), and environmental conditions. Gazebo integrates with ROS 2 via gazebo_ros packages,
            allowing robots to be tested in simulation before physical deployment.
          </p>

          <h2>NVIDIA Isaac</h2>
          <p>
            NVIDIA Isaac Sim is a scalable robotics simulation platform built on NVIDIA Omniverse.
            It provides photorealistic rendering, accurate physics simulation, and synthetic data generation for AI training.
            Isaac Sim supports ROS 2 integration and can simulate complex environments with multiple robots.
          </p>

          <div className="demo-instructions">
            <h3>Try the RAG Chatbot:</h3>
            <ul>
              <li>Click the blue button in the bottom-right corner</li>
              <li>Sign in with Google</li>
              <li>Ask questions about Physical AI, ROS 2, or robotics</li>
              <li>Try selecting text above and asking questions about it</li>
              <li>Use the "Personalize" and "Translate" buttons</li>
            </ul>
          </div>
        </section>
      </main>

      {/* RAG Chat Widget */}
      <RAGChatWidget />
    </div>
  );
}

export default App;
