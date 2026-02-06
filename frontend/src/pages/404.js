import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function NotFound() {
  return (
    <Layout title="Page Not Found">
      <main style={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '60vh',
        padding: '2rem',
        textAlign: 'center'
      }}>
        <h1 style={{ fontSize: '3rem', marginBottom: '1rem' }}>
          Page Not Found
        </h1>

        <p style={{ fontSize: '1.2rem', color: '#666', marginBottom: '2rem', maxWidth: '600px' }}>
          The page you're looking for doesn't exist. But don't worry - there's plenty to learn about Physical AI and Humanoid Robotics!
        </p>

        <div style={{
          display: 'flex',
          gap: '1rem',
          flexWrap: 'wrap',
          justifyContent: 'center',
          marginBottom: '3rem'
        }}>
          <Link
            to="/intro"
            style={{
              padding: '12px 24px',
              backgroundColor: '#3b82f6',
              color: 'white',
              borderRadius: '8px',
              textDecoration: 'none',
              fontWeight: 'bold'
            }}
          >
            Start Learning
          </Link>
          <Link
            to="/module-1/introduction"
            style={{
              padding: '12px 24px',
              backgroundColor: '#10b981',
              color: 'white',
              borderRadius: '8px',
              textDecoration: 'none',
              fontWeight: 'bold'
            }}
          >
            Module 1: ROS 2
          </Link>
        </div>

        <div style={{
          backgroundColor: '#f3f4f6',
          padding: '2rem',
          borderRadius: '12px',
          maxWidth: '600px'
        }}>
          <h3 style={{ marginBottom: '1rem' }}>Course Modules</h3>
          <ul style={{ textAlign: 'left', lineHeight: '2' }}>
            <li><Link to="/module-1/introduction">Module 1: The Robotic Nervous System (ROS 2)</Link></li>
            <li><Link to="/module-2/introduction">Module 2: The Digital Twin (Gazebo & Unity)</Link></li>
            <li><Link to="/module-3/introduction">Module 3: The AI-Robot Brain (NVIDIA Isaac)</Link></li>
            <li><Link to="/module-4/introduction">Module 4: Vision-Language-Action (VLA)</Link></li>
          </ul>
        </div>
      </main>
    </Layout>
  );
}
