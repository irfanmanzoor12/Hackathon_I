import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

export default function NotFound() {
  return (
    <Layout title="Physical AI & Humanoid Robotics.">
      <main style={{
        padding: '4rem 2rem',
        maxWidth: '900px',
        margin: '0 auto',
      }}>
        <div style={{ textAlign: 'center', marginBottom: '3rem' }}>
          <h1 style={{ fontSize: '2.5rem', marginBottom: '1rem' }}>
            Oops! Page Not Found
          </h1>
          <p style={{ fontSize: '1.1rem', color: '#888', marginBottom: '2rem' }}>
            Looks like this robot took a wrong turn! Let's get you back on track with Physical AI & Humanoid Robotics.
          </p>
          <Link
            to="/intro"
            style={{
              display: 'inline-block',
              padding: '14px 28px',
              backgroundColor: '#3b82f6',
              color: 'white',
              borderRadius: '8px',
              textDecoration: 'none',
              fontWeight: 'bold',
              fontSize: '1.1rem',
            }}
          >
            Start Learning
          </Link>
        </div>

        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
          gap: '1.5rem',
          marginTop: '3rem',
        }}>
          <ModuleCard
            title="Module 1: ROS 2"
            description="Learn the Robotic Nervous System - nodes, topics, services, and communication pipelines."
            link="/module-1/introduction"
            weeks="Weeks 1-3"
            color="#10b981"
          />
          <ModuleCard
            title="Module 2: Digital Twin"
            description="Master Gazebo & Unity for physics simulation and robot visualization."
            link="/module-2/introduction"
            weeks="Weeks 4-7"
            color="#8b5cf6"
          />
          <ModuleCard
            title="Module 3: NVIDIA Isaac"
            description="Explore AI-powered perception, navigation, and manipulation."
            link="/module-3/introduction"
            weeks="Weeks 8-10"
            color="#f59e0b"
          />
          <ModuleCard
            title="Module 4: VLA"
            description="Integrate Vision-Language-Action models for autonomous humanoids."
            link="/module-4/introduction"
            weeks="Weeks 11-13"
            color="#ef4444"
          />
        </div>

        <div style={{
          marginTop: '3rem',
          padding: '2rem',
          backgroundColor: 'var(--ifm-background-surface-color)',
          borderRadius: '12px',
          border: '1px solid var(--ifm-color-emphasis-200)',
        }}>
          <h3 style={{ marginBottom: '1rem' }}>What You'll Build</h3>
          <ul style={{ lineHeight: '2', paddingLeft: '1.5rem' }}>
            <li>Autonomous humanoid robot using ROS 2</li>
            <li>Digital twin simulations in Gazebo and Unity</li>
            <li>AI-powered perception with NVIDIA Isaac</li>
            <li>Voice-controlled robots with Vision-Language-Action models</li>
          </ul>
        </div>
      </main>
    </Layout>
  );
}

function ModuleCard({ title, description, link, weeks, color }) {
  return (
    <Link
      to={link}
      style={{
        display: 'block',
        padding: '1.5rem',
        backgroundColor: 'var(--ifm-background-surface-color)',
        borderRadius: '12px',
        border: '1px solid var(--ifm-color-emphasis-200)',
        textDecoration: 'none',
        color: 'inherit',
        transition: 'transform 0.2s, box-shadow 0.2s',
      }}
      onMouseOver={(e) => {
        e.currentTarget.style.transform = 'translateY(-4px)';
        e.currentTarget.style.boxShadow = '0 10px 30px rgba(0,0,0,0.1)';
      }}
      onMouseOut={(e) => {
        e.currentTarget.style.transform = 'translateY(0)';
        e.currentTarget.style.boxShadow = 'none';
      }}
    >
      <div style={{
        display: 'inline-block',
        padding: '4px 12px',
        backgroundColor: color,
        color: 'white',
        borderRadius: '4px',
        fontSize: '0.8rem',
        fontWeight: 'bold',
        marginBottom: '0.75rem',
      }}>
        {weeks}
      </div>
      <h4 style={{ margin: '0.5rem 0', fontSize: '1.1rem' }}>{title}</h4>
      <p style={{ margin: 0, color: '#888', fontSize: '0.9rem' }}>{description}</p>
    </Link>
  );
}
