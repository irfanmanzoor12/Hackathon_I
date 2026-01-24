/**
 * Agentic Avatar - Animated AI Assistant
 * Features: Eye movements, mouth animations, head bobbing
 */

import React, { useEffect, useState } from 'react';
import { motion } from 'framer-motion';
import './AgenticAvatar.css';

const AgenticAvatar = ({ state = 'idle', size = 'medium' }) => {
  const [eyePosition, setEyePosition] = useState({ x: 0, y: 0 });
  const [blinkState, setBlinkState] = useState(false);

  // Realistic eye blinking
  useEffect(() => {
    const blinkInterval = setInterval(() => {
      setBlinkState(true);
      setTimeout(() => setBlinkState(false), 150);
    }, 3000 + Math.random() * 2000);

    return () => clearInterval(blinkInterval);
  }, []);

  // Random eye movements (idle state)
  useEffect(() => {
    if (state === 'idle') {
      const eyeMovement = setInterval(() => {
        setEyePosition({
          x: (Math.random() - 0.5) * 4,
          y: (Math.random() - 0.5) * 3
        });
      }, 2000);

      return () => clearInterval(eyeMovement);
    } else {
      // Look forward when active
      setEyePosition({ x: 0, y: 0 });
    }
  }, [state]);

  const sizeMap = {
    tiny: 30,
    small: 50,
    medium: 80,
    large: 120
  };

  const avatarSize = sizeMap[size] || 80;

  return (
    <motion.div
      className={`agentic-avatar agentic-avatar-${size}`}
      style={{ width: avatarSize, height: avatarSize }}
      animate={{
        rotate: state === 'thinking' ? [0, -5, 5, 0] : 0,
        y: state === 'listening' ? [0, -2, 0] : 0
      }}
      transition={{
        rotate: { duration: 1.5, repeat: state === 'thinking' ? Infinity : 0 },
        y: { duration: 1, repeat: state === 'listening' ? Infinity : 0 }
      }}
    >
      <svg
        viewBox="0 0 100 100"
        width={avatarSize}
        height={avatarSize}
        className="avatar-svg"
      >
        {/* Head circle with gradient */}
        <defs>
          <linearGradient id="headGradient" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" style={{ stopColor: '#3B82F6', stopOpacity: 1 }} />
            <stop offset="100%" style={{ stopColor: '#1D4ED8', stopOpacity: 1 }} />
          </linearGradient>

          <radialGradient id="glowGradient">
            <stop offset="0%" style={{ stopColor: '#60A5FA', stopOpacity: 0.8 }} />
            <stop offset="100%" style={{ stopColor: '#3B82F6', stopOpacity: 0 }} />
          </radialGradient>
        </defs>

        {/* Glow effect */}
        <motion.circle
          cx="50"
          cy="50"
          r="45"
          fill="url(#glowGradient)"
          animate={{
            opacity: state === 'thinking' ? [0.3, 0.6, 0.3] : 0.2
          }}
          transition={{ duration: 2, repeat: Infinity }}
        />

        {/* Head */}
        <motion.circle
          cx="50"
          cy="50"
          r="35"
          fill="url(#headGradient)"
          animate={{
            scale: state === 'responding' ? [1, 1.02, 1] : 1
          }}
          transition={{ duration: 0.8, repeat: state === 'responding' ? Infinity : 0 }}
        />

        {/* Left Eye */}
        <motion.g
          animate={{
            x: eyePosition.x,
            y: eyePosition.y
          }}
          transition={{ duration: 0.3 }}
        >
          <ellipse
            cx="38"
            cy="45"
            rx="4"
            ry={blinkState ? 0.5 : 6}
            fill="white"
          />
          <circle cx="38" cy="45" r="2" fill="#1E293B" />
        </motion.g>

        {/* Right Eye */}
        <motion.g
          animate={{
            x: eyePosition.x,
            y: eyePosition.y
          }}
          transition={{ duration: 0.3 }}
        >
          <ellipse
            cx="62"
            cy="45"
            rx="4"
            ry={blinkState ? 0.5 : 6}
            fill="white"
          />
          <circle cx="62" cy="45" r="2" fill="#1E293B" />
        </motion.g>

        {/* Mouth - changes based on state */}
        <motion.path
          d={getMouthPath(state)}
          stroke="white"
          strokeWidth="2"
          fill="none"
          strokeLinecap="round"
          animate={{
            d: state === 'responding'
              ? [getMouthPath('responding'), getMouthPath('idle'), getMouthPath('responding')]
              : getMouthPath(state)
          }}
          transition={{ duration: 0.5, repeat: state === 'responding' ? Infinity : 0 }}
        />

        {/* Thinking indicator - rotating dots */}
        {state === 'thinking' && (
          <>
            {[0, 120, 240].map((angle, idx) => (
              <motion.circle
                key={idx}
                cx={50 + Math.cos((angle * Math.PI) / 180) * 25}
                cy={50 + Math.sin((angle * Math.PI) / 180) * 25}
                r="2"
                fill="#60A5FA"
                animate={{
                  opacity: [0.3, 1, 0.3],
                  scale: [0.8, 1.2, 0.8]
                }}
                transition={{
                  duration: 1.5,
                  repeat: Infinity,
                  delay: idx * 0.2
                }}
              />
            ))}
          </>
        )}

        {/* Listening waves */}
        {state === 'listening' && (
          <>
            {[1, 2, 3].map((i) => (
              <motion.circle
                key={i}
                cx="50"
                cy="50"
                r="35"
                stroke="#60A5FA"
                strokeWidth="1"
                fill="none"
                initial={{ scale: 1, opacity: 0.6 }}
                animate={{
                  scale: 1.5,
                  opacity: 0
                }}
                transition={{
                  duration: 2,
                  repeat: Infinity,
                  delay: i * 0.4
                }}
              />
            ))}
          </>
        )}
      </svg>

      {/* State label (optional for debugging) */}
      {size === 'large' && (
        <div className="state-label">{state}</div>
      )}
    </motion.div>
  );
};

// Get mouth path based on state
const getMouthPath = (state) => {
  switch (state) {
    case 'idle':
      return 'M 40 62 Q 50 66 60 62'; // Slight smile
    case 'listening':
      return 'M 40 62 Q 50 58 60 62'; // Open mouth
    case 'thinking':
      return 'M 40 60 L 60 60'; // Neutral
    case 'responding':
      return 'M 40 58 Q 50 65 60 58'; // Wide smile
    default:
      return 'M 40 62 Q 50 66 60 62';
  }
};

export default AgenticAvatar;
