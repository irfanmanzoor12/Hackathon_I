---
sidebar_position: 5
title: URDF Robot Description
---

# URDF: Universal Robot Description Format

URDF is the XML format for describing robot kinematics, dynamics, and visual properties.

## 🎯 Basic Structure

```xml
<robot name="humanoid">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Joint Types

- **fixed**: No movement
- **revolute**: Rotation with limits
- **continuous**: Unlimited rotation
- **prismatic**: Linear sliding

---

**Personalize or translate this chapter!**
