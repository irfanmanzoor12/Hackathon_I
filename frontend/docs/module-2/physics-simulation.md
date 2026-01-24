---
sidebar_position: 2
title: "Physics Simulation Concepts"
---

# Physics Simulation Concepts

## Fundamentals of Physics Simulation

Physics simulation computes how objects move, interact, and respond to forces. At its core, simulation integrates Newton's laws of motion over discrete time steps.

### Core Physics Equations

#### Linear Motion

```
F = m * a  (Force = mass × acceleration)

a = dv/dt  (acceleration = rate of change of velocity)
v = dp/dt  (velocity = rate of change of position)

Rearranged for simulation:
v(t+Δt) = v(t) + a*Δt
p(t+Δt) = p(t) + v*Δt
```

#### Rotational Motion

```
τ = I * α  (Torque = inertia × angular acceleration)

α = dω/dt  (angular acceleration = rate of change of angular velocity)
ω = dq/dt  (angular velocity = rate of change of orientation)

Rearranged for simulation:
ω(t+Δt) = ω(t) + α*Δt
q(t+Δt) = q(t) + ω*Δt * q(t)  [Quaternion update]
```

## Integration Methods

### Euler Method (Simple but Less Accurate)

```python
# Explicit Euler Integration
def euler_integration(state, forces, dt):
    position, velocity, orientation, angular_velocity = state

    # Calculate accelerations from forces
    linear_accel = forces.linear / mass
    angular_accel = forces.angular / inertia_matrix

    # Update velocities
    new_velocity = velocity + linear_accel * dt
    new_angular_velocity = angular_velocity + angular_accel * dt

    # Update positions
    new_position = position + new_velocity * dt
    new_orientation = orientation + angular_velocity * dt

    return (new_position, new_velocity, new_orientation, new_angular_velocity)
```

**Characteristics:**
- Simple to implement
- Fast computation
- Less accurate
- Can be unstable with large time steps
- Good for teaching and prototyping

### Runge-Kutta Method (More Accurate)

```python
# RK4 Integration
def rk4_integration(state, forces, dt):
    """4th order Runge-Kutta integration"""

    def derivatives(state, forces):
        position, velocity, orientation, angular_velocity = state
        linear_accel = forces.linear / mass
        angular_accel = forces.angular / inertia_matrix
        return (velocity, linear_accel, angular_velocity, angular_accel)

    # RK4 stages
    k1 = derivatives(state, forces)

    state_temp = add_scaled_state(state, k1, 0.5 * dt)
    k2 = derivatives(state_temp, forces)

    state_temp = add_scaled_state(state, k2, 0.5 * dt)
    k3 = derivatives(state_temp, forces)

    state_temp = add_scaled_state(state, k3, dt)
    k4 = derivatives(state_temp, forces)

    # Combine k terms
    k_avg = average_states([k1, k2, k3, k4], [1, 2, 2, 1])
    new_state = add_scaled_state(state, k_avg, dt / 6.0)

    return new_state
```

**Characteristics:**
- More accurate than Euler
- Slightly higher computational cost
- Better stability
- Standard in modern simulators

## Force Models

### Gravity

```python
class GravityForce:
    def __init__(self, g=9.81):
        self.g = g

    def calculate_force(self, mass):
        # F = m * g (pointing downward)
        return (0, 0, -mass * self.g)
```

### Friction

#### Static Friction
Maximum static friction before sliding begins:

```
f_max = μ_s * N

Where:
f_max = Maximum static friction force
μ_s = Static friction coefficient
N = Normal force (perpendicular to surface)
```

#### Kinetic Friction
Friction during sliding:

```
f = μ_k * N * v_hat

Where:
μ_k = Kinetic friction coefficient (typically < μ_s)
v_hat = Unit vector in direction of sliding velocity
```

#### Implementation

```python
class FrictionForce:
    def __init__(self, mu_static=0.5, mu_kinetic=0.3):
        self.mu_s = mu_static
        self.mu_k = mu_kinetic

    def calculate_friction(self, normal_force, velocity, threshold=0.01):
        """Calculate friction force"""
        import numpy as np

        speed = np.linalg.norm(velocity)

        # Static friction (object stationary or nearly so)
        if speed < threshold:
            # Return force that opposes applied force, up to max static friction
            return np.zeros_like(velocity)

        # Kinetic friction (object sliding)
        else:
            # Friction opposes direction of motion
            friction_magnitude = self.mu_k * normal_force
            friction_direction = -velocity / speed
            return friction_magnitude * friction_direction
```

### Damping (Air Resistance, Viscous Friction)

```
f_damping = -c * v

Where:
c = Damping coefficient
v = Velocity

For angular motion:
τ_damping = -c_angular * ω
```

#### Implementation

```python
class DampingForce:
    def __init__(self, linear_damping=0.1, angular_damping=0.1):
        self.c_linear = linear_damping
        self.c_angular = angular_damping

    def calculate_damping(self, velocity, angular_velocity):
        """Calculate damping forces and torques"""
        linear_damping_force = -self.c_linear * velocity
        angular_damping_torque = -self.c_angular * angular_velocity

        return linear_damping_force, angular_damping_torque
```

### Spring Force (Elastic Deformation)

```
F = -k * Δx

Where:
k = Spring constant
Δx = Displacement from rest position
```

#### Implementation

```python
class SpringForce:
    def __init__(self, stiffness=1000.0, damping=10.0):
        self.k = stiffness
        self.c = damping

    def calculate_force(self, displacement, velocity):
        """Hooke's law with damping"""
        spring_force = -self.k * displacement
        damping_force = -self.c * velocity
        return spring_force + damping_force
```

## Collision Detection and Response

### Collision Detection Methods

#### Bounding Volume Hierarchy (BVH)
```
            ┌─────────────┐
            │ Root AABB   │
            └──────┬──────┘
                   │
        ┌──────────┼──────────┐
        │          │          │
        ▼          ▼          ▼
    ┌─────┐  ┌─────┐  ┌─────┐
    │ Box1│  │ Box2│  │ Box3│
    └──┬──┘  └──┬──┘  └──┬──┘
       │        │        │
       ▼        ▼        ▼
    Link1   Link2    Link3
```

#### Distance and Penetration Detection

```python
def check_collision(body1, body2):
    """Simple sphere collision detection"""
    center1 = body1.position
    center2 = body2.position
    radius1 = body1.collision_radius
    radius2 = body2.collision_radius

    distance = np.linalg.norm(center2 - center1)
    min_distance = radius1 + radius2

    if distance < min_distance:
        # Collision detected
        penetration_depth = min_distance - distance
        contact_normal = (center2 - center1) / distance

        return True, contact_normal, penetration_depth
    else:
        return False, None, None
```

### Collision Response

#### Impulse-Based Method

When two objects collide, apply an impulse to change their velocities:

```
Impulse = J = Δv * m

Where J is chosen to satisfy:
1. Coefficient of restitution (bounciness)
2. Non-penetration constraint
3. Friction if present
```

#### Implementation

```python
class CollisionResolver:
    def __init__(self, restitution=0.5):
        self.e = restitution  # Coefficient of restitution

    def resolve_collision(self, body1, body2, contact_normal, penetration):
        """Resolve collision between two bodies"""

        # Relative velocity at contact point
        relative_velocity = body2.velocity - body1.velocity
        velocity_along_normal = np.dot(relative_velocity, contact_normal)

        # Do not resolve if velocities are separating
        if velocity_along_normal > 0:
            return

        # Calculate impulse magnitude
        inv_mass1 = 1.0 / body1.mass
        inv_mass2 = 1.0 / body2.mass

        impulse_magnitude = -(1 + self.e) * velocity_along_normal
        impulse_magnitude /= (inv_mass1 + inv_mass2)

        # Apply impulse
        impulse = impulse_magnitude * contact_normal
        body1.velocity -= impulse * inv_mass1
        body2.velocity += impulse * inv_mass2

        # Separate bodies if penetrating
        separation = (penetration + 0.01) / (inv_mass1 + inv_mass2)
        body1.position -= separation * inv_mass1 * contact_normal
        body2.position += separation * inv_mass2 * contact_normal
```

## Constraints and Joints

### Joint Types in Simulation

#### Revolute Joint (Hinge)

```python
class RevoluteJoint:
    def __init__(self, body1, body2, anchor, axis):
        self.body1 = body1
        self.body2 = body2
        self.anchor = anchor  # Joint center
        self.axis = normalize(axis)  # Rotation axis

    def apply_constraint(self):
        """Keep bodies at fixed distance, allow rotation around axis"""
        # Calculate relative position
        r1 = self.anchor - self.body1.position
        r2 = self.anchor - self.body2.position

        # Constraint: distance remains constant
        relative_pos = self.body2.position + r2 - (self.body1.position + r1)

        if np.linalg.norm(relative_pos) > 0.01:
            # Apply constraint force
            normal = normalize(relative_pos)
            # ... calculate impulse to maintain distance constraint
```

#### Prismatic Joint (Slider)

```python
class PrismaticJoint:
    def __init__(self, body1, body2, axis):
        self.body1 = body1
        self.body2 = body2
        self.axis = normalize(axis)  # Allowed motion direction

    def apply_constraint(self):
        """Allow motion along axis, restrict perpendicular motion"""
        # ... implement slider constraint
```

## Complete Physics Loop

```python
class PhysicsEngine:
    def __init__(self, gravity=9.81, dt=0.001):
        self.gravity = gravity
        self.dt = dt  # Time step
        self.bodies = []
        self.joints = []
        self.collision_pairs = []

    def step(self):
        """Execute one physics simulation step"""

        # 1. Apply forces
        for body in self.bodies:
            body.apply_gravity(self.gravity)
            body.apply_forces()

        # 2. Detect collisions
        self.collision_pairs = self.detect_collisions()

        # 3. Resolve collisions
        for body1, body2, contact_data in self.collision_pairs:
            self.resolve_collision(body1, body2, contact_data)

        # 4. Apply constraints (joints, etc)
        for joint in self.joints:
            joint.apply_constraint()

        # 5. Integrate motion
        for body in self.bodies:
            body.integrate(self.dt)

    def detect_collisions(self):
        """Broad phase and narrow phase collision detection"""
        collisions = []

        for i, body1 in enumerate(self.bodies):
            for body2 in self.bodies[i+1:]:
                # Broad phase: check bounding volumes
                if self.bounding_volumes_intersect(body1, body2):
                    # Narrow phase: precise collision check
                    contact = self.precise_collision_check(body1, body2)
                    if contact:
                        collisions.append((body1, body2, contact))

        return collisions

    def resolve_collision(self, body1, body2, contact):
        """Handle collision response"""
        # Apply impulses, separate bodies, handle friction
        pass
```

## Practical Simulation Parameters

### Time Step Selection

```
Stability criterion: dt < 0.1 / sqrt(k/m)

Where:
k = spring constant (or stiffness)
m = mass

General Guidelines:
- For humanoid robots: dt = 0.001 to 0.002 seconds (500-1000 Hz)
- For mobile robots: dt = 0.01 to 0.02 seconds (50-100 Hz)
- For visualization: dt = 0.016 seconds (60 FPS)
```

### Material Properties

```python
# Typical material parameters

materials = {
    'rubber': {
        'friction': 0.7,
        'restitution': 0.6,
        'density': 1200,  # kg/m³
    },
    'metal': {
        'friction': 0.4,
        'restitution': 0.3,
        'density': 7850,
    },
    'wood': {
        'friction': 0.5,
        'restitution': 0.4,
        'density': 600,
    },
    'concrete': {
        'friction': 0.6,
        'restitution': 0.05,
        'density': 2400,
    },
}
```

## Verification and Validation

### Energy Conservation Check

```python
def check_energy_conservation(bodies, dt):
    """Verify energy is conserved (in absence of damping)"""
    total_energy = 0

    for body in bodies:
        # Kinetic energy
        ke = 0.5 * body.mass * np.dot(body.velocity, body.velocity)
        # Rotational kinetic energy
        rke = 0.5 * np.dot(body.angular_velocity,
                            np.dot(body.inertia, body.angular_velocity))
        # Potential energy
        pe = body.mass * 9.81 * body.position[2]

        total_energy += ke + rke + pe

    return total_energy
```

### Center of Mass Validation

```python
def validate_center_of_mass(bodies):
    """Verify center of mass moves correctly under external forces"""
    total_mass = sum(body.mass for body in bodies)
    total_force = sum(body.external_force for body in bodies)

    # Expected acceleration of center of mass
    expected_accel = total_force / total_mass

    # Actual acceleration (from simulation)
    actual_positions = [body.position for body in bodies]
    # ... calculate actual COM acceleration

    # Compare and report discrepancy
```

## Best Practices

1. **Choose Appropriate Time Step**: Balance accuracy vs. computation speed
2. **Validate Physics Parameters**: Compare with real-world measurements
3. **Test Edge Cases**: Verify behavior with extreme values
4. **Monitor Stability**: Watch for simulation divergence
5. **Profile Performance**: Identify bottlenecks
6. **Document Parameters**: Record all simulation settings
7. **Compare with Reality**: Periodically validate against real systems

## Summary

Physics simulation provides the foundation for realistic robot behavior in digital twins. By understanding force models, integration methods, collision handling, and constraints, you can create accurate, stable simulations. The key is choosing appropriate parameters and validation methods for your specific application.

## References

- Physics Engine Development: https://en.wikipedia.org/wiki/Physics_engine
- Rigid Body Dynamics: https://www.youtube.com/watch?v=iFHM8_9a6_Q
- GDC Physics Tutorials: https://www.gdcvault.com/
