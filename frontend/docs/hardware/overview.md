---
sidebar_position: 1
title: "Hardware Requirements"
---

<ChapterActions />

# Hardware Requirements

## Building Your Physical AI Lab

This course requires significant computational resources. Here's everything you need to know about setting up your development environment.

## 🖥️ The "Digital Twin" Workstation (Required)

This is the most critical component. NVIDIA Isaac Sim requires RTX capabilities.

### GPU Requirements

| Component | Minimum | Recommended | Ideal |
|-----------|---------|-------------|-------|
| **GPU** | RTX 4070 Ti (12GB) | RTX 3090 (24GB) | RTX 4090 (24GB) |
| **VRAM** | 12GB | 24GB | 24GB |
| **Compute** | CUDA 11.8+ | CUDA 12+ | CUDA 12+ |

**Why?** You need high VRAM to load USD assets for robots and environments, plus run VLA models simultaneously.

### CPU Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **CPU** | Intel i7 13th Gen | AMD Ryzen 9 5900X |
| **Cores** | 8 cores | 12+ cores |
| **Speed** | 3.0 GHz | 3.5+ GHz |

**Why?** Physics calculations in Gazebo/Isaac are CPU-intensive.

### Memory & Storage

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| **RAM** | 32GB DDR4 | 64GB DDR5 |
| **Storage** | 256GB SSD | 1TB NVMe |
| **OS** | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

## 🤖 The "Physical AI" Edge Kit

For real-world deployment after simulation.

### The Brain

| Device | Price | Specs | Use Case |
|--------|-------|-------|----------|
| **Jetson Orin Nano** | ~$249 | 8GB, 40 TOPS | Development |
| **Jetson Orin NX** | ~$599 | 16GB, 100 TOPS | Production |
| **Jetson AGX Orin** | ~$1999 | 64GB, 275 TOPS | Enterprise |

### The Eyes (Vision)

| Sensor | Price | Features |
|--------|-------|----------|
| **Intel RealSense D435i** | ~$349 | RGB + Depth + IMU |
| **Intel RealSense D455** | ~$449 | Wider FOV, longer range |
| **ZED 2** | ~$449 | Stereo depth, AI features |

### Voice Interface

| Device | Price | Features |
|--------|-------|----------|
| **ReSpeaker USB Mic Array** | ~$69 | Far-field, 4-mic array |
| **USB Conference Speaker** | ~$50 | For audio output |

## 🦿 Robot Hardware Options

### Option A: The "Proxy" Approach (Budget-Friendly)

Use a quadruped or robotic arm as a proxy. Software principles transfer 90% effectively.

| Robot | Price | Pros | Cons |
|-------|-------|------|------|
| **Unitree Go2** | $1,800-$3,000 | Durable, ROS 2 support | Not biped |
| **Hiwonder TonyPi Pro** | ~$600 | Affordable humanoid | Limited compute |

### Option B: Real Humanoid (Premium)

| Robot | Price | Features |
|-------|-------|----------|
| **Unitree G1** | ~$16,000 | Walking, ROS 2 SDK |
| **Unitree H1** | ~$90,000 | Full humanoid |

## ☁️ Cloud Alternative

If local hardware isn't available:

### Cloud Workstations

| Provider | Instance | GPU | Cost/Hour |
|----------|----------|-----|-----------|
| **AWS** | g5.2xlarge | A10G (24GB) | ~$1.50 |
| **GCP** | a2-highgpu-1g | A100 (40GB) | ~$3.00 |
| **Azure** | NC4as_T4_v3 | T4 (16GB) | ~$0.50 |

### Estimated Cloud Costs

```
Usage: 10 hours/week × 12 weeks = 120 hours
Instance cost: ~$1.50/hour
Storage: ~$25/quarter
Total: ~$205 per quarter
```

## 📊 Complete Kit Summary

### Economy Student Kit (~$700)

```
┌─────────────────────────────────────────┐
│         Economy Student Kit             │
├─────────────────────────────────────────┤
│ • Jetson Orin Nano Super (8GB)  $249   │
│ • Intel RealSense D435i          $349   │
│ • ReSpeaker USB Mic Array        $69    │
│ • 128GB High-Endurance SD Card   $30    │
├─────────────────────────────────────────┤
│ TOTAL                            ~$700  │
└─────────────────────────────────────────┘
```

### Full Lab Setup (~$15,000+)

```
┌─────────────────────────────────────────┐
│           Full Lab Setup                │
├─────────────────────────────────────────┤
│ • RTX 4090 Workstation          $3,500  │
│ • Jetson AGX Orin               $1,999  │
│ • Intel RealSense D455          $449    │
│ • ReSpeaker Mic Array           $69     │
│ • Unitree Go2 Robot             $3,000  │
│ • Peripherals & Setup           $500    │
├─────────────────────────────────────────┤
│ TOTAL                          ~$9,500  │
└─────────────────────────────────────────┘
```

## 🔧 Software Stack

### Required Software

```bash
# Operating System
Ubuntu 22.04 LTS

# ROS 2
sudo apt install ros-humble-desktop

# NVIDIA Software
CUDA Toolkit 11.8+
cuDNN 8.6+
TensorRT 8.5+

# Python
Python 3.10+
PyTorch 2.0+
```

### Development Tools

```bash
# Essential tools
sudo apt install -y \
  git \
  cmake \
  build-essential \
  python3-pip \
  python3-venv

# ROS 2 tools
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rqt*
```

## 💡 Recommendations by Use Case

### For Students (Learning)
- Start with cloud instances
- Get Jetson Orin Nano for deployment
- Use simulation extensively

### For Researchers
- RTX 4080/4090 workstation
- Jetson Orin NX for edge
- Quadruped robot for testing

### For Production
- Multiple RTX workstations
- Jetson AGX Orin for deployment
- Full humanoid robot

## ⚠️ Important Notes

1. **GPU is Critical**: MacBooks and non-RTX machines will NOT work with Isaac Sim
2. **Linux Required**: ROS 2 is native to Linux; dual-boot or dedicated machines recommended
3. **Ventilation**: High-performance GPUs generate significant heat
4. **Power**: RTX 4090 requires 450W+ PSU capacity

---

*Need help choosing hardware? Ask the chatbot for personalized recommendations based on your budget and goals!*
