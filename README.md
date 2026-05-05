#  Franka Panda Hand Tracking Teleop (Isaac Sim + ROS 2)

Control a Franka Panda robot arm in NVIDIA Isaac Sim using your hand via webcam. Built with MediaPipe for real-time hand tracking, ikpy for inverse kinematics, and ROS 2 Humble for communication.

---

## Demo Video

https://github.com/Teksupportt/franka_teleop/raw/main/franka_teleop.mp4

---

## How It Works

The system runs three parallel threads:

1. **Camera Thread** - Captures frames from a USB webcam at 640x480 @ 30 FPS
2. **MediaPipe Thread** - Detects hand landmarks in real time; maps wrist position to a 3D Cartesian target and finger spread to gripper openness
3. **IK Thread** - Solves inverse kinematics using `ikpy` against the Panda URDF and applies low-pass filtering + joint step limiting for smooth motion

Joint states (7 arm joints + 2 finger joints) are published to `/joint_command` at 20 Hz, which Isaac Sim subscribes to via the ROS 2 bridge.

---

## Repository Structure

```
franka_teleop/
├── teleop.py           # Main ROS 2 node
├── panda.urdf          # Franka Panda URDF (required for IK)
├── franka_teleop.mp4   # Demo video
└── README.md
```

---

## Dependencies

### System
| Dependency | Version | Notes |
|---|---|---|
| Ubuntu | 22.04 | Running inside WSL2 |
| ROS 2 | Humble | Full desktop install |
| Python | 3.10+ | Included with Ubuntu 22.04 |
| NVIDIA Isaac Sim | 5.1 | Windows host, ROS 2 bridge enabled |

### Python Packages
| Package | Install |
|---|---|
| `mediapipe` | `pip install mediapipe` |
| `ikpy` | `pip install ikpy` |
| `opencv-python` | `pip install opencv-python` |
| `numpy` | `pip install "numpy<2"` Must be NumPy 1.x |

> **NumPy compatibility:** MediaPipe requires NumPy < 2.0.


### ROS 2 Packages
```bash
sudo apt install ros-humble-sensor-msgs
```

### URDF File
The `panda.urdf` file is required for IK solving. You can obtain it from the [franka_description](https://github.com/frankaemika/franka_ros) ROS package:
```bash
sudo apt install ros-humble-franka-description
cp /opt/ros/humble/share/franka_description/robots/panda/panda.urdf ~/ros/Panda_teleop/
```
Or place your own `panda.urdf` in the project directory and update the path in `teleop.py` line 12.

---

## Setup

### 1. Clone the repository (WSL2)
```bash
git clone https://github.com/Teksupportt/franka_teleop.git
cd franka_teleop
```

### 2. Install Python dependencies
```bash
pip install "numpy<2" mediapipe ikpy opencv-python --break-system-packages
```

### 3. Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```

### 4. Update URDF path (if needed)
Edit line 12 of `teleop.py` to point to your `panda.urdf`:
```python
chain = Chain.from_urdf_file("/path/to/your/panda.urdf", ...)
```

---

## Running with Isaac Sim (WSL2)

### Step 1 - Attach USB webcam to WSL2

On **Windows**, open PowerShell as Administrator and run:
```powershell
usbipd attach --wsl --busid 2-3
```
> Replace `2-3` with your camera's bus ID. Find it with `usbipd list`.

Verify the camera is visible in WSL2:
```bash
ls /dev/video*
```

### Step 2 - Launch Isaac Sim on Windows

1. Open **NVIDIA Isaac Sim 5.1**
2. Load your Franka Panda scene
3. Enable the **ROS 2 Bridge** extension: `Window → Extensions → search "ROS2 Bridge" → Enable`
4. Make sure the `/joint_command` topic is subscribed to by the articulation controller

### Step 3 - Run the teleop node (WSL2)

```bash
source /opt/ros/humble/setup.bash
cd ~/ros/Panda_teleop
python3 teleop.py
```

A window will open showing the webcam feed with:
- 🟢 Green dot = hand detected
- 🔴 Red dot = no hand detected
- Cyan bar = gripper openness %
- Live joint angles and target XYZ overlay

### Step 4 - Control the robot

| Hand Action | Robot Action |
|---|---|
| Move hand left/right | Robot moves along Y axis |
| Move hand up/down | Robot moves along X axis |
| Open/close fingers | Opens/closes gripper |

---

## Tuning Parameters

In `teleop.py`, you can adjust these values in `__init__`:

| Parameter | Default | Description |
|---|---|---|
| `self.alpha` | `0.4` | Low-pass filter strength (lower = smoother but slower) |
| `self.max_joint_step` | `0.15` | Max joint change per step (lower = safer) |
| `self.gripper_opening` | `0.08` | Initial gripper opening (meters) |

---

## Troubleshooting

**Camera not found**
```bash
# Check available video devices
ls /dev/video*
# Re-attach USB device from Windows PowerShell (Admin)
usbipd attach --wsl --busid 2-3
```

**MediaPipe / NumPy error**
```bash
pip install "numpy<2" --break-system-packages
pip install --upgrade mediapipe --break-system-packages
```

**IK solving fails**
- Ensure `panda.urdf` path in `teleop.py` is correct
- Hand position may be out of robot workspace - move hand closer to center of frame

**No joint movement in Isaac Sim**
- Confirm ROS 2 Bridge is active in Isaac Sim
- Check topic name matches: `ros2 topic list | grep joint_command`

---

## License

MIT License - free to use, modify, and distribute.

---

## Acknowledgements

- [MediaPipe](https://mediapipe.dev/) - Hand landmark detection
- [ikpy](https://github.com/Phylliade/ikpy) - Inverse kinematics
- [Franka Robotics](https://www.franka.de/) - Panda URDF
- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) - Robot simulation
