# ME5250 Project 1: Underactuated Finger Mechanism

## Overview
This repository contains the MATLAB implementation and analysis of an underactuated robotic finger mechanism for ME5250 Fall 2025. The project demonstrates analytical forward kinematics solutions for a complex closed-loop mechanism with phase-switching behavior.

## Project Description
The mechanism is a single-actuator driven finger with adaptive grasping capabilities through geometric constraints. It features:
- **9 links** connected through 10 revolute joints and 1 prismatic joint
- **2 DOF** with single actuator control (underactuated)
- **Dual-phase operation**: Rigid body motion transitioning to V-linkage articulation
- **Analytical solutions** for forward kinematics (no numerical iteration required)

## Mechanism Features
### Phase 1: Rigid Link Assembly (θ ∈ [-130°, -50°])
- Four-bar linkage operating as rigid body
- 80° rotation producing initial finger curl
- Rotation matrix transformations for kinematics

### Phase 2: V-Linkage Mechanism (θ ∈ [-50°, 0°])
- Articulated V-linkage with coupled motion
- Constraint circle intersection method for solutions
- Smooth transition at switching point

## Repository Contents
### Core MATLAB Files
- `slider_crank.m` - Base slider-crank mechanism analysis
- `rigid_quad_linkage.m` - Rigid link assembly
- `v_linkage_mechanism.m` - V-linkage phase kinematics
- `finger_mechanism.m` - Complete integrated system with phase switching

## How to Run
1. Clone the repository:
```bash
   git clone https://github.com/[your-username]/ME5250-Underactuated-Finger.git
```

2. Open MATLAB and navigate to the project directory

3. Run the main combined mechanism:
```matlab
   combined_finger_mechanism.m
```

4. Use the interactive slider to explore the mechanism's range of motion

## Analysis Features
- Real-time visualization of mechanism motion
- End-effector trajectory tracking
- Phase transition visualization
- Solution branch selection for smooth motion
- Workspace analysis (~92mm × 65mm coverage)

## Technical Approach
### Forward Kinematics Methodology
1. **Slider-Crank Analysis**: Law of cosines for actuator-to-crank mapping
2. **Closed-Loop Decomposition**: Separated into manageable subsystems
3. **Constraint Circle Method**: Geometric intersection for V-linkage solutions
4. **Analytical Solutions**: All kinematics solved in closed-form

## Video Demonstration
[View mechanism animation and analysis](https://drive.google.com/file/d/1M96xGyKvcZoVZR8mid9pmtTU457SAQWX/view?usp=sharing)

## Requirements
- MATLAB R2020a or later
- No additional toolboxes required

## Author
Ahmad Hassan  
ME5250 - Fall 2025  
Northeastern University

## Acknowledgments
Project completed for ME5250 under the guidance of Prof. Whitney.

## License
This project is for educational purposes as part of ME5250 coursework.
