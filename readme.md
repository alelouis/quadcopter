# **quadcopter simulation**
![rustc](https://img.shields.io/badge/rustc-1.67.0-important)

**Implementation of quadcopter flight dynamics & control.**  
Experimentation using [rapier](https://rapier.rs/) physics engine, [kiss3D](https://docs.rs/kiss3d/0.35.0/kiss3d/) 3D renderer as well as [egui](https://docs.rs/egui/0.20.1/egui/) for metrics visualization.


## **What it does**
The main idea was to connect several things in order to create a toy drone 3D simulator :
- Reading throttle, roll, pitch and yaw inputs from a real RF radiocontroller (tested with Radiomaster Zorro in joystick usb mode).
- Simulate physics of a quadcopter (drone with 4 rotors) from individual control of each rotor angular velocity.
- Implement a P(ID soon) controller for angular velocity error correction.
- Use rapier physics engine in order to model the drone as a rigid body on which derived accelerations and torques are applied.
- Use kiss3D rendering engine to visualize a simple FPV scene used for real time piloting.
- Integrate a graphical interface (with egui) for real time plotting of key metrics from the simulation.
## **How to run**
Clone the repository.
```bash
gh clone alelouis/quadcopter
```
Then run main binary using cargo, preferably in release mode.
```bash
cargo run --release
```

You can also launch the second app for viewing radio commands in realtime.
```bash
cargo run --bin logs --release
```