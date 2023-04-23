mod controller;
mod drone;
mod pid;
mod sensors;
mod simulation;
mod view;

use controller::get_commands;
use pid::pid;

use crate::controller::Command;
use std::sync::mpsc::channel;
use std::thread;
use zerocopy::AsBytes;
use zmq::{Context, Message};

fn main() {
    // Setup UI
    let mut graphical = view::setup_ui();

    // Initialize simulation
    let constants = simulation::Constants::new(9.81, 0.05, 1.0, 1.0, 0.650, 0.1, 0.2);
    let mut sim = simulation::Simulation::new(constants);

    // Start controller command polling
    let (tx, rx) = channel();
    thread::spawn(move || get_commands(tx));
    let mut command = Command::new();
    let mut quad_state = drone::QuadState::new();

    // Client socket
    let context = Context::new();
    let requester = context.socket(zmq::REQ).unwrap();
    assert!(requester.connect("tcp://localhost:5555").is_ok());

    // Simulation loop

    while !graphical.window.should_close() {
        // Rapier update
        sim.step();

        // Get command events from controller
        controller::update_commands(&mut command, &rx);

        // Sense gyroscopic information
        let rb = sim.get_drone_rb();
        let frame_angvel = sensors::get_angular_velocities(rb);

        // PID error loop
        let inputs = pid(
            frame_angvel.x,
            frame_angvel.y,
            frame_angvel.z,
            command,
            constants,
        );

        // Compute acceleration and torque from inputs and apply to rigidbody
        simulation::update_physics(inputs, rb, constants);

        // Update drone state
        quad_state.update(constants, inputs, rb, command);

        // Update UI
        view::update_ui(&mut graphical, rb, inputs);

        // Send command to socket
        requester
            .send(quad_state.as_bytes(), zmq::DONTWAIT)
            .unwrap_or(());
        let mut msg = Message::new();

        requester.recv(&mut msg, zmq::DONTWAIT).unwrap_or(());
    }
}
