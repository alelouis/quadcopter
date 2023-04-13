mod controller;
mod drone;
mod pid;
mod sensors;
mod simulation;
mod ui;

use controller::get_commands;
use pid::pid;

use crate::controller::Command;
use std::sync::mpsc::channel;
use std::thread;

fn main() {
    // Setup UI
    let mut graphical = ui::setup_ui();

    // Initialize simulation
    let constants = simulation::Constants::new(9.81, 1.0, 0.1, 1.0, 1.0, 1.0);
    let mut sim = simulation::Simulation::new(constants);

    // Start controller command polling
    let (tx, rx) = channel();
    thread::spawn(move || get_commands(tx));
    let mut command = Command::new();

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

        // Update UI
        ui::update_ui(&mut graphical, rb, inputs);
    }
}
