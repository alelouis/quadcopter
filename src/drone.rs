use crate::controller::Command;
use crate::simulation::Constants;
use nalgebra::matrix;
use rapier3d::na::{Matrix3, Vector3, Vector4};
use rapier3d::prelude::*;
use zerocopy::{AsBytes, FromBytes};

#[repr(C)]
#[derive(Copy, Clone, Default, AsBytes, FromBytes, Debug)]
pub struct QuadState {
    // World frame position
    pub position: [f32; 3],
    // Quad center frame rotation (roll, pitch, yaw)
    pub rotation: [f32; 3],
    // World frame velocity
    pub velocity: [f32; 3],
    // Quad centered frame angular velocities (roll dot, pitch dot, yaw dot)
    pub angular_velocity: [f32; 3],
    // World frame acceleration
    pub acceleration: [f32; 3],
    // World frame angular acceleration
    pub torque: [f32; 3],
    pub input_throttle: f32,
    pub input_roll: f32,
    pub input_pitch: f32,
    pub input_yaw: f32,
    pub rotors_velocities: [f32; 4],
}

impl QuadState {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn update(
        self: &mut Self,
        constants: Constants,
        inputs: Vector4<Real>,
        rb: &mut RigidBody,
        cmd: Command,
    ) {
        self.position = vector_to_f64(rb.position().translation.vector);
        self.rotation = vector_to_f64({
            let (roll, pitch, yaw) = rb.rotation().euler_angles();
            Vector3::new(roll, pitch, yaw) // why is this wrong ?
        });
        self.velocity = vector_to_f64(rb.linvel().xyz());
        self.angular_velocity = vector_to_f64(
            get_rotation_matrix(rb.position().translation.vector).transpose() * rb.angvel(),
        );
        self.acceleration = vector_to_f64(compute_acceleration(
            inputs,
            rb.position().translation.vector,
            constants.m,
            constants.k,
        ));
        self.torque = vector_to_f64(compute_torque(
            inputs,
            constants.l,
            constants.b,
            constants.k,
        ));
        self.input_throttle = cmd.throttle;
        self.input_roll = cmd.roll;
        self.input_pitch = cmd.pitch;
        self.input_yaw = cmd.yaw;
        self.rotors_velocities = [inputs.x, inputs.y, inputs.z, inputs.w];
    }
}

fn vector_to_f64(vec: Vector3<Real>) -> [f32; 3] {
    [vec.x as f32, vec.y as f32, vec.z as f32]
}

// Thrust is sum of velocities squares
pub fn thrust(inputs: Vector4<Real>, k: Real) -> Real {
    k * inputs.sum()
}

// Torque computation from input squared velocities
pub fn compute_torque(inputs: Vector4<Real>, l: Real, b: Real, k: Real) -> Vector3<Real> {
    let tau = vector![
        l * k * (inputs[0] - inputs[2]),
        l * k * (inputs[1] - inputs[3]),
        b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
    ];
    tau
}

// Rotation matrix from body frame to inertial frame.
pub fn get_rotation_matrix(angles: Vector3<Real>) -> Matrix3<Real> {
    let roll = angles[0]; // roll
    let pitch = angles[1]; // pitch
    let yaw = angles[2]; // yaw
    let r1 = matrix![1.0, 0.0, 0.0; 0.0, roll.cos(), roll.sin(); 0.0, -roll.sin(), roll.cos()];
    let r2 = matrix![pitch.cos(), 0.0, -pitch.sin(); 0.0, 1.0, 0.0; pitch.sin(), 0.0, pitch.cos()];
    let r3 = matrix![yaw.cos(), yaw.sin(), 0.0; -yaw.sin(), yaw.cos(), 0.0; 0.0, 0.0, 1.0];
    let r = r3.transpose() * r2.transpose() * r1.transpose();
    r
}

// Compute acceleration in normal quad direction.
pub fn compute_acceleration(
    inputs: Vector4<Real>,
    angles: Vector3<Real>,
    m: Real,
    k: Real,
) -> Vector3<Real> {
    let normal =
        get_rotation_matrix(vector![angles.x, angles.y, angles.z]) * Vector3::new(0.0, 0.0, 1.0);
    let t = normal * thrust(inputs, k);
    let a = 1.0 / m * t;
    a
}
