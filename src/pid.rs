use crate::controller::Command;
use crate::simulation::Constants;
use rapier3d::na::{Matrix3, Vector4};
use rapier3d::prelude::*;
use std::cmp::max;

fn compute_expof(rc_command: f32) -> f32 {
    // https://github.com/ctzsnooze/betaflight/blob/b61d641bfc743e3cc58da05b50c869c0e7f59c7c/src/main/fc/rc.c
    let mut expof = 0.5;
    let expof = rc_command.abs() * (rc_command.powf(5.0) * expof + rc_command * (1.0 - expof));
    let center_sensitivity: f32 = 0.5;
    let stick_movement: f32 = (0.6 * 10.0 - center_sensitivity).max(0.0);
    let angle_rate: f32 = rc_command * center_sensitivity + stick_movement * expof;
    angle_rate
}

pub fn pid(
    phi_dot: Real,
    theta_dot: Real,
    psi_dot: Real,
    command: Command,
    constants: Constants,
) -> Vector4<Real> {
    // Still an only P controller for now...
    // TODO: Integration and derivative components of error filtering
    // PID coefficients
    let kp = 0.4;
    // let ki = 0.5;
    // let kd = 0.5;

    // Computing error on each euler angle
    let e_phi = kp * (phi_dot + compute_expof(command.pitch));
    let e_theta = kp * (theta_dot + compute_expof(command.yaw));
    let e_psi = kp * (psi_dot + compute_expof(command.roll));

    // Simple moment of inertia
    let i: Matrix3<Real> = Matrix3::identity();

    // Setting minimum thrust to barely uplift
    let mut target_thrust = 3.0 + command.throttle.sqrt() * 4.0;
    target_thrust /= 4.0;

    let b = constants.b;
    let l = constants.l;
    let k = constants.k;

    // Deriving correction inputs to minimize error
    // input_i is the rotor angular velocity squared
    let input_1 =
        target_thrust - (2.0 * b * e_phi * i.m11 + e_psi * i.m33 * k * l) / (4.0 * b * k * l);
    let input_2 = target_thrust + (e_psi * i.m33) / (4.0 * b) - (e_theta * i.m22) / (2.0 * k * l);
    let input_3 =
        target_thrust - (-2.0 * b * e_phi * i.m11 + e_psi * i.m33 * k * l) / (4.0 * b * k * l);
    let input_4 = target_thrust + (e_psi * i.m33) / (4.0 * b) + (e_theta * i.m22) / (2.0 * k * l);

    return Vector4::new(input_1, input_2, input_3, input_4);
}
