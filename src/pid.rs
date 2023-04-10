use rapier3d::na::{Matrix3, Vector4};
use rapier3d::prelude::*;

pub fn pid(
    k: Real,
    L: Real,
    phi_dot: Real,
    theta_dot: Real,
    psi_dot: Real,
    b: Real,
    throttle_cmd: f32,
    pitch_cmd: f32,
    roll_cmd: f32,
    yaw_cmd: f32,
) -> Vector4<Real> {
    let m = 0.5;
    let g = 9.81;
    let kp = 0.5;
    let e_phi = kp * (phi_dot + 2.0 * pitch_cmd);
    let e_theta = kp * (theta_dot + 2.0 * yaw_cmd);
    let e_psi = kp * (psi_dot + 2.0 * roll_cmd);
    let I: Matrix3<Real> = Matrix3::identity();

    let mut target_thrust = 40.0 + throttle_cmd * 50.0;
    target_thrust /= 4.0;

    let input_1 =
        target_thrust - (2.0 * b * e_phi * I.m11 + e_psi * I.m33 * k * L) / (4.0 * b * k * L);
    let input_2 = target_thrust + (e_psi * I.m33) / (4.0 * b) - (e_theta * I.m22) / (2.0 * k * L);
    let input_3 =
        target_thrust - (-2.0 * b * e_phi * I.m11 + e_psi * I.m33 * k * L) / (4.0 * b * k * L);
    let input_4 = target_thrust + (e_psi * I.m33) / (4.0 * b) + (e_theta * I.m22) / (2.0 * k * L);

    return Vector4::new(input_1, input_2, input_3, input_4);
}
