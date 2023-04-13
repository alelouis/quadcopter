use crate::controller::Command;
use crate::simulation::Constants;
use rapier3d::na::{Matrix3, Vector4};
use rapier3d::prelude::*;

pub fn pid(
    phi_dot: Real,
    theta_dot: Real,
    psi_dot: Real,
    command: Command,
    constants: Constants,
) -> Vector4<Real> {
    //let m = 0.5;
    //let g = 9.81;
    let kp = 0.5;
    let e_phi = kp * (phi_dot + 2.0 * command.pitch);
    let e_theta = kp * (theta_dot + 2.0 * command.yaw);
    let e_psi = kp * (psi_dot + 2.0 * command.roll);
    let i: Matrix3<Real> = Matrix3::identity();

    let mut target_thrust = 80.0 + command.throttle * 80.0;
    target_thrust /= 4.0;

    let b = constants.b;
    let l = constants.l;
    let k = constants.k;

    let input_1 =
        target_thrust - (2.0 * b * e_phi * i.m11 + e_psi * i.m33 * k * l) / (4.0 * b * k * l);
    let input_2 = target_thrust + (e_psi * i.m33) / (4.0 * b) - (e_theta * i.m22) / (2.0 * k * l);
    let input_3 =
        target_thrust - (-2.0 * b * e_phi * i.m11 + e_psi * i.m33 * k * l) / (4.0 * b * k * l);
    let input_4 = target_thrust + (e_psi * i.m33) / (4.0 * b) + (e_theta * i.m22) / (2.0 * k * l);

    return Vector4::new(input_1, input_2, input_3, input_4);
}
