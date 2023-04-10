use nalgebra::matrix;
use rapier3d::na::{Matrix3, Vector3, Vector4};
use rapier3d::prelude::*;

pub fn thrust(inputs: Vector4<Real>, k: Real) -> Real {
    k * inputs.sum()
}

pub fn torque(inputs: Vector4<Real>, L: Real, b: Real, k: Real) -> Vector3<Real> {
    let tau = vector![
        L * k * (inputs[0] - inputs[2]),
        L * k * (inputs[1] - inputs[3]),
        b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
    ];
    tau
}

pub fn rotation(angles: Vector3<Real>) -> Matrix3<Real> {
    let phi = angles[0];
    let theta = angles[1];
    let psi = angles[2];
    let R1 = matrix![1.0, 0.0, 0.0; 0.0, phi.cos(), phi.sin(); 0.0, -phi.sin(), phi.cos()];
    let R2 = matrix![theta.cos(), 0.0, -theta.sin(); 0.0, 1.0, 0.0; theta.sin(), 0.0, theta.cos()];
    let R3 = matrix![psi.cos(), psi.sin(), 0.0; -psi.sin(), psi.cos(), 0.0; 0.0, 0.0, 1.0];
    let R = R3.transpose() * R2.transpose() * R1.transpose();
    R
}

pub fn acceleration(
    inputs: Vector4<Real>,
    angles: Vector3<Real>,
    xdot: Vector3<Real>,
    m: Real,
    k: Real,
    kd: Real,
) -> Vector3<Real> {
    let quat = rapier3d::na::UnitQuaternion::from_euler_angles(angles.x, angles.y, angles.z);
    let normal = quat.transform_vector(&Vector3::new(0.0, 1.0, 0.0));
    let T = normal * thrust(inputs, k);
    //let Fd = -kd * xdot;
    let a = 1.0 / m * T;
    a
}
