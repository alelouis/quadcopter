use nalgebra::matrix;
use rapier3d::na::{Matrix3, Vector3, Vector4};
use rapier3d::prelude::*;

pub fn thrust(inputs: Vector4<Real>, k: Real) -> Real {
    k * inputs.sum()
}

pub fn compute_torque(inputs: Vector4<Real>, l: Real, b: Real, k: Real) -> Vector3<Real> {
    let tau = vector![
        l * k * (inputs[0] - inputs[2]),
        l * k * (inputs[1] - inputs[3]),
        b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
    ];
    tau
}

pub fn get_rotation_matrix(angles: Vector3<Real>) -> Matrix3<Real> {
    let phi = angles[0];
    let theta = angles[1];
    let psi = angles[2];
    let r1 = matrix![1.0, 0.0, 0.0; 0.0, phi.cos(), phi.sin(); 0.0, -phi.sin(), phi.cos()];
    let r2 = matrix![theta.cos(), 0.0, -theta.sin(); 0.0, 1.0, 0.0; theta.sin(), 0.0, theta.cos()];
    let r3 = matrix![psi.cos(), psi.sin(), 0.0; -psi.sin(), psi.cos(), 0.0; 0.0, 0.0, 1.0];
    let r = r3.transpose() * r2.transpose() * r1.transpose();
    r
}

pub fn compute_acceleration(
    inputs: Vector4<Real>,
    angles: Vector3<Real>,
    m: Real,
    k: Real,
) -> Vector3<Real> {
    let quat = rapier3d::na::UnitQuaternion::from_euler_angles(angles.x, angles.y, angles.z);
    let normal = quat.transform_vector(&Vector3::new(0.0, 0.0, 1.0));
    let t = normal * thrust(inputs, k);
    //let Fd = -kd * xdot;
    let a = 1.0 / m * t;
    a
}
