use crate::drone::get_rotation_matrix;
use kiss3d::nalgebra::vector;
use rapier3d::dynamics::RigidBody;

pub fn get_angular_velocities(
    rb: &mut RigidBody,
) -> nalgebra::Matrix<f32, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f32, 3, 1>>
{
    let (roll, pitch, yaw) = rb.rotation().euler_angles();
    let world_angvel = rb.angvel().clone();
    let frame_angvel = get_rotation_matrix(vector![roll, pitch, yaw]).transpose() * world_angvel;
    frame_angvel
}
