use rapier3d::na::{matrix, Matrix1x3, Matrix3, Matrix3x1, Vector3, Vector4};
use rapier3d::prelude::*;
use nalgebra;

fn thrust(inputs: Vector4<Real>, k: Real) -> Vector3<Real> {
    let T = vector![0.0, 0.0, k * inputs.sum()];
    T
}

fn torque(inputs: Vector4<Real>, L: Real, b: Real, k: Real) -> Vector3<Real> {
    let tau = vector![
        L * k * (inputs[0] - inputs[2]),
        L * k * (inputs[1] - inputs[3]),
        b * (inputs[0] - inputs[1] + inputs[2] - inputs[3])
    ];
    tau
}

fn rotation(angles: Vector3<Real>) -> Matrix3<Real> {
    let phi = angles[0];
    let theta = angles[1];
    let psi = angles[2];
    let R1 = matrix![1.0, 0.0, 0.0; 0.0, phi.cos(), phi.sin(); 0.0, -phi.sin(), phi.cos()];
    let R2 = matrix![theta.cos(), 0.0, -theta.sin(); 0.0, 1.0, 0.0; theta.sin(), 0.0, theta.cos()];
    let R3 = matrix![psi.cos(), psi.sin(), 0.0; -psi.sin(), psi.cos(), 0.0; 0.0, 0.0, 1.0];
    let R = R3.transpose() * R2.transpose() * R1.transpose();
    R
}

fn acceleration(inputs: Vector4<Real>, angles: Vector3<Real>, xdot: Vector3<Real>, m: Real, k: Real, kd: Real) -> Vector3<Real> {
    let R = rotation(angles);
    let T = R * thrust(inputs, k);
    let Fd = -kd * xdot;
    let a = 1.0/m * T + Fd;
    a
}

fn angular_acceleration(inputs: Vector4<Real>, omega: &Vector<Real>, I: Matrix3<Real>, L: Real, b: Real, k: Real) -> Vector3<Real> {
    let tau = torque(inputs, L, b, k);
    let vec = (I * omega).xyz();
    let cross = &omega.cross(&vec);
    let omegaddot = I.pseudo_inverse(0.0).unwrap() * (tau - cross);
    omegaddot
}

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    collider_set.insert(collider);

    /* Create the bounding ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 0.0, 0.0])
        .build();
    let collider = ColliderBuilder::ball(0.5).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

    /* Create other structures necessary for the simulation. */
    let g = 9.81;
    let gravity = vector![0.0, -g, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let physics_hooks = ();
    let event_handler = ();
    let L = 0.2;
    let dt = 1.;

    /* Run the game loop, stepping the simulation once per frame. */
    for _ in 0..200 {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            None,
            &physics_hooks,
            &event_handler,
        );

        let k = 1.0;
        let rb = rigid_body_set.get_mut(ball_body_handle).unwrap();
        let inputs = vector![1.0, 1.0, 1.0, 1.0] * 5.0
            ;
        let (theta, phi, psi) = rb.rotation().euler_angles();
        let linvel = rb.linvel().xyz();
        let acc = acceleration(inputs, vector![theta, phi, psi], linvel, 1.0, k, 0.05);
        let w = angular_acceleration(inputs, rb.angvel(), Matrix3::identity(), L, 0.01, k);

        rb.add_force(dt*acc, true);
        rb.add_torque(dt*w, true);

        let ball_body = &rigid_body_set[ball_body_handle];
        println!("Ball position: {}, {}, {}", ball_body.translation().x, ball_body.translation().y, ball_body.translation().z);
        //println!("Ball orientation: {},", ball_body.angvel());
    }
}
