use crate::drone::*;
use nalgebra::{SMatrix, Vector4};

use rapier3d::prelude::*;

pub struct Simulation {
    // Ugly wrap of rapier3d physics engine components
    gravity: SMatrix<Real, 3, 1>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    drone_handle: RigidBodyHandle,
}

#[derive(Copy, Clone)]
pub struct Constants {
    // #physics
    pub(crate) g: Real,
    pub(crate) l: Real,
    pub(crate) k: Real,
    pub(crate) b: Real,
    pub(crate) m: Real,
    pub(crate) dt: Real,
}

impl Constants {
    pub fn new(g: Real, l: Real, k: Real, b: Real, m: Real, dt: Real) -> Self {
        Self { g, l, k, b, m, dt }
    }
}

impl Simulation {
    pub fn new(constants: Constants) -> Self {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();

        // Create the drone's rigid body and colliders
        let drone_rb = RigidBodyBuilder::dynamic()
            .translation(vector![0.0, 0.0, 2.0])
            .build();

        let drone_collider = ColliderBuilder::cuboid(1.0, 1.0, 0.2)
            .mass(constants.m)
            .build();
        let drone_handle = rigid_body_set.insert(drone_rb);
        collider_set.insert_with_parent(drone_collider, drone_handle, &mut rigid_body_set);

        // Let the body hit the floor
        let floor = ColliderBuilder::cuboid(100.0, 100., 0.1).build();
        collider_set.insert(floor);

        Self {
            gravity: vector![0.0, 0.0, -constants.g],
            rigid_body_set,
            collider_set,
            integration_parameters: Default::default(),
            physics_pipeline: Default::default(),
            island_manager: Default::default(),
            broad_phase: Default::default(),
            narrow_phase: Default::default(),
            impulse_joint_set: Default::default(),
            multibody_joint_set: Default::default(),
            ccd_solver: Default::default(),
            drone_handle,
        }
    }

    pub fn step(&mut self) {
        // rapier physics engine step
        let physics_hooks = ();
        let event_handler = ();
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &physics_hooks,
            &event_handler,
        )
    }

    pub fn get_drone_rb(&mut self) -> &mut RigidBody {
        return self.rigid_body_set.get_mut(self.drone_handle).unwrap();
    }
}

pub fn update_physics(inputs: Vector4<Real>, rb: &mut RigidBody, constants: Constants) {
    // Compute new acceleration and torque from PID correction inputs
    let (roll, pitch, yaw) = rb.rotation().euler_angles();

    // Get acceleration in world frame
    let acceleration =
        compute_acceleration(inputs, vector![roll, pitch, yaw], constants.m, constants.k);

    // Get torque in world frame
    let frame_torque = compute_torque(inputs, constants.l, constants.b, constants.k);
    let world_torque = get_rotation_matrix(vector![roll, pitch, yaw]) * frame_torque;

    // physics update
    rb.reset_forces(true);
    rb.reset_torques(true);
    rb.add_force(constants.dt * acceleration, true);
    rb.add_torque(constants.dt * world_torque, true);
}
