mod controller;
mod drone;
mod pid;

use controller::get_commands;
use drone::{acceleration, rotation, thrust, torque};
use pid::pid;

use nalgebra;
use rapier3d::na::{matrix, Matrix1x3, Matrix3, Matrix3x1, Vector3, Vector4};
use rapier3d::prelude::*;

use kiss3d::camera::{ArcBall, Camera, FirstPerson};
use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::light::Light;
use kiss3d::nalgebra::{vector, Point2, Point3, Translation, Translation3, UnitQuaternion};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::{State, Window};
use nalgebra::SMatrix;

use std::sync::mpsc::{channel, TryRecvError};
use std::thread;

struct AppState {
    c: SceneNode,
    x: SceneNode,
    y: SceneNode,
    z: SceneNode,
    physics_pipeline: PhysicsPipeline,
    gravity: Vector<Real>,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
    ball_body_handle: RigidBodyHandle,
    L: Real,
    dt: Real,
    up: bool,
    down: bool,
    right: bool,
    left: bool,
    inputs: Vector4<Real>,
}

impl State for AppState {
    fn step(&mut self, window: &mut Window) {}
}

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    collider_set.insert(collider);

    /* Create the bounding ball. */
    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(vector![0.0, 2.0, 0.0])
        .build();
    let collider = ColliderBuilder::cuboid(1.0, 0.2, 1.0).mass(0.5).build();
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
    let L = 1.0;
    let dt = 1.;

    let mut window = Window::new("QuadCopter");
    let mut c = window.add_cube(1.0, 0.1, 1.0);
    c.set_color(0.0, 1.0, 0.0);
    let mut x = window.add_cube(10.0, 0.01, 0.01);
    let mut y = window.add_cube(0.01, 10.0, 0.01);
    let mut z = window.add_cube(0.01, 0.01, 10.0);

    window.set_light(Light::StickToCamera);
    let mut up = false;
    let mut down = false;
    let mut right = false;
    let mut left = false;
    let mut inputs = vector![0.0, 0.0, 0.0, 0.0];
    let mut thrust_vec = vector![0.0, 0.0, 0.0, 0.0];
    let eye = Point3::new(5.0f32, 5.0, 5.0);
    let mut arc_ball = FirstPerson::new(eye, Point3::origin());
    arc_ball.unbind_movement_keys();

    let a_ = Point3::new(-0.1, -0.1, 0.0);
    let b_ = Point3::new(0.0, 0.1, 0.0);
    let c_ = Point3::new(0.1, -0.1, 0.0);
    window.set_line_width(2.0);

    let mut i0 = window.add_circle(25.0);
    let mut i1 = window.add_circle(25.0);
    let mut i2 = window.add_circle(25.0);
    let mut i3 = window.add_circle(25.0);

    i0.append_translation(&kiss3d::nalgebra::Translation2::new(200.0, 0.0));
    i1.append_translation(&kiss3d::nalgebra::Translation2::new(250.0, 0.0));
    i2.append_translation(&kiss3d::nalgebra::Translation2::new(200.0, 50.0));
    i3.append_translation(&kiss3d::nalgebra::Translation2::new(250.0, 50.0));

    let font = kiss3d::text::Font::default();

    let (tx, rx) = channel();
    thread::spawn(move || get_commands(tx));
    let mut last_throttle: f32 = 0.0;
    let mut last_pitch: f32 = 0.0;
    let mut last_roll: f32 = 0.0;
    let mut last_yaw: f32 = 0.0;

    while !window.should_close() {
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
        let k = 0.1;
        let rb = rigid_body_set.get_mut(ball_body_handle).unwrap();
        let mut inputs = vector![0.0, 0.0, 0.0, 0.0];

        let (theta, phi, psi) = rb.rotation().euler_angles();
        let angular_velocities =
            rotation(vector![theta, phi, psi]).transpose() * rb.angvel().clone();
        let linvel = rb.linvel().xyz();
        let b = 1.0;

        let event = rx.try_recv();

        match event {
            Ok((throttle, pitch, roll, yaw)) => {
                last_throttle = throttle.unwrap_or(last_throttle);
                last_pitch = pitch.unwrap_or(last_pitch);
                last_roll = roll.unwrap_or(last_roll);
                last_yaw = yaw.unwrap_or(last_yaw);
            }
            _ => {}
        }

        let inputs_pid = pid(
            k,
            L,
            angular_velocities.x,
            angular_velocities.y,
            angular_velocities.z,
            b,
            last_throttle,
            last_pitch,
            last_roll,
            last_yaw,
        );
        inputs += inputs_pid;
        let acc = acceleration(inputs, vector![theta, phi, psi], linvel, 1.0, k, 0.00);
        let frame_torque = torque(inputs, L, 1.00, k);
        let world_torque = rotation(vector![theta, phi, psi]) * frame_torque;

        rb.reset_forces(true);
        rb.reset_torques(true);
        rb.add_force(dt * acc, true);
        rb.add_torque(dt * world_torque, true);

        let ball_body = &rigid_body_set[ball_body_handle];

        let position = kiss3d::nalgebra::Vector3::new(
            ball_body.translation().x,
            ball_body.translation().y,
            ball_body.translation().z,
        );
        c.set_local_translation(Translation { vector: position });

        let rotation = kiss3d::nalgebra::Vector3::new(1.0, 0.0, 0.0).normalize();
        let rotation_unit = kiss3d::nalgebra::Unit::new_normalize(rotation);
        let (roll, pitch, yaw) = ball_body.rotation().euler_angles();
        c.set_local_rotation(UnitQuaternion::from_euler_angles(roll, pitch, yaw));
        y.set_local_translation(Translation { vector: position });
        y.set_local_rotation(UnitQuaternion::from_euler_angles(theta, phi, psi));

        arc_ball.look_at(
            Point3::from(position + kiss3d::nalgebra::Vector3::new(5.0, 5.0, 5.0)),
            Point3::from(position),
        );
        //
        window.render_with_camera(&mut arc_ball);
        let vertical_velocity = ball_body.linvel().y;
        let horizontal_velocity =
            (ball_body.linvel().x.powi(2) + ball_body.linvel().z.powi(2)).sqrt();
        let altitude = ball_body.position().translation.y;
        let X = ball_body.position().translation.x;
        let Z = ball_body.position().translation.z;
        let rolld = roll.to_degrees();
        let pitchd = pitch.to_degrees();
        let yawd = yaw.to_degrees();
        let thrust = inputs.sum();

        let text = &*format!(
            "\
        Vertical velocity = {vertical_velocity:.2}\n\
        Horizontal velocity = {horizontal_velocity:.2}\n\
        Thrust = {thrust:.2}\n\
        Alt. = {altitude:.2}\n\
        X = {X:.2}\n\
        Z = {Z:.2}\n\
        Roll = {rolld:.2}\n\
        Pitch = {pitchd:.2}\n\
        Yaw = {yawd:.2}\n\
        Theta Ang Vel = {angular_velocities:.2}"
        );

        let i0_intensity = inputs[0] / inputs.sum();
        let i1_intensity = inputs[1] / inputs.sum();
        let i2_intensity = inputs[2] / inputs.sum();
        let i3_intensity = inputs[3] / inputs.sum();
        i0.set_color(0.0, i0_intensity, 0.0);
        i1.set_color(0.0, i1_intensity, 0.0);
        i2.set_color(0.0, i2_intensity, 0.0);
        i3.set_color(0.0, i3_intensity, 0.0);

        window.draw_text(
            text,
            &Point2::new(100.0, 100.0),
            80.0,
            &font,
            &Point3::new(1.0, 1.0, 1.0),
        );
    }
}
