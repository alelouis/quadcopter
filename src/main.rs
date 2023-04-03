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
    fn step(&mut self, window: &mut Window) {

        //println!("Ball position: {}, {}, {}",  ball_body.translation().x, ball_body.translation().y, ball_body.translation().z);
        //println!("Ball orientation: {},", ball_body.angvel());

        //translation += Vector3::new(0.0, 0.0, 0.01);
        //c.prepend_to_local_rotation(&rot);
        //c.set_local_translation(Translation {vector: translation});
    }
}

fn thrust(inputs: Vector4<Real>, k: Real) -> Real {
    k * inputs.sum()
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

fn acceleration(
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

fn pid(k: Real, L: Real, phi_dot: Real, theta_dot: Real, psi_dot: Real, b: Real) -> Vector4<Real> {
    let m = 0.5;
    let g = 9.81;
    let kp = 0.5;
    let e_phi = kp * (phi_dot);
    let e_theta = kp * (theta_dot);
    let e_psi = kp * (psi_dot - 1.0);
    let I: Matrix3<Real> = Matrix3::identity();

    let mut target_thrust = 100.0;
    target_thrust /= 4.0;

    let input_1 =
        target_thrust - (2.0 * b * e_phi * I.m11 + e_psi * I.m33 * k * L) / (4.0 * b * k * L);
    let input_2 = target_thrust + (e_psi * I.m33) / (4.0 * b) - (e_theta * I.m22) / (2.0 * k * L);
    let input_3 =
        target_thrust - (-2.0 * b * e_phi * I.m11 + e_psi * I.m33 * k * L) / (4.0 * b * k * L);
    let input_4 = target_thrust + (e_psi * I.m33) / (4.0 * b) + (e_theta * I.m22) / (2.0 * k * L);

    return Vector4::new(input_1, input_2, input_3, input_4);
}

fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    /* Create the ground. */
    // let collider = ColliderBuilder::cuboid(100.0, 0.1, 100.0).build();
    // collider_set.insert(collider);

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

        if up == true {
            thrust_vec += vector![1.0, 1.0, 1.0, 1.0] * 0.05;
        }

        if down == true {
            thrust_vec -= vector![1.0, 1.0, 1.0, 1.0] * 0.05;
        }

        inputs = thrust_vec;

        if left == true {
            let total_thrust = thrust_vec.sum();
            inputs += vector![
                total_thrust / 50.0,
                total_thrust / 50.0,
                -total_thrust / 50.0,
                -total_thrust / 50.0
            ];
        }

        if right == true {
            let total_thrust = thrust_vec.sum();
            inputs += vector![
                -total_thrust / 50.0,
                -total_thrust / 50.0,
                total_thrust / 50.0,
                total_thrust / 50.0
            ];
        }

        for mut event in window.events().iter() {
            match event.value {
                WindowEvent::Key(button, Action::Press, _) => {
                    match button {
                        Key::Return => {
                            rb.set_position(
                                Isometry::new(vector![0.0, 2.0, 0.0], vector![0.0, 0.0, 0.0]),
                                true,
                            );
                            inputs = vector![0.0, 0.0, 0.0, 0.0];
                            rb.set_angvel(vector![0.0, 0.0, 0.0], false);
                            rb.set_linvel(vector![0.0, 0.0, 0.0], false);
                        }
                        Key::Up => {
                            up = true;
                        }
                        Key::Down => {
                            down = true;
                        }
                        Key::Left => {
                            left = true;
                        }
                        Key::Right => {
                            right = true;
                        }
                        _ => {}
                    }
                    event.inhibited = true // override the default keyboard handler
                }
                WindowEvent::Key(button, Action::Release, _) => {
                    match button {
                        Key::Up => {
                            up = false;
                        }
                        Key::Down => {
                            down = false;
                        }
                        Key::Left => {
                            left = false;
                        }
                        Key::Right => {
                            right = false;
                        }
                        _ => {}
                    }
                    event.inhibited = true // override the default keyboard handler
                }
                _ => {}
            }
        }

        let (theta, phi, psi) = rb.rotation().euler_angles();
        let angular_velocities = rb.angvel().clone();
        let linvel = rb.linvel().xyz();
        let b = 1.0;
        let inputs_pid = pid(
            k,
            L,
            angular_velocities.x,
            angular_velocities.y,
            angular_velocities.z,
            b,
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
        //x.set_local_translation(Translation { vector: position });
        //z.set_local_translation(Translation { vector: position });
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

        let i0_intensity = inputs[0] / inputs.max();
        let i1_intensity = inputs[1] / inputs.max();
        let i2_intensity = inputs[2] / inputs.max();
        let i3_intensity = inputs[3] / inputs.max();
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
