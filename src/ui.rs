use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point2, Point3, Translation, UnitQuaternion};
use kiss3d::scene::SceneNode;
use kiss3d::text::Font;
use kiss3d::window::Window;
use nalgebra::SMatrix;
use rapier3d::prelude::RigidBody;
use rapier3d::prelude::*;
use std::rc::Rc;

pub struct Graphical {
    pub(crate) window: Window,
    pub(crate) drone_object: SceneNode,
    pub(crate) drone_normal_vec: SceneNode,
    pub(crate) arc_ball: FirstPerson,
    pub(crate) font: Rc<Font>,
}

pub fn setup_ui() -> Graphical {
    // Window and general setup
    let mut window = Window::new("QuadCopter");
    window.set_line_width(2.0);
    let font = kiss3d::text::Font::default();

    // Drone
    let mut drone_object = window.add_cube(1.0, 0.1, 1.0);
    drone_object.set_color(0.0, 1.0, 0.0);
    let drone_normal_vec = window.add_cube(0.01, 10.0, 0.01);

    // Floor
    window.add_cube(10.0, 0.01, 0.01);
    window.add_cube(0.01, 0.01, 10.0);

    // Camera & lights
    window.set_light(Light::StickToCamera);
    let eye = Point3::new(5.0f32, 5.0, 5.0);
    let mut arc_ball = FirstPerson::new(eye, Point3::origin());
    arc_ball.unbind_movement_keys();

    let graphical = Graphical {
        window,
        drone_normal_vec,
        drone_object,
        arc_ball,
        font,
    };

    graphical
}

pub fn update_ui(graphical: &mut Graphical, drone_rb_ref: &RigidBody, inputs: SMatrix<Real, 4, 1>) {
    let position = kiss3d::nalgebra::Vector3::new(
        drone_rb_ref.translation().x,
        drone_rb_ref.translation().y,
        drone_rb_ref.translation().z,
    );
    graphical
        .drone_object
        .set_local_translation(Translation { vector: position });

    let (roll, pitch, yaw) = drone_rb_ref.rotation().euler_angles();
    graphical
        .drone_object
        .set_local_rotation(UnitQuaternion::from_euler_angles(roll, pitch, yaw));
    graphical
        .drone_normal_vec
        .set_local_translation(Translation { vector: position });
    graphical
        .drone_normal_vec
        .set_local_rotation(UnitQuaternion::from_euler_angles(roll, pitch, yaw));

    graphical.arc_ball.look_at(
        Point3::from(position + kiss3d::nalgebra::Vector3::new(5.0, 5.0, 5.0)),
        Point3::from(position),
    );
    //
    graphical.window.render_with_camera(&mut graphical.arc_ball);
    let vertical_velocity = drone_rb_ref.linvel().y;
    let horizontal_velocity =
        (drone_rb_ref.linvel().x.powi(2) + drone_rb_ref.linvel().z.powi(2)).sqrt();
    let altitude = drone_rb_ref.position().translation.y;
    let x_position = drone_rb_ref.position().translation.x;
    let z_position = drone_rb_ref.position().translation.z;
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
        X = {x_position:.2}\n\
        Z = {z_position:.2}\n\
        Roll = {rolld:.2}\n\
        Pitch = {pitchd:.2}\n\
        Yaw = {yawd:.2}\n"
    );

    graphical.window.draw_text(
        text,
        &Point2::new(100.0, 100.0),
        80.0,
        &graphical.font,
        &Point3::new(1.0, 1.0, 1.0),
    );
}
