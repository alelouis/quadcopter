use egui::plot::{Legend, Line, LineStyle, Plot, PlotPoints, Points};
use egui::*;
extern crate itertools;
extern crate itertools_num;

use itertools_num::linspace;

use crate::controller::Command;
use crate::drone::QuadState;
use colorous;
use nalgebra::Vector3;
use std::collections::HashMap;
use std::fmt;
use std::sync::mpsc::Receiver;
use std::time::Instant;
use zerocopy::{AsBytes, LayoutVerified};
use zmq::Socket;
use Signal::{
    Altitude, Pitch, Roll, Rotor1Velocity, Rotor2Velocity, Rotor3Velocity, Rotor4Velocity,
    Throttle, Velocity, Yaw,
};

#[derive(Eq, Hash, PartialEq, Debug, Copy, Clone)]
enum Signal {
    Throttle,
    Roll,
    Pitch,
    Yaw,
    Rotor1Velocity,
    Rotor2Velocity,
    Rotor3Velocity,
    Rotor4Velocity,
    Velocity,
    Altitude,
}

impl fmt::Display for Signal {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

pub struct GraphView {
    label: String,
    initialized: bool,
    socket: Socket,
    metrics: HashMap<Signal, Vec<f64>>,
    start: Instant,
}

impl GraphView {
    pub fn new(__cc: &eframe::CreationContext<'_>) -> Self {
        let context = zmq::Context::new();
        let responder = context.socket(zmq::REP).unwrap();
        assert!(responder.bind("tcp://*:5555").is_ok());
        let mut metrics: HashMap<Signal, Vec<f64>> = HashMap::new();
        let keys = [
            Throttle,
            Roll,
            Pitch,
            Yaw,
            Rotor1Velocity,
            Rotor2Velocity,
            Rotor3Velocity,
            Rotor4Velocity,
            Velocity,
            Altitude,
        ];

        for k in keys {
            metrics.insert(k, vec![]);
        }

        Self {
            label: "Quadcopter Logs".to_owned(),
            initialized: false,
            socket: responder,
            metrics,
            start: Instant::now(),
        }
    }
}

fn wrap_with_time(time: &Vec<f64>, vec: &Vec<f64>) -> Vec<[f64; 2]> {
    vec.iter()
        .zip(time)
        .map(|(v, t)| [t.clone(), v.clone()])
        .collect()
}

fn generate_lines(
    signals: Vec<Signal>,
    metrics: HashMap<Signal, Vec<f64>>,
    start: Instant,
    color_idx: Option<usize>,
) -> Vec<Line> {
    let mut plot_lines = vec![];
    let colors = colorous::PASTEL2;
    let time =
        linspace::<f64>(0., start.elapsed().as_secs_f64(), metrics[&Throttle].len()).collect();
    for (idx, s) in signals.iter().enumerate() {
        let points = wrap_with_time(&time, &metrics[&s]);
        let scoped_points = {
            if points.len() > 10 * 60 {
                points.as_slice()[points.len() - 10 * 60..].to_vec()
            } else {
                points
            }
        };
        let plot_points: PlotPoints = PlotPoints::from(scoped_points);
        if let Some(force_idx) = color_idx {
            let line = Line::new(plot_points)
                .color(Color32::from_rgb(
                    colors[force_idx].r,
                    colors[force_idx].g,
                    colors[force_idx].b,
                ))
                .width(2.0)
                .name(s.to_string());
            plot_lines.push(line);
        } else {
            let line = Line::new(plot_points)
                .color(Color32::from_rgb(
                    colors[idx].r,
                    colors[idx].g,
                    colors[idx].b,
                ))
                .name(s.to_string());
            plot_lines.push(line);
        }
    }
    plot_lines
}

impl eframe::App for GraphView {
    fn update(&mut self, ctx: &Context, frame: &mut eframe::Frame) {
        let mut msg = zmq::Message::new();
        loop {
            let msg_recv = self.socket.recv_msg(0);
            if msg_recv.is_ok() {
                msg = msg_recv.unwrap();
            } else {
                break;
            }
        }
        let (lv, _rest) = LayoutVerified::<_, QuadState>::new_from_prefix(msg.as_bytes()).unwrap();
        let parsed_one = lv.into_ref();
        self.socket.send("OK", 0).unwrap();
        self.metrics
            .entry(Throttle)
            .and_modify(|v| v.push(parsed_one.input_throttle as f64));
        self.metrics
            .entry(Roll)
            .and_modify(|v| v.push(parsed_one.rotation[0].to_degrees() as f64));
        self.metrics
            .entry(Pitch)
            .and_modify(|v| v.push(parsed_one.rotation[1].to_degrees() as f64));
        self.metrics
            .entry(Yaw)
            .and_modify(|v| v.push(parsed_one.rotation[2].to_degrees() as f64));
        self.metrics
            .entry(Rotor1Velocity)
            .and_modify(|v| v.push(parsed_one.rotors_velocities[0] as f64));
        self.metrics
            .entry(Rotor2Velocity)
            .and_modify(|v| v.push(parsed_one.rotors_velocities[1] as f64));
        self.metrics
            .entry(Rotor3Velocity)
            .and_modify(|v| v.push(parsed_one.rotors_velocities[2] as f64));
        self.metrics
            .entry(Rotor4Velocity)
            .and_modify(|v| v.push(parsed_one.rotors_velocities[3] as f64));
        self.metrics.entry(Velocity).and_modify(|v| {
            v.push({
                let vel_array = parsed_one.velocity;
                let vel = Vector3::new(vel_array[0], vel_array[1], vel_array[2]);
                vel.norm()
            } as f64)
        });
        self.metrics
            .entry(Altitude)
            .and_modify(|v| v.push(parsed_one.position[2] as f64));

        let n_plots = 6.0;

        CentralPanel::default().show(ctx, |ui| {
            ui.ctx().request_repaint();

            // Declare plot
            let mut plot_throttle = Plot::new("throttle")
                .legend(Legend::default())
                .height(ui.available_height() / n_plots);

            let mut plot_roll_pitch_yaw = Plot::new("roll_pitch_yaw")
                .legend(Legend::default())
                .height(ui.available_height() / n_plots)
                .center_y_axis(true)
                .include_y(-180)
                .include_y(180);

            let mut plot_rotors = Plot::new("rotors")
                .legend(Legend::default())
                .height(ui.available_height() / n_plots);

            let mut plot_altitude = Plot::new("altitude")
                .legend(Legend::default())
                .height(ui.available_height() / n_plots)
                .include_y(0.0);

            let mut plot_velocity = Plot::new("velocity")
                .legend(Legend::default())
                .height(ui.available_height() / n_plots)
                .include_y(0.0);

            let plot_throttle_signal = vec![Throttle];
            let plot_roll_pitch_yaw_signals = vec![Roll, Pitch, Yaw];
            let plot_rotor_signals = vec![
                Rotor1Velocity,
                Rotor2Velocity,
                Rotor3Velocity,
                Rotor4Velocity,
            ];
            let plot_velocity_signals = vec![Velocity];
            let plot_altitude_signals = vec![Altitude];

            let mut plot_throttle_lines = generate_lines(
                plot_throttle_signal,
                self.metrics.clone(),
                self.start,
                Some(1),
            );
            let mut plot_roll_pitch_yaw_lines = generate_lines(
                plot_roll_pitch_yaw_signals,
                self.metrics.clone(),
                self.start,
                None,
            );
            let mut plot_rotors_lines =
                generate_lines(plot_rotor_signals, self.metrics.clone(), self.start, None);
            let mut plot_velocity_lines = generate_lines(
                plot_velocity_signals,
                self.metrics.clone(),
                self.start,
                Some(1),
            );
            let mut plot_altitude_lines = generate_lines(
                plot_altitude_signals,
                self.metrics.clone(),
                self.start,
                Some(2),
            );

            // Plot lines
            ui.label("Throttle");
            plot_throttle.show(ui, |plot_ui| {
                for line in plot_throttle_lines {
                    plot_ui.line(line);
                }
            });
            ui.add_space(20.0);

            ui.label("Roll, Pitch, Yaw (degrees)");
            plot_roll_pitch_yaw.show(ui, |plot_ui| {
                for line in plot_roll_pitch_yaw_lines {
                    plot_ui.line(line);
                }
            });
            ui.add_space(20.0);

            ui.label("Rotors speed");
            plot_rotors.show(ui, |plot_ui| {
                for line in plot_rotors_lines {
                    plot_ui.line(line);
                }
            });
            ui.add_space(20.0);

            ui.label("Velocity (m/s)");
            plot_velocity.show(ui, |plot_ui| {
                for line in plot_velocity_lines {
                    plot_ui.line(line);
                }
            });
            ui.add_space(20.0);

            ui.label("Altitude (m)");
            plot_altitude.show(ui, |plot_ui| {
                for line in plot_altitude_lines {
                    plot_ui.line(line);
                }
            });
        });
    }
}
