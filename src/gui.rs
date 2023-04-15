use egui::plot::{Legend, Line, LineStyle, Plot, PlotPoints, Points};
use egui::*;
extern crate itertools;
extern crate itertools_num;

use itertools_num::linspace;

use crate::controller::Command;
use std::sync::mpsc::Receiver;
use zerocopy::{AsBytes, LayoutVerified};
use zmq::Socket;

pub struct GraphView {
    label: String,
    initialized: bool,
    socket: Socket,
    throttle: Vec<f64>,
    pitch: Vec<f64>,
    roll: Vec<f64>,
    yaw: Vec<f64>,
}

impl GraphView {
    pub fn new(__cc: &eframe::CreationContext<'_>) -> Self {
        let context = zmq::Context::new();
        let responder = context.socket(zmq::REP).unwrap();
        assert!(responder.bind("tcp://*:5555").is_ok());
        let throttle = vec![];
        let pitch = vec![];
        let roll = vec![];
        let yaw = vec![];

        Self {
            label: "Quadcopter Logs".to_owned(),
            initialized: false,
            socket: responder,
            throttle,
            pitch,
            roll,
            yaw,
        }
    }
}

fn wrap_with_time(time: &Vec<f64>, vec: &Vec<f64>) -> Vec<[f64; 2]> {
    vec.iter()
        .zip(time)
        .map(|(v, t)| [t.clone(), v.clone()])
        .collect()
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

        let (lv, _rest) = LayoutVerified::<_, Command>::new_from_prefix(msg.as_bytes()).unwrap();
        let parsed_one = lv.into_ref();
        self.socket.send("OK", 0).unwrap();
        self.throttle.push(parsed_one.throttle as f64);
        self.pitch.push(parsed_one.pitch as f64);
        self.roll.push(parsed_one.roll as f64);
        self.yaw.push(parsed_one.yaw as f64);

        CentralPanel::default().show(ctx, |ui| {
            // Update frame
            ui.ctx().request_repaint();

            // Declare plot
            let mut plot = Plot::new("y_plot").legend(Legend::default());

            // Convert x position data to PlotPoints
            let time = linspace::<f64>(0., 1., self.throttle.len()).collect();
            let throttle_points = wrap_with_time(&time, &self.throttle);
            let pitch_points = wrap_with_time(&time, &self.pitch);
            let roll_points = wrap_with_time(&time, &self.roll);
            let yaw_points = wrap_with_time(&time, &self.yaw);

            let throttle_plot_points: PlotPoints = PlotPoints::from(throttle_points);
            let pitch_plot_points: PlotPoints = PlotPoints::from(pitch_points);
            let roll_plot_points: PlotPoints = PlotPoints::from(roll_points);
            let yaw_plot_points: PlotPoints = PlotPoints::from(yaw_points);

            // Define plot elements
            let throttle_line = Line::new(throttle_plot_points)
                .color(Color32::from_rgb(100, 100, 200))
                .name("Throttle");

            let pitch_line = Line::new(pitch_plot_points)
                .color(Color32::from_rgb(200, 100, 200))
                .name("Pitch");

            let roll_line = Line::new(roll_plot_points)
                .color(Color32::from_rgb(100, 200, 100))
                .name("Roll");

            let yaw_line = Line::new(yaw_plot_points)
                .color(Color32::from_rgb(255, 255, 0))
                .name("Yaw");

            // Plot lines
            plot.show(ui, |plot_ui| {
                plot_ui.line(throttle_line);
                plot_ui.line(pitch_line);
                plot_ui.line(roll_line);
                plot_ui.line(yaw_line);
            });
        });
    }
}
