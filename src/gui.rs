use egui::plot::{Legend, Line, LineStyle, Plot, PlotPoints, Points};
use egui::*;
extern crate itertools;
extern crate itertools_num;

use itertools_num::linspace;

use crate::controller::Command;
use crate::drone::QuadState;
use std::collections::HashMap;
use std::fmt;
use std::sync::mpsc::Receiver;
use zerocopy::{AsBytes, LayoutVerified};
use zmq::Socket;
use Signal::{Pitch, Roll, Throttle, Yaw};

#[derive(Eq, Hash, PartialEq, Debug)]
enum Signal {
    Throttle,
    Roll,
    Pitch,
    Yaw,
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
}

impl GraphView {
    pub fn new(__cc: &eframe::CreationContext<'_>) -> Self {
        let context = zmq::Context::new();
        let responder = context.socket(zmq::REP).unwrap();
        assert!(responder.bind("tcp://*:5555").is_ok());
        let mut metrics: HashMap<Signal, Vec<f64>> = HashMap::new();
        let keys = [Throttle, Roll, Pitch, Yaw];

        for k in keys {
            metrics.insert(k, vec![]);
        }

        Self {
            label: "Quadcopter Logs".to_owned(),
            initialized: false,
            socket: responder,
            metrics,
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
        let (lv, _rest) = LayoutVerified::<_, QuadState>::new_from_prefix(msg.as_bytes()).unwrap();
        let parsed_one = lv.into_ref();
        self.socket.send("OK", 0).unwrap();
        self.metrics
            .entry(Throttle)
            .and_modify(|v| v.push(parsed_one.input_throttle as f64));
        self.metrics
            .entry(Roll)
            .and_modify(|v| v.push(parsed_one.rotation[0] as f64));
        self.metrics
            .entry(Pitch)
            .and_modify(|v| v.push(parsed_one.rotation[1] as f64));
        self.metrics
            .entry(Yaw)
            .and_modify(|v| v.push(parsed_one.rotation[2] as f64));

        CentralPanel::default().show(ctx, |ui| {
            // Update frame
            ui.ctx().request_repaint();

            // Declare plot
            let mut plot = Plot::new("y_plot").legend(Legend::default());

            let signals_to_plot = [Yaw];
            let time = linspace::<f64>(0., 1., self.metrics[&Throttle].len()).collect();
            let mut lines = vec![];

            for s in signals_to_plot {
                let points = wrap_with_time(&time, &self.metrics[&s]);
                let plot_points: PlotPoints = PlotPoints::from(points);
                let line = Line::new(plot_points)
                    .color(Color32::from_rgb(200, 200, 200))
                    .name(s.to_string());
                lines.push(line);
            }

            // Plot lines
            plot.show(ui, |plot_ui| {
                for line in lines {
                    plot_ui.line(line);
                }
            });
        });
    }
}
