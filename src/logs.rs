extern crate zmq;

mod controller;
mod drone;
mod gui;
mod simulation;

use crate::controller::Command;
use std::time::Duration;
use std::{env, thread};
use zerocopy::{AsBytes, LayoutVerified};
use zmq::{Context, Error, Message};

fn main() {
    // GUI thread
    eframe::run_native(
        "Logs",
        eframe::NativeOptions::default(),
        Box::new(|cc| Box::new(gui::GraphView::new(cc))),
    );
}
