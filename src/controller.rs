use gilrs::{Axis, Event, EventType, Gilrs};
use std::sync::mpsc::{Receiver, Sender};
use std::{thread, time};

#[derive(Copy, Clone)]
pub struct Command {
    pub throttle: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

impl Command {
    pub fn new() -> Self {
        Self {
            throttle: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

pub fn update_commands(
    mut command: &mut Command,
    rx: &Receiver<(Option<f32>, Option<f32>, Option<f32>, Option<f32>)>,
) {
    let event = rx.try_recv();
    match event {
        Ok((throttle, pitch, roll, yaw)) => {
            command.throttle = throttle.unwrap_or(command.throttle);
            command.pitch = pitch.unwrap_or(command.pitch);
            command.roll = roll.unwrap_or(command.roll);
            command.yaw = yaw.unwrap_or(command.yaw);
        }
        _ => {}
    }
}

pub fn get_commands(tx: Sender<(Option<f32>, Option<f32>, Option<f32>, Option<f32>)>) {
    let mut gilrs = Gilrs::new().unwrap();

    loop {
        thread::sleep(time::Duration::from_millis((1000.0 / 60.0) as u64));
        // throttle, pitch, roll, yaw
        let mut last_events: (Option<f32>, Option<f32>, Option<f32>, Option<f32>) =
            (None, None, None, None);
        while let Some(Event { event, .. }) = gilrs.next_event() {
            match event {
                EventType::AxisChanged(axis, value, ..) => match axis {
                    Axis::LeftStickX => {
                        // Roll [-1, 1]
                        last_events.2 = Some(value);
                    }
                    Axis::LeftStickY => {
                        // pitch [-1, 1]
                        last_events.1 = Some(-value);
                    }
                    Axis::RightStickX => {
                        // Yaw [-1, 1]
                        last_events.3 = Some(value);
                    }
                    _ => {}
                },
                EventType::ButtonChanged(_, value, ..) => {
                    // Throttle [0, 1]
                    last_events.0 = Some(value);
                }
                _ => {}
            }
        }

        tx.send(last_events).expect("Couldn't send from thread");
    }
}
