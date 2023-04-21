use gilrs::{Axis, Event, EventType, Gilrs};
use std::sync::mpsc::{Receiver, Sender};
use std::{thread, time};
use zerocopy::{AsBytes, FromBytes};

#[repr(C)]
#[derive(Copy, Clone, Debug, AsBytes, FromBytes)]
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
    // Get event from controller thread and update if new command was sent for either throttle,
    // roll, pitch or yaw.
    let event = rx.try_recv();
    match event {
        Ok((throttle, roll, pitch, yaw)) => {
            command.throttle = throttle.unwrap_or(command.throttle);
            command.pitch = pitch.unwrap_or(command.pitch);
            command.roll = roll.unwrap_or(command.roll);
            command.yaw = yaw.unwrap_or(command.yaw);
        }
        _ => {}
    }
}

pub fn get_commands(tx: Sender<(Option<f32>, Option<f32>, Option<f32>, Option<f32>)>) {
    // Currently tested with Radiomaster ZORRO RF radio controller.
    // Of course, you would need to have a valid model and USB joystick mode enabled.

    // Get controllers
    let mut gilrs = Gilrs::new().unwrap();

    // Polls for new events
    loop {
        // Who's need more than 60 fps for control ?
        thread::sleep(time::Duration::from_millis((1000.0 / 60.0) as u64));
        let mut last_events: (Option<f32>, Option<f32>, Option<f32>, Option<f32>) =
            (None, None, None, None);

        while let Some(Event { event, .. }) = gilrs.next_event() {
            match event {
                EventType::AxisChanged(axis, value, ..) => match axis {
                    Axis::LeftStickX => {
                        // Pitch [-1, 1]
                        last_events.2 = Some(value);
                    }
                    Axis::LeftStickY => {
                        // Yaw [-1, 1]
                        last_events.3 = Some(-value);
                    }
                    Axis::RightStickX => {
                        // Roll [-1, 1]
                        last_events.1 = Some(value);
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

        // Send tuple (throttle, pitch, roll, yaw)
        tx.send(last_events).expect("Couldn't send from thread");
    }
}
