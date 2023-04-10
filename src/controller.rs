use gilrs::{Axis, Button, Event, EventType, Gilrs};
use nalgebra::vector;
use std::sync::mpsc::Sender;
use std::{thread, time};

pub fn get_commands(tx: Sender<(Option<f32>, Option<f32>, Option<f32>, Option<f32>)>) {
    let mut gilrs = Gilrs::new().unwrap();

    let mut active_gamepad = None;

    loop {
        thread::sleep(time::Duration::from_millis((1000.0 / 60.0) as u64));
        // Examine new events
        let mut last_event: Option<Event> = None;
        // throttle, pitch, roll, yaw
        let mut last_events: (Option<f32>, Option<f32>, Option<f32>, Option<f32>) =
            (None, None, None, None);
        while let Some(Event { id, event, time }) = gilrs.next_event() {
            match event {
                EventType::AxisChanged(axis, value, code) => match axis {
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
                EventType::ButtonChanged(button, value, code) => {
                    // Throttle [0, 1]
                    last_events.0 = Some(value);
                }
                _ => {}
            }
            last_event = Some(Event { id, event, time });
            active_gamepad = Some(id);
        }

        tx.send(last_events).expect("Couldn't send from thread");
    }
}
