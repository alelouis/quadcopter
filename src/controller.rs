use gilrs::{Axis, Event, EventType, Gilrs};
use std::sync::mpsc::Sender;
use std::{thread, time};

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
