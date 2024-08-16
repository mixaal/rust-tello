use std::{thread, time::Duration};

use rust_gamepad::gamepad::{self, Buttons, Gamepad, GamepadState};
use rust_tello::TelloController;

const SENSITIVITY: f32 = 1.0;

pub fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .init();

    let js = Gamepad::new("/dev/input/js0", gamepad::XBOX_MAPPING.clone());
    js.background_handler();
    let mut tello = TelloController::new();

    let (update_tx, _) = rust_tello::comm_channel();
    let _h = tello.start_ctrl_receiver(update_tx);

    tello.connect();
    loop {
        tracing::info!("waiting to connect to tello...");
        if tello.is_connected() {
            tracing::info!("connected to tello");
            break;
        }
        thread::sleep(Duration::from_secs(1));
    }
    tello.start_stick_update();
    tracing::info!("use gamepad to fly the drone");
    let mut last_state = GamepadState::initial();
    loop {
        let st = js.state();
        // let a = st.a();
        // let b = st.b();
        // let x = st.x();
        // let y = st.y();
        let rb = st.rb();
        let lb = st.lb();
        let horiz = st.horiz();
        let vert = st.vert();
        let rt = st.rt(SENSITIVITY);
        let lt = st.lt(SENSITIVITY);
        // let start = st.start();
        // let select = st.select();
        // let r_stick = st.r_stick();
        let l_stick = st.l_stick(SENSITIVITY);
        // let rx = r_stick.0;
        // let ry = r_stick.1;
        // let lx = l_stick.0;
        let ly = l_stick.1;
        tracing::debug!("joy={:?}", st);

        if st.button_clicked(Buttons::START, &last_state) {
            let flying = tello.flying();
            if !flying {
                tracing::info!("takeoff");
                tello.takeoff();
            } else {
                tracing::info!("land");
                tello.land();
            }
        }
        if st.button_clicked(Buttons::SELECT, &last_state) {
            tracing::info!("hover");
            tello.hover();
        }

        if st.button_clicked(Buttons::A, &last_state) {
            tracing::info!("take picture");
            tello.take_picture();
        }
        if rb != lb {
            if !lb {
                tracing::info!("turn right");
                tello.turn_clockwise(1.0);
            } else {
                tracing::info!("turn left");
                tello.turn_counter_clockwise(1.0);
            }
        } else {
            tello.turn_clockwise(0.0); // reset rotation
        }

        let dt = rt - lt;
        tracing::info!(ly, dt, "forward delta, up delta");
        if vert != 0.0 {
            tello.forward(-vert);
        } else {
            tello.forward(dt);
        }
        tello.up(-ly);
        tello.right(horiz);

        last_state = st;
        thread::sleep(Duration::from_millis(200));
    }

    // let _ = h.join();
}
