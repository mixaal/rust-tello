use std::{sync::mpsc, thread, time::Duration};

use rust_tello::tello::TelloController;

pub fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::WARN)
        .init();

    let (tx, rx) = mpsc::channel();
    let mut tello = TelloController::new();

    let h = tello.start_ctrl_receiver();
    tello.start_video_receiver(tx);

    tello.start_video_contoller(); // send video request every 500ms if video is on
    tello.connect();
    loop {
        tracing::info!("waiting to connect to tello...");
        if tello.is_connected() {
            tracing::info!("connected to tello");
            break;
        }
        thread::sleep(Duration::from_secs(1));
    }

    tello.toggle_video(); // toggle video on

    tello.start_mplayer(rx);

    tracing::info!("waiting for main thread to finish");
    let _ = h.join();
}
