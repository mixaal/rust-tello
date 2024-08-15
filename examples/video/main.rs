use std::{sync::mpsc, thread, time::Duration};

use rust_tello::TelloController;

pub fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::WARN)
        .init();

    let (tx, rx) = mpsc::channel();
    let mut tello = TelloController::new();

    let (update_tx, update_rx) = rust_tello::comm_channel();
    let h = tello.start_ctrl_receiver(update_tx);
    tello.start_video_receiver(tx, 262_144);

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
