use std::{thread, time::Duration};

use rust_tello::TelloController;

pub fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::TRACE)
        .init();

    let mut tello = TelloController::new();
    let (update_tx, update_rx) = rust_tello::comm_channel();
    let h = tello.start_ctrl_receiver(update_tx);
    tello.connect();
    loop {
        tracing::info!("waiting to connect to tello...");
        if tello.is_connected() {
            tracing::info!("connected to tello");
            break;
        }
        thread::sleep(Duration::from_secs(1));
    }
    tracing::info!("Sleep for 4 secs and take 10 pictures...");
    thread::sleep(Duration::from_secs(4));
    for _ in 0..10 {
        tello.take_picture();
        thread::sleep(Duration::from_secs(1));
        tracing::info!("take picture sent...");
    }

    let _ = h.join();
}
