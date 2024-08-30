use std::{
    io::Write,
    sync::{
        atomic::Ordering,
        mpsc::{self, Receiver, Sender},
        Arc, RwLock,
    },
    thread::{self, JoinHandle},
    time::Duration,
};

use messages::{FlightData, LightData, LogData, WifiData};
use tello::{Stick, Tello};

pub(crate) mod crc;
pub(crate) mod dump;
pub(crate) mod env;
pub mod messages;
pub(crate) mod tello;
pub(crate) mod utils;

#[macro_use]
extern crate lazy_static;

pub type VideoRecvChannel = Receiver<Vec<u8>>;
pub type VideoPublishChannel = Sender<Vec<u8>>;
pub type VideoChannel = (VideoPublishChannel, VideoRecvChannel);

pub type UpdateDataPublishChannel = Sender<UpdateData>;
pub type UpdateDataRecvChannel = Receiver<UpdateData>;
pub type UpdateDataChannel = (UpdateDataPublishChannel, UpdateDataRecvChannel);

pub fn comm_channel() -> UpdateDataChannel {
    mpsc::channel()
}

pub fn video_channel() -> VideoChannel {
    mpsc::channel()
}

#[derive(Debug)]
pub struct UpdateData {
    pub flight: Option<FlightData>,
    pub wifi: Option<WifiData>,
    pub light: Option<LightData>,
    pub log: Option<LogData>,
}

pub struct TelloController {
    video: Arc<RwLock<bool>>,
    inner: Arc<Tello>,
}

impl TelloController {
    pub fn new() -> Self {
        Self {
            video: Arc::new(RwLock::new(false)),
            inner: Arc::new(Tello::new()),
        }
    }

    pub fn start_mplayer(&self, video_channel: VideoRecvChannel) -> Option<JoinHandle<()>> {
        let method_name = "start_mplayer";
        let mut err_cnt = 0;
        let stdin = utils::start_mplayer_with_stdin(false);
        if stdin.is_none() {
            return None;
        }
        let mut stdin = stdin.unwrap();
        let jh = thread::spawn(move || loop {
            let video_data = video_channel.recv();
            if video_data.is_err() {
                tracing::warn!(
                    method_name,
                    "can't get video data: {}",
                    video_data.err().unwrap()
                );
                err_cnt += 1;
                if err_cnt > 10 {
                    return;
                }
                continue;
            }
            err_cnt = 0; //reset error counter
            let video_data = video_data.unwrap();
            utils::append_to_file("video.dump", video_data.clone());
            let r = stdin.write_all(&video_data);
            if r.is_err() {
                tracing::warn!(
                    method_name,
                    "unable to write data to maplyer: {}",
                    r.err().unwrap()
                );
            }
        });
        Some(jh)
    }

    pub fn set_sticks(&self, st: &Stick) {
        let mut g = self.inner.stick.write().unwrap();
        *g = st.clone();
    }

    pub fn takeoff(&self) {
        self.inner.takeoff();
    }

    pub fn land(&self) {
        self.inner.land();
    }

    pub fn forward(&self, amt: f32) {
        self.inner.forward(amt);
    }

    pub fn backward(&self, amt: f32) {
        self.inner.backward(amt);
    }

    pub fn up(&self, amt: f32) {
        self.inner.up(amt);
    }

    pub fn down(&self, amt: f32) {
        self.inner.down(amt);
    }

    pub fn left(&self, amt: f32) {
        self.inner.left(amt);
    }

    pub fn right(&self, amt: f32) {
        self.inner.right(amt);
    }

    pub fn turn_clockwise(&self, amt: f32) {
        self.inner.turn_clockwise(amt);
    }

    pub fn turn_counter_clockwise(&self, amt: f32) {
        self.inner.turn_counter_clockwise(amt);
    }

    pub fn hover(&self) {
        self.inner.hover();
    }

    pub fn flying(&self) -> bool {
        let g = self.inner.flying.read().unwrap();
        *g
    }

    // Receive the control data from the tello
    pub fn start_ctrl_receiver(&self, tx: UpdateDataPublishChannel) -> JoinHandle<()> {
        let self_local = self.inner.clone();
        let j = thread::spawn(move || self_local.ctrl_receiver(tx));
        j
    }

    // Captures the video data
    pub fn start_video_receiver(&self, video_channel: VideoPublishChannel) -> JoinHandle<()> {
        let self_local = self.inner.clone();
        let j = thread::spawn(move || self_local.video_receiver(video_channel));
        j
    }

    // Send movement updates to the drone
    pub fn start_stick_update(&self) -> JoinHandle<()> {
        let self_local = self.inner.clone();
        let j = thread::spawn(move || self_local.send_update_sticks());
        j
    }

    pub fn toggle_video(&mut self) {
        let mut g = self.video.write().unwrap();
        *g = !*g;
    }

    pub fn start_video_contoller(&self) -> JoinHandle<()> {
        let self_local = self.inner.clone();
        let video = self.video.clone();
        let j = thread::spawn(move || loop {
            let g = video.read().unwrap();
            let video_on = *g;
            drop(g);
            if video_on {
                self_local.query_video_sps_pps();
            }

            thread::sleep(Duration::from_millis(500));
        });
        j
    }

    pub fn is_connected(&self) -> bool {
        self.inner.connected.load(Ordering::Relaxed)
    }

    pub fn connect(&mut self) {
        let method_name = "tello_connect";
        tracing::info!(
            method_name,
            self.inner.remote_addr,
            self.inner.video_port,
            "start"
        );
        let msg = messages::connect(self.inner.video_port);
        let r = self.inner.ctrl_conn.send_to(&msg, &self.inner.remote_addr);
        if r.is_err() {
            let errmsg = format!("can't connect to tello: {}", r.unwrap_err());
            utils::fatal(&errmsg);
        }
        let sent_bytes = r.unwrap();
        tracing::info!(method_name, sent_bytes, "connecting...");
    }

    pub fn take_picture(&self) {
        let method_name = "take_picture";
        tracing::debug!(method_name, self.inner.remote_addr, "send");
        let msg = messages::take_picture(self.inner.ctrl_seq.fetch_add(1, Ordering::Relaxed));
        let r = self.inner.ctrl_conn.send_to(&msg, &self.inner.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to take picture: {}", r.unwrap_err());
        }
    }

    pub fn query_video_sps_pps(&self) {
        self.inner.query_video_sps_pps()
    }
}
