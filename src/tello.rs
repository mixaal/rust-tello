// Original go version is from Steve Merrony, see the copyright from his original
// go source file bellow: https://github.com/SMerrony/tello/blob/master/tello.go

// ---------------------------snip-----------------------------
// tello.go

// Copyright (C) 2018  Steve Merrony

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// ---------------------------snip-----------------------------
use chrono::{Datelike, Timelike};
use std::{
    collections::HashMap,
    net::UdpSocket,
    sync::{
        atomic::{AtomicBool, AtomicU16, AtomicU64, Ordering},
        Arc, RwLock,
    },
    thread::{self},
    time::{Duration, Instant},
};

use crate::{
    dump::ConnDumper,
    env,
    messages::{
        self, FileChunk, FileInternal, FilePiece, FileType, FlightData, LightData, LogData,
        TelloPacket, WifiData,
    },
    utils, UpdateData, UpdateDataPublishChannel, VideoPublishChannel,
};

const RC_VAL_MIN: i16 = 364;
const RC_VAL_MAX: i16 = 1684;

// pub type VideoFrameHandler = Arc<dyn Fn(usize, &Vec<u8>) -> () + Send + Sync>;

#[derive(Debug, Clone)]
pub struct Stick {
    rx: f32,
    ry: f32,
    lx: f32,
    ly: f32,
}

impl Stick {
    fn default() -> Self {
        Self {
            rx: 0.0,
            ry: 0.0,
            lx: 0.0,
            ly: 0.0,
        }
    }

    pub fn new(r: (f32, f32), l: (f32, f32)) -> Self {
        Self {
            rx: r.0,
            ry: r.1,
            lx: l.0,
            ly: l.1,
        }
    }
}

#[derive(Debug)]
pub(crate) struct Tello {
    pub ctrl_port: u16,
    pub local_port: u16,
    pub video_port: u16,
    pub remote_addr: String,
    pub ctrl_conn: UdpSocket,
    pub recv_conn: UdpSocket,
    pub video_conn: UdpSocket,
    pub connected: &'static AtomicBool,
    pub ctrl_dumper: Option<ConnDumper>,
    pub(crate) ctrl_seq: &'static AtomicU16,
    files: Arc<RwLock<HashMap<u16, FileInternal>>>,
    pub(crate) stick: Arc<RwLock<Stick>>,
    pub(crate) flying: Arc<RwLock<bool>>,
    video_dump_file: String,
}

impl Clone for Tello {
    fn clone(&self) -> Self {
        Self {
            ctrl_port: self.ctrl_port.clone(),
            local_port: self.local_port.clone(),
            video_port: self.video_port.clone(),
            remote_addr: self.remote_addr.clone(),
            ctrl_conn: utils::udp_sock_clone(&self.ctrl_conn),
            recv_conn: utils::udp_sock_clone(&self.recv_conn),
            video_conn: utils::udp_sock_clone(&self.video_conn),
            connected: self.connected,
            ctrl_dumper: self.ctrl_dumper.clone(),
            ctrl_seq: self.ctrl_seq,
            files: self.files.clone(),
            stick: self.stick.clone(),
            flying: self.flying.clone(),
            video_dump_file: self.video_dump_file.clone(),
        }
    }
}

static TELLO_CONNECTED: AtomicBool = AtomicBool::new(false);
static TELLO_CTRL_SEQ: AtomicU16 = AtomicU16::new(0);
static TELLO_CTRL_PACKET_COUNTER: AtomicU64 = AtomicU64::new(0);

impl Tello {
    pub fn new() -> Self {
        let video_dump_file = format!("./video-{}", utils::now_secs());
        let ctrl_port = *env::ENV_TELLO_CTRL_PORT;
        let local_port = *env::ENV_TELLO_LOCAL_PORT;
        let video_port = *env::ENV_TELLO_VIDEO_PORT;
        let tello_addr = env::ENV_TELLO_ADDR.clone();
        let remote_addr = format!("{tello_addr}:{ctrl_port}");
        let local_addr = format!("0.0.0.0:{local_port}");
        let video_addr = format!("0.0.0.0:{video_port}");
        // let (tx, rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = mpsc::channel();
        Self {
            ctrl_conn: utils::udp_sock("0.0.0.0:0"),
            recv_conn: utils::udp_sock(&local_addr),
            video_conn: utils::udp_sock(&video_addr),
            ctrl_port,
            local_port,
            video_port,
            remote_addr,
            connected: &TELLO_CONNECTED,
            ctrl_dumper: Some(ConnDumper::new("ctrl_comm", &TELLO_CTRL_PACKET_COUNTER)),
            ctrl_seq: &TELLO_CTRL_SEQ,
            files: Arc::new(RwLock::new(HashMap::new())),
            stick: Arc::new(RwLock::new(Stick::default())),
            flying: Arc::new(RwLock::new(false)),
            video_dump_file,
        }
    }

    pub(crate) fn takeoff(&self) {
        let method_name = "takeoff";
        tracing::debug!(method_name, "send");
        let msg = messages::do_takeoff(self.ctrl_seq.fetch_add(1, Ordering::Relaxed));
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to take off: {}", r.unwrap_err());
        }
    }

    pub(crate) fn land(&self) {
        let method_name = "land";
        tracing::debug!(method_name, "send");
        let msg = messages::do_land(self.ctrl_seq.fetch_add(1, Ordering::Relaxed));
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to land: {}", r.unwrap_err());
        }
    }

    pub(crate) fn forward(&self, amt: f32) {
        let method_name = "forward";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((0.0, amt), (0.0, 0.0));
        let mut g = self.stick.write().unwrap();
        g.ry = amt;
    }

    pub(crate) fn backward(&self, amt: f32) {
        let method_name = "backward";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((0.0, -amt), (0.0, 0.0));
        let mut g = self.stick.write().unwrap();
        g.ry = -amt;
    }

    pub(crate) fn left(&self, amt: f32) {
        let method_name = "left";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((-amt, 0.0), (0.0, 0.0));
        let mut g = self.stick.write().unwrap();
        g.rx = -amt;
    }

    pub(crate) fn right(&self, amt: f32) {
        let method_name = "right";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((amt, 0.0), (0.0, 0.0));
        let mut g = self.stick.write().unwrap();
        g.rx = amt;
    }

    pub(crate) fn up(&self, amt: f32) {
        let method_name = "up";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((0.0, 0.0), (0.0, amt));
        let mut g = self.stick.write().unwrap();
        g.ly = amt;
        // *g = st.clone();
    }

    pub(crate) fn down(&self, amt: f32) {
        let method_name = "down";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((0.0, 0.0), (0.0, -amt));
        let mut g = self.stick.write().unwrap();
        // *g = st.clone();
        g.ly = -amt;
    }

    pub(crate) fn turn_clockwise(&self, amt: f32) {
        let method_name = "turn_clockwise";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((0.0, 0.0), (amt, 0.0));
        let mut g = self.stick.write().unwrap();
        // *g = st.clone();
        g.lx = amt;
    }

    pub(crate) fn turn_counter_clockwise(&self, amt: f32) {
        let method_name = "turn_counter_clockwise";
        tracing::debug!(method_name, amt, "update");
        // let st = Stick::new((0.0, 0.0), (-amt, 0.0));
        let mut g = self.stick.write().unwrap();
        // *g = st.clone();
        g.lx = -amt;
    }

    pub(crate) fn hover(&self) {
        let method_name = "hover";
        tracing::debug!(method_name, "update");
        let st = Stick::new((0.0, 0.0), (0.0, 0.0));
        let mut g = self.stick.write().unwrap();
        *g = st.clone();
    }

    pub(crate) fn send_file_size(&self) {
        let method_name = "send_file_size";
        tracing::debug!(method_name, "send");
        let msg = messages::ack_file_size(self.ctrl_seq.fetch_add(1, Ordering::Relaxed));
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to ack file size: {}", r.unwrap_err());
        }
    }

    pub(crate) fn ack_file_piece(&self, done: bool, f_id: u16, piece_no: u32) {
        let method_name = "ack_file_piece";
        tracing::debug!(method_name, self.remote_addr, done, f_id, piece_no, "send");
        let msg = messages::ack_file_piece(
            self.ctrl_seq.fetch_add(1, Ordering::Relaxed),
            done,
            f_id,
            piece_no,
        );
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to ack file piece: {}", r.unwrap_err());
        }
    }

    pub(crate) fn ack_file_done(&self, f_id: u16, size: u32) {
        let method_name = "ack_file_done";
        tracing::debug!(method_name, self.remote_addr, f_id, size, "send");
        let msg = messages::file_done(self.ctrl_seq.fetch_add(1, Ordering::Relaxed), f_id, size);
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to ack file done: {}", r.unwrap_err());
        }
    }

    pub(crate) fn ack_log_header(&self, pl: &Vec<u8>) {
        let method_name = "ack_log_header";
        tracing::debug!(method_name, self.remote_addr, "send");
        let buf: [u8; 2] = [pl[0], pl[1]];
        let msg =
            TelloPacket::ack_log(self.ctrl_seq.fetch_add(1, Ordering::Relaxed), &buf).to_buffer();
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to ack log header: {}", r.unwrap_err());
        }
    }

    pub(crate) fn send_date_time(&self) {
        let now = chrono::Local::now();
        let method_name = "send_date_time";
        tracing::debug!(method_name, self.remote_addr, "send");
        let ms = now.timestamp_subsec_millis() as u16;
        let msg = messages::send_date_time(
            self.ctrl_seq.fetch_add(1, Ordering::Relaxed),
            now.year() as u16,
            now.month() as u16,
            now.day() as u16,
            now.hour() as u16,
            now.minute() as u16,
            now.second() as u16,
            ms,
        );
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(method_name, "unable to ack log header: {}", r.unwrap_err());
        }
    }

    pub fn query_video_sps_pps(&self) {
        let method_name = "query_video_sps_pps";
        tracing::debug!(method_name, self.remote_addr, "send");
        let msg = messages::query_video_spsfps();
        let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
        if r.is_err() {
            tracing::warn!(
                method_name,
                "unable to send query for sps/pps: {}",
                r.unwrap_err()
            );
        }
    }

    pub fn process_packet(&self, pkt: &TelloPacket, tx: &UpdateDataPublishChannel) {
        let method_name = "process_packet";
        match pkt.message_id {
            messages::MSG_DO_LAND => {
                tracing::info!(method_name, "do land msg recv");
            }
            messages::MSG_DO_TAKE_PIC => {
                tracing::info!(method_name, "do take pic recv: {:?}", pkt.payload);
            }
            messages::MSG_DO_TAKEOFF => {
                tracing::info!(method_name, "do take off recv");
            }
            messages::MSG_FILE_SIZE => {
                tracing::info!(method_name, "file size received");
                let file_internal = FileInternal::new(&pkt.payload);
                tracing::info!(method_name, "file internal: {:?}", file_internal);
                if file_internal.file_type == FileType::FtJPEG {
                    self.files
                        .write()
                        .unwrap()
                        .insert(file_internal.f_id, file_internal);
                    self.send_file_size();
                } else {
                    tracing::warn!(method_name, "unknown file type received");
                }
            }
            messages::MSG_FILE_DATA => {
                tracing::info!(method_name, "file data received");
                let chunk = FileChunk::new(&pkt.payload);
                tracing::info!(method_name, "chunk: {:?}", chunk);
                let piece_no = chunk.piece_num;
                let f_id = chunk.f_id;
                tracing::info!(method_name, f_id, piece_no, "file data chunk: {:?}", chunk);
                self.files
                    .write()
                    .unwrap()
                    .entry(chunk.f_id)
                    .and_modify(|internal_file| {
                        while internal_file.pieces.len() <= chunk.piece_num as usize {
                            internal_file.pieces.push(FilePiece::new());
                        }
                        let this_piece = &mut internal_file.pieces[chunk.piece_num as usize];
                        let l = chunk.chunk_len as u32;
                        let idx = (chunk.chunk_num & 7) as usize;
                        if this_piece.num_chunks < 8 {
                            if this_piece.chunks[idx].is_none() {
                                this_piece.num_chunks += 1;
                                internal_file.accum_size += l;
                                this_piece.chunks[idx] = Some(chunk);
                            }
                        }
                        if this_piece.num_chunks == 8 {
                            tracing::info!(method_name, f_id, piece_no, "ack all 8 chunks");
                            self.ack_file_piece(false, f_id, piece_no);
                        }
                        let accum_size = internal_file.accum_size;
                        if accum_size == internal_file.expected_size {
                            tracing::info!(method_name, f_id, piece_no, "file is of expected size");
                            self.ack_file_piece(true, f_id, piece_no);
                            self.ack_file_done(f_id, accum_size);
                            internal_file.save();
                        }
                    });
            }
            messages::MSG_FLIGHT_STATUS => {
                tracing::info!(method_name, "flight status received");
                let flight_data = FlightData::new(&pkt.payload);
                tracing::info!(method_name, "flight_data: {:?}", flight_data);
                let mut g = self.flying.write().unwrap();
                *g = flight_data.flying;
                drop(g);
                let r = tx.send(UpdateData::from_flight_data(flight_data));
                if r.is_err() {
                    tracing::error!("unable to send flight data: {}", r.err().unwrap());
                }
            }
            messages::MSG_LIGHT_STRENGTH => {
                tracing::info!(method_name, "light strength received");
                let light_strength = LightData::new(&pkt.payload);
                tracing::info!(method_name, "light data: {:?}", light_strength);
                let r = tx.send(UpdateData::from_light_data(light_strength));
                if r.is_err() {
                    tracing::error!("unable to send light health data: {}", r.err().unwrap());
                }
            }
            messages::MSG_LOG_CONFIG => {
                tracing::info!(method_name, "log config received");
            }
            messages::MSG_LOG_HEADER => {
                tracing::info!(method_name, "log header received");
                self.ack_log_header(&pkt.payload);
            }
            messages::MSG_LOG_DATA => {
                tracing::info!(method_name, "log data received");
                let log_data = LogData::new(&pkt.payload);
                tracing::info!("log_data={:?}", log_data);
                if log_data.imu.is_some() || log_data.mvo.is_some() {
                    let r = tx.send(UpdateData::from_log_data(log_data));
                    if r.is_err() {
                        tracing::error!("unable to send flight log data: {}", r.err().unwrap());
                    }
                }
            }
            messages::MSG_QUERY_HEIGHT_LIMIT => {
                tracing::info!(method_name, "max height received");
            }
            messages::MSG_QUERY_LOW_BATT_THRESH => {
                tracing::info!(method_name, "low battery threshold received");
            }
            messages::MSG_QUERY_SSID => {
                tracing::info!(method_name, "SSID received");
            }
            messages::MSG_QUERY_VERSION => {
                tracing::info!(method_name, "version received");
            }
            messages::MSG_QUERY_VIDEO_BITRATE => {
                tracing::info!(method_name, "VBR received");
            }
            messages::MSG_SET_DATE_TIME => {
                tracing::info!(method_name, "send set date time received");
                self.send_date_time();
            }
            messages::MSG_SET_LOW_BATT_THRESH => {
                tracing::info!(method_name, "set low battery threshold received");
            }
            messages::MSG_SMART_VIDEO_STATUS => {
                tracing::info!(method_name, "set smart video status received");
            }
            messages::MSG_SWITCH_PIC_VIDEO => {
                tracing::info!(method_name, "set switch pic video  received");
            }
            messages::MSG_WIFI_STRENGTH => {
                tracing::info!(method_name, "wifi strength info received");
                let info = WifiData::new(&pkt.payload);
                tracing::info!(method_name, "wifi data: {:?}", info);
                let r = tx.send(UpdateData::from_wifi_data(info));
                if r.is_err() {
                    tracing::error!("unable to send wifi data: {}", r.err().unwrap());
                }
            }
            _ => {
                let cmd = pkt.message_id;
                tracing::info!("Not yet supported: {:x}", cmd);
            }
        };
    }

    pub(crate) fn ctrl_receiver(&self, tx: UpdateDataPublishChannel) {
        let method_name = "ctrl_recv";
        let mut buff: [u8; 4096] = [0; 4096];

        loop {
            let r = self.ctrl_conn.recv(&mut buff);
            if r.is_err() {
                tracing::warn!(method_name, "udp read error: {}", r.unwrap_err());
                continue;
            }
            if let Some(ref dumper) = &self.ctrl_dumper {
                dumper.dump(&buff);
            }
            let nread = r.unwrap();
            if !self.connected.load(Ordering::Relaxed) && nread == 11 {
                if utils::contains_any(&buff, "conn_ack:".as_bytes()).is_some() {
                    self.connected.store(true, Ordering::Relaxed);
                } else {
                    tracing::warn!(method_name, "unexpected response to connect request");
                }
                continue;
            }
            if buff[0] != messages::MSG_HDR {
                tracing::warn!(method_name, "packet unknown header: {:x}", buff[0]);
                continue;
            }
            let pkt = TelloPacket::from_buffer(&buff);
            self.process_packet(&pkt, &tx);
        }
    }

    pub(crate) fn video_receiver(&self, video_channel: VideoPublishChannel, min_size: usize) {
        let method_name = "video_recv";
        let mut buff: [u8; 2048] = [0; 2048];

        loop {
            let r = self.video_conn.recv(&mut buff);
            if r.is_err() {
                tracing::warn!(method_name, "udp read error: {}", r.unwrap_err());
                continue;
            }

            let nread = r.unwrap();
            tracing::debug!(method_name, nread, "read video stream data");
            let video_packet = buff[2..nread].to_vec();

            // dump all video to file
            utils::append_to_file(&self.video_dump_file, &video_packet);
            let video_data_len = video_packet.len();
            let r = video_channel.send(video_packet);
            if r.is_err() {
                tracing::error!(
                    method_name,
                    "error sending video data: {}",
                    r.err().unwrap()
                );
            }
            tracing::debug!(method_name, video_data_len);
        }
    }

    fn joy(v: f32, min: i16, max: i16, smooth: bool) -> i16 {
        if smooth {
            let mut x = v * (max - min) as f32;
            x += (max + min) as f32;
            x *= 0.5;
            let mut y = x as i16;
            if y < min {
                y = min;
            }
            if y > max {
                y = max;
            }
            // -1 .... min
            // x ....  y = 0.5 * (min * (1.0 - x)  + max * (1.0 + x))
            // +1 .... max
            return y;
        }
        if v < -0.5 {
            min
        } else if v > 0.5 {
            max
        } else {
            (min + max) / 2
        }
    }

    pub(crate) fn send_update_sticks(&self) {
        let method_name = "update_sticks";
        loop {
            let start = Instant::now();
            let st = self.stick.read().unwrap();
            let rx = Self::joy(st.rx, RC_VAL_MIN, RC_VAL_MAX, true);
            let ry = Self::joy(st.ry, RC_VAL_MIN, RC_VAL_MAX, true);
            let lx = Self::joy(st.lx, RC_VAL_MIN, RC_VAL_MAX, true);
            let ly = Self::joy(st.ly, RC_VAL_MIN, RC_VAL_MAX, true);
            drop(st);

            let now = chrono::Local::now();
            let ms = now.timestamp_subsec_micros() & 0xffff;
            let g = self.flying.read().unwrap();
            let flying = *g;
            drop(g);
            if flying {
                tracing::debug!(method_name, rx, ry, lx, ly, "update drone movement");
                let msg = messages::send_stick_update(
                    rx,
                    ry,
                    lx,
                    ly,
                    false,
                    now.hour() as u8,
                    now.minute() as u8,
                    now.second() as u8,
                    ms as u16,
                );
                let r = self.ctrl_conn.send_to(&msg, &self.remote_addr);
                if r.is_err() {
                    tracing::warn!(method_name, "unable to ack log header: {}", r.unwrap_err());
                }
            }
            let now = Instant::now();
            let dur = now - start;
            let dur_ms = dur.as_millis();
            if dur_ms < 50 {
                let sleep_duration = 50 - dur_ms;
                // tracing::debug!(method_name, "update sticks duration={:?}", dur);
                thread::sleep(Duration::from_millis(
                    sleep_duration.try_into().expect("too big value to fit"),
                ));
            }
        }
    }
}

impl UpdateData {
    pub fn default() -> Self {
        UpdateData {
            flight: None,
            wifi: None,
            light: None,
            log: None,
        }
    }
    pub(crate) fn from_flight_data(flight: FlightData) -> Self {
        Self {
            flight: Some(flight),
            wifi: None,
            light: None,
            log: None,
        }
    }

    pub(crate) fn from_wifi_data(wifi: WifiData) -> Self {
        Self {
            flight: None,
            wifi: Some(wifi),
            light: None,
            log: None,
        }
    }

    pub(crate) fn from_light_data(light: LightData) -> Self {
        Self {
            flight: None,
            wifi: None,
            light: Some(light),
            log: None,
        }
    }

    pub(crate) fn from_log_data(log: LogData) -> Self {
        Self {
            flight: None,
            wifi: None,
            light: None,
            log: Some(log),
        }
    }
}
