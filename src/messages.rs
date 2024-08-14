use std::{fs, path::PathBuf};

use crate::{
    crc::{calculate_crc16, calculate_crc8},
    env, utils,
};

// https://tellopilots.com/wiki/protocol/

pub(crate) const MSG_HDR: u8 = 0xcc;
const MIN_PKT_SZ: usize = 11;

// const PT_EXTENDED: u8 = 0;
const PT_GET: u8 = 1;
const PT_DATA1: u8 = 2;
const PT_DATA2: u8 = 4;
const PT_SET: u8 = 5;
const PT_FLIP: u8 = 6;

const BOUNCE_ON: u8 = 0x30;
const BOUNCE_OFF: u8 = 0x31;

#[repr(u8)]
enum FlipType {
    FlipForward = 0,
    FlipLeft = 1,
    FlipBackward = 2,
    FlipRight = 3,
    _FlipForwardLeft = 4,
    _FlipBackwardLeft = 5,
    _FlipBackwardRight = 6,
    _FlipForwardRight = 7,
}

#[repr(u8)]
pub enum SmartVideoCmd {
    Sv360 = 1 << 2,    // Slowly rotate around 360 degrees.
    SvCircle = 2 << 2, // Circle around a point in front of the drone.
    SvUpOut = 3 << 2,  // Perform the 'Up and Out' manouvre.
}

#[repr(u8)]
pub enum VBR {
    VbrAuto = 0, // let the Tello choose the best for the current connection
    Vbr1M,       // Set the VBR to 1Mbps
    Vbr1M5,      // Set the VBR to 1.5Mbps
    Vbr2M,       // Set the VBR to 2Mbps
    Vbr3M,       // Set the VBR to 3Mbps
    Vbr4M,       // Set the VBR to 4mbps
}

#[repr(u8)]
enum VideoMode {
    NORMAL,
    WIDE,
}

const logRecNewMVO: u16 = 0x001d;
const logRecIMU: u16 = 0x0800;

// Tello message IDs

const _MSG_DO_CONNECT: u16 = 0x0001; // 1
const _MSG_CONNECTED: u16 = 0x0002; // 2
pub(crate) const MSG_QUERY_SSID: u16 = 0x0011; // 17
const _MSG_SET_SSID: u16 = 0x0012; // 18
const _MSG_QUERY_SSIDPASS: u16 = 0x0013; // 19
const _MSG_SET_SSIDPASS: u16 = 0x0014; // 20
const _MSG_QUERY_WIFI_REGION: u16 = 0x0015; // 21
const _MSG_SET_WIFI_REGION: u16 = 0x0016; // 22
pub(crate) const MSG_WIFI_STRENGTH: u16 = 0x001a; // 26
const MSG_SET_VIDEO_BITRATE: u16 = 0x0020; // 32
const _MSG_SET_DYN_ADJ_RATE: u16 = 0x0021; // 33
const _MSG_EIS_SETTING: u16 = 0x0024; // 36
const MSG_QUERY_VIDEO_SPSPPS: u16 = 0x0025; // 37
pub(crate) const MSG_QUERY_VIDEO_BITRATE: u16 = 0x0028; // 40
pub(crate) const MSG_DO_TAKE_PIC: u16 = 0x0030; // 48
pub(crate) const MSG_SWITCH_PIC_VIDEO: u16 = 0x0031; // 49
const _MSG_DO_START_REC: u16 = 0x0032; // 50
const _MSG_EXPOSURE_VALS: u16 = 0x0034; // 52 (Get or set?)
pub(crate) const MSG_LIGHT_STRENGTH: u16 = 0x0035; // 53
const _MSG_QUERY_JPEGQUALITY: u16 = 0x0037; // 55
const _MSG_ERROR1: u16 = 0x0043; // 67
const _MSG_ERROR2: u16 = 0x0044; // 68
pub(crate) const MSG_QUERY_VERSION: u16 = 0x0045; // 69
pub(crate) const MSG_SET_DATE_TIME: u16 = 0x0046; // 70
const _MSG_QUERY_ACTIVATION_TIME: u16 = 0x0047; // 71
const _MSG_QUERY_LOADER_VERSION: u16 = 0x0049; // 73
const MSG_SET_STICK: u16 = 0x0050; // 80
pub(crate) const MSG_DO_TAKEOFF: u16 = 0x0054; // 84
pub(crate) const MSG_DO_LAND: u16 = 0x0055; // 85
pub(crate) const MSG_FLIGHT_STATUS: u16 = 0x0056; // 86
const _MSG_SET_HEIGHT_LIMIT: u16 = 0x0058; // 88
const MSG_DO_FLIP: u16 = 0x005c; // 92
const MSG_DO_THROW_TAKEOFF: u16 = 0x005d; // 93
const MSG_DO_PALM_LAND: u16 = 0x005e; // 94
pub(crate) const MSG_FILE_SIZE: u16 = 0x0062; // 98
pub(crate) const MSG_FILE_DATA: u16 = 0x0063; // 99
const MSG_FILE_DONE: u16 = 0x0064; // 100
const MSG_DO_SMART_VIDEO: u16 = 0x0080; // 128
pub(crate) const MSG_SMART_VIDEO_STATUS: u16 = 0x0081; // 129
pub(crate) const MSG_LOG_HEADER: u16 = 0x1050; // 4176
pub(crate) const MSG_LOG_DATA: u16 = 0x1051; // 4177
pub(crate) const MSG_LOG_CONFIG: u16 = 0x1052; // 4178
const MSG_DO_BOUNCE: u16 = 0x1053; // 4179
const _MSG_DO_CALIBRATION: u16 = 0x1054; // 4180
pub(crate) const MSG_SET_LOW_BATT_THRESH: u16 = 0x1055; // 4181
pub(crate) const MSG_QUERY_HEIGHT_LIMIT: u16 = 0x1056; // 4182
pub(crate) const MSG_QUERY_LOW_BATT_THRESH: u16 = 0x1057; // 4183
const _MSG_SET_ATTITUDE: u16 = 0x1058; // 4184
const MSG_QUERY_ATTITUDE: u16 = 0x1059; // 4185

#[derive(Debug)]
#[allow(dead_code)]
pub struct TelloPacket {
    header: u8,
    size13: u16,
    crc8: u8,
    from_drone: bool, // the following 4 fields are encoded in a single byte in the raw packet
    to_drone: bool,
    packet_type: u8,    // 3-bit
    packet_subtype: u8, // 3-bit
    pub message_id: u16,
    sequence: u16,
    pub payload: Vec<u8>,
    crc16: u16,
}

impl TelloPacket {
    pub fn new(packet_type: u8, cmd: u16, sequence: u16, param: Option<u8>) -> Self {
        let payload = if param.is_some() {
            vec![param.unwrap()]
        } else {
            Vec::new()
        };
        Self::new_with_payload(packet_type, cmd, sequence, payload)
    }

    fn new_with_payload(packet_type: u8, cmd: u16, sequence: u16, payload: Vec<u8>) -> Self {
        Self {
            header: MSG_HDR,
            size13: 0,
            crc8: 0,
            from_drone: false,
            to_drone: true,
            packet_type,
            packet_subtype: 0,
            message_id: cmd,
            sequence,
            payload,
            crc16: 0,
        }
    }

    pub fn ack_log(sequence: u16, id: &[u8; 2]) -> Self {
        let payload = vec![0, id[0], id[1]];
        Self::new_with_payload(PT_DATA1, MSG_LOG_HEADER, sequence, payload)
    }

    pub fn ack_file_piece(sequence: u16, done: bool, f_id: u16, piece_no: u32) -> Self {
        let payload = vec![
            done as u8,
            f_id as u8,
            (f_id >> 8) as u8,
            piece_no as u8,
            (piece_no >> 8) as u8,
            (piece_no >> 16) as u8,
            (piece_no >> 24) as u8,
        ];
        Self::new_with_payload(PT_DATA1, MSG_FILE_DATA, sequence, payload)
    }

    pub fn file_done(sequence: u16, f_id: u16, size: u32) -> Self {
        let payload = vec![
            f_id as u8,
            (f_id >> 8) as u8,
            size as u8,
            (size >> 8) as u8,
            (size >> 16) as u8,
            (size >> 24) as u8,
        ];
        Self::new_with_payload(PT_DATA1, MSG_FILE_DONE, sequence, payload)
    }

    #[must_use]
    pub fn to_buffer(&self) -> Vec<u8> {
        let payload_size = self.payload.len();
        let packet_size = MIN_PKT_SZ + payload_size;
        let mut buff = Vec::with_capacity(packet_size);
        // buff.reserve_exact(packet_size);
        unsafe {
            buff.set_len(packet_size);
        }

        buff[0] = self.header;
        buff[1] = (packet_size << 3) as u8;
        buff[2] = (packet_size >> 5) as u8;
        buff[3] = calculate_crc8(&buff[0..3]);
        buff[4] = self.packet_subtype + (self.packet_type << 3);
        if self.to_drone {
            buff[4] |= 0x40;
        }
        if self.from_drone {
            buff[4] |= 0x80;
        }
        buff[5] = self.message_id as u8;
        buff[6] = (self.message_id >> 8) as u8;
        buff[7] = self.sequence as u8;
        buff[8] = (self.sequence >> 8) as u8;

        for p in 0..payload_size {
            buff[9 + p] = self.payload[p];
        }
        let crc16 = calculate_crc16(&buff[0..9 + payload_size]);
        buff[9 + payload_size] = crc16 as u8;
        buff[10 + payload_size] = (crc16 >> 8) as u8;

        buff
    }

    pub fn from_buffer(buff: &[u8]) -> Self {
        let method_name = "from_buffer";
        let pkt_sz = ((buff[1] as u16 + ((buff[2] as u16) << 8)) as u16) >> 3;
        let pkt_sz = pkt_sz as usize;
        let payload_sz = pkt_sz - MIN_PKT_SZ;
        let crc16 = ((buff[pkt_sz - 1] as u16) << 8) + (buff[pkt_sz - 2] as u16);
        let crc8 = calculate_crc8(&buff[0..3]);
        let calc_crc16 = calculate_crc16(&buff[0..9 + payload_sz]);
        if calc_crc16 != crc16 {
            tracing::error!("mismatched crc16: {crc16} != {calc_crc16}");
        }
        if buff[3] != crc8 {
            tracing::error!("mismatched crc8: {crc8} != {}", buff[3]);
        }
        let mut payload = Vec::new();
        tracing::debug!(method_name, payload_sz, "create pkt from buffer");
        if payload_sz > 0 {
            payload.reserve_exact(payload_sz);
            unsafe {
                payload.set_len(payload_sz);
            }
            payload.clone_from_slice(&buff[9..9 + payload_sz]);
        }
        Self {
            header: buff[0],
            size13: pkt_sz as u16,
            crc8: buff[3],
            from_drone: (buff[4] & 0x80) == 1,
            to_drone: (buff[4] & 0x40) == 1,
            packet_type: ((buff[4] >> 3) & 0x07) as u8,
            packet_subtype: (buff[4] & 0x07) as u8,
            message_id: ((buff[6] as u16) << 8) | (buff[5] as u16),
            sequence: ((buff[8] as u16) << 8) | (buff[7] as u16),
            payload,
            crc16,
        }
    }
}

#[derive(Debug, PartialEq)]
pub(crate) enum FileType {
    FtJPEG,
    FtUnknown,
}

impl From<u8> for FileType {
    fn from(value: u8) -> Self {
        match value {
            0x1 => FileType::FtJPEG,
            _ => FileType::FtUnknown,
        }
    }
}

#[derive(Debug)]
pub(crate) struct FileInternal {
    pub f_id: u16,
    pub file_type: FileType,
    pub expected_size: u32,
    pub accum_size: u32,
    pub pieces: Vec<FilePiece>,
}

impl FileInternal {
    pub fn new(pl: &Vec<u8>) -> Self {
        let file_type = pl[0].into();

        let expected_size = (pl[1] as u32)
            + ((pl[2] as u32) << 8)
            + ((pl[3] as u32) << 16)
            + ((pl[4] as u32) << 24);
        let f_id = (pl[5] as u16) + ((pl[6] as u16) << 8);
        let pieces = Vec::with_capacity(1024);
        Self {
            f_id,
            file_type,
            expected_size,
            accum_size: 0,
            pieces,
        }
    }

    fn get_file_path(&mut self, save_dir: &str) -> PathBuf {
        let mut curr_id = self.f_id;
        loop {
            let f_name = format!("pic_{:04}.jpg", curr_id);
            let path = PathBuf::from(&save_dir).join(f_name);
            if !path.exists() {
                return path;
            }
            if curr_id >= 10000 {
                tracing::warn!("clean up picture storage - overwriting file: {:?}", path);
                return path;
            }

            curr_id += 1;
        }
    }
    pub fn save(&mut self) {
        let method_name = "save";
        let save_dir = env::ENV_TELLO_PICS_DIR.clone();
        tracing::info!(method_name, save_dir, "start");
        let r = fs::create_dir_all(&save_dir);
        if r.is_err() {
            tracing::warn!(
                method_name,
                save_dir,
                "can't create directory: {}",
                r.unwrap_err()
            );
            return;
        }
        let path = self.get_file_path(&save_dir);
        let mut buffer = Vec::new();
        // FIXME : !!!rewrite this to append to file, we will get rid mut all around here (including &mut self)!!!
        for pieces in self.pieces.iter_mut() {
            for ch in pieces.chunks.iter_mut() {
                if let Some(ref mut chunk) = ch {
                    buffer.append(&mut chunk.chunk_data);
                }
            }
        }
        if buffer.len() == 0 {
            tracing::error!("repeating save image occurs, but we need to ignore it");
            return;
        }
        let r = std::fs::write(&path, buffer);
        if r.is_err() {
            tracing::warn!(
                method_name,
                save_dir,
                "file {:?} not written: {}",
                path,
                r.unwrap_err()
            );
        }
    }
}

#[derive(Debug)]
pub(crate) struct FilePiece {
    pub num_chunks: i32,
    pub chunks: [Option<FileChunk>; 8],
}

const NO_CHUNK: Option<FileChunk> = None;
impl FilePiece {
    pub fn new() -> Self {
        Self {
            num_chunks: 0,
            chunks: [NO_CHUNK; 8],
        }
    }
}

#[derive(Debug)]
pub(crate) struct FileChunk {
    pub f_id: u16,
    pub piece_num: u32,
    pub chunk_num: u32,
    pub chunk_len: u16,
    pub chunk_data: Vec<u8>,
}

impl FileChunk {
    pub fn new(pl: &Vec<u8>) -> Self {
        if pl.len() <= 13 {
            utils::fatal("len must be at least 13 bytes");
        }
        Self {
            f_id: (pl[0] as u16) + (pl[1] as u16) << 8,
            piece_num: (pl[2] as u32)
                + ((pl[3] as u32) << 8)
                + ((pl[4] as u32) << 16)
                + ((pl[5] as u32) << 24),
            chunk_num: (pl[6] as u32)
                + ((pl[7] as u32) << 8)
                + ((pl[8] as u32) << 16)
                + ((pl[9] as u32) << 24),
            chunk_len: (pl[10] as u16) + ((pl[11] as u16) << 8),
            chunk_data: pl[12..].to_vec(),
        }
    }
}

#[derive(Debug)]
#[allow(dead_code)]
pub struct WifiData {
    wifi_interference: u8,
    wifi_strength: u8,
}

impl WifiData {
    pub fn new(pl: &Vec<u8>) -> Self {
        Self {
            wifi_strength: pl[0] as u8,
            wifi_interference: pl[1] as u8,
        }
    }
}

#[derive(Debug)]
#[allow(dead_code)]
pub struct LightData {
    light_strength: u8,
    light_strength_updated: u128,
}

impl LightData {
    pub fn new(pl: &Vec<u8>) -> Self {
        Self {
            light_strength: pl[0] as u8,
            light_strength_updated: utils::now_msecs(),
        }
    }
}

#[derive(Debug)]
pub(crate) struct IMUData {
    roll: f64,
    pitch: f64,
    yaw: f64,
    temperature: i16,
}

#[derive(Debug)]
pub(crate) struct MVOData {
    position: Option<utils::Vec3<f32>>,
    vx: Option<i16>,
    vy: Option<i16>,
    vz: Option<i16>,
}

#[derive(Debug)]
pub struct LogData {
    pub(crate) imu: Option<IMUData>,
    pub(crate) mvo: Option<MVOData>,
}

const logValidVelX: u8 = 0x01;
const logValidVelY: u8 = 0x02;
const logValidVelZ: u8 = 0x04;
const logValidPosY: u8 = 0x10;
const logValidPosX: u8 = 0x20;
const logValidPosZ: u8 = 0x40;

impl LogData {
    pub(crate) fn new(data: &Vec<u8>) -> Self {
        let mut pos = 1;
        if data.len() < 2 {
            return Self {
                imu: None,
                mvo: None,
            };
        }
        let mut imu = None;
        let mut mvo = None;
        while pos < data.len() - 6 {
            if data[pos] != 85 {
                //log.Println("Error parsing log record (bad separator)")
                tracing::warn!("Error parsing log record (bad separator)");
                break;
            }
            //let recLen = int((data[pos+1] as u8)) + int(uint8(data[pos+2] as u8))<<8;
            let recLen = utils::get_u16(data, pos + 1, pos + 2) as usize;
            //logRecType := uint16(data[pos+4]) + uint16(data[pos+5])<<8
            let logRecType = utils::get_u16(data, pos + 4, pos + 5);
            //log.Printf("Flight Log - Rec type: %x, len:%d\n", logRecType, recLen)
            tracing::info!("Flight Log - Rec type: {:x}, len:{}\n", logRecType, recLen);
            let xorVal = data[pos + 6];
            match logRecType {
                logRecNewMVO => {
                    tracing::info!("NewMOV rec found");
                    let mut i: usize = 0;
                    let xorBuf = utils::decode_buffer(data, xorVal, pos, recLen);

                    let mut offset = 10;
                    let flags = data[offset + 76];
                    tracing::debug!(flags, "mvo");

                    let vx = if flags & logValidVelX != 0 {
                        Some(utils::get_i16(&xorBuf, offset + 2, offset + 3))
                    //tello.fd.MVO.VelocityX = (int16(xorBuf[offset+2]) + int16(xorBuf[offset+3])<<8)
                    } else {
                        None
                    };
                    let vy = if flags & logValidVelY != 0 {
                        Some(utils::get_i16(&xorBuf, offset + 4, offset + 5))
                    //tello.fd.MVO.VelocityY = (int16(xorBuf[offset+4]) + int16(xorBuf[offset+5])<<8)
                    } else {
                        None
                    };
                    let vz = if flags & logValidVelZ != 0 {
                        //tello.fd.MVO.VelocityZ = -(int16(xorBuf[offset+6]) + int16(xorBuf[offset+7])<<8)
                        Some(-utils::get_i16(&xorBuf, offset + 6, offset + 7))
                    } else {
                        None
                    };

                    let position = if flags & logValidPosY != 0
                        && flags & logValidPosX != 0
                        && flags & logValidPosZ != 0
                    {
                        let py = utils::bytes_to_f32(&xorBuf, offset + 8)
                            .expect("py conversion went wrong");
                        let px = utils::bytes_to_f32(&xorBuf, offset + 12)
                            .expect("px conversion went wrong");
                        let pz = utils::bytes_to_f32(&xorBuf, offset + 16)
                            .expect("pz conversion went wrong");

                        Some(utils::Vec3::new(px, py, pz))
                    } else {
                        None
                    };

                    if vx != None || vy != None || vz != None || position != None {
                        mvo = Some(MVOData {
                            vx,
                            vy,
                            vz,
                            position,
                        });
                    }
                }
                logRecIMU => {
                    //log.Println("IMU rec found")
                    let xorBuf = utils::decode_buffer(data, xorVal, pos, recLen);
                    tracing::debug!(xorVal, "xor_buf={:?}", xorBuf);
                    let offset = 10;
                    let qw =
                        utils::bytes_to_f32(&xorBuf, offset + 48).expect("quater w decode failed");
                    let qx =
                        utils::bytes_to_f32(&xorBuf, offset + 52).expect("quater x decode failed");
                    let qy =
                        utils::bytes_to_f32(&xorBuf, offset + 56).expect("quater y decode failed");
                    let qz =
                        utils::bytes_to_f32(&xorBuf, offset + 60).expect("quater z decode failed");
                    tracing::debug!("qw={qw}, qx={qx}, qy={qy}, qz={qz}");
                    let temp = utils::get_u16(&xorBuf, offset + 106, offset + 107) / 100;
                    let (pitch, roll, yaw) = utils::quat_to_euler_deg(qx, qy, qz, qw);
                    imu = Some(IMUData {
                        roll,
                        pitch,
                        yaw,
                        temperature: temp as i16,
                    });
                }
                _ => {
                    tracing::info!(logRecType, "skipping unknown record");
                }
            }
            pos += recLen as usize;
        }

        Self { imu, mvo }
    }
}

// FlightData holds our current knowledge of the drone's state.
// This data is not all sent at once from the drone, different fields may be updated
// at varying rates.
#[derive(Debug)]
#[allow(dead_code)]
pub struct FlightData {
    battery_critical: bool,
    battery_low: bool,
    battery_milli_volts: i16,
    battery_percentage: i8,
    battery_state: bool,
    camera_state: u8,
    down_visual_state: bool,
    drone_fly_time_left: i16,
    drone_hover: bool,
    east_speed: i16,
    electrical_machinery_state: u8,
    em_open: bool,
    error_state: bool,
    factory_mode: bool,
    pub flying: bool,
    fly_mode: u8,
    fly_time: i16,
    front_in: bool,
    front_lsc: bool,
    front_out: bool,
    gravity_state: bool,
    height: i16, // seems to be in decimetres
    imu_calibration_state: i8,
    imu_state: bool,
    north_speed: i16,
    on_ground: bool,
    outage_recording: bool,
    power_state: bool,
    pressure_state: bool,
    throw_fly_timer: i8,
    vertical_speed: i16,
    wind_state: bool,
}

impl FlightData {
    pub fn new(pl: &Vec<u8>) -> Self {
        Self {
            height: (pl[0] as i16) + (pl[1] as i16) << 8,
            north_speed: ((pl[2] as u16) | (pl[3] as u16) << 8) as i16,
            east_speed: (pl[4] as i16) | (pl[5] as i16) << 8,
            vertical_speed: (pl[6] as i16) | (pl[7] as i16) << 8,
            fly_time: (pl[8] as i16) | (pl[9] as i16) << 8,

            imu_state: (pl[10] & 1) == 1,
            pressure_state: (pl[10] >> 1 & 1) == 1,
            down_visual_state: (pl[10] >> 2 & 1) == 1,
            power_state: (pl[10] >> 3 & 1) == 1,
            battery_state: (pl[10] >> 4 & 1) == 1,
            gravity_state: (pl[10] >> 5 & 1) == 1,
            // what is bit 6?
            wind_state: (pl[10] >> 7 & 1) == 1,

            imu_calibration_state: pl[11] as i8,
            battery_percentage: pl[12] as i8,
            drone_fly_time_left: (pl[13] as i16) + (pl[14] as i16) << 8,
            battery_milli_volts: (pl[15] as i16) + (pl[16] as i16) << 8,

            flying: (pl[17] & 1) == 1,
            on_ground: (pl[17] >> 1 & 1) == 1,
            em_open: (pl[17] >> 2 & 1) == 1,
            drone_hover: (pl[17] >> 3 & 1) == 1,
            outage_recording: (pl[17] >> 4 & 1) == 1,
            battery_low: (pl[17] >> 5 & 1) == 1,
            battery_critical: (pl[17] >> 6 & 1) == 1,
            factory_mode: (pl[17] >> 7 & 1) == 1,

            fly_mode: pl[18] as u8,
            throw_fly_timer: pl[19] as i8,
            camera_state: pl[20] as u8,
            electrical_machinery_state: pl[21] as u8,

            front_in: (pl[22] & 1) == 1,
            front_out: (pl[22] >> 1 & 1) == 1,
            front_lsc: (pl[22] >> 2 & 1) == 1,
            error_state: (pl[23] & 1) == 1,
        }
    }
}

#[must_use]
pub fn do_takeoff(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_TAKEOFF, seq, None).to_buffer()
}

#[must_use]
pub fn throw_takeoff(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_DO_THROW_TAKEOFF, seq, None).to_buffer()
}

#[must_use]
pub fn do_land(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_LAND, seq, Some(0)).to_buffer()
}

#[must_use]
pub fn cancel_land(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_LAND, seq, Some(1)).to_buffer()
}

#[must_use]
pub fn palm_land(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_PALM_LAND, seq, Some(0)).to_buffer()
}

#[must_use]
pub fn bounce_on(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_BOUNCE, seq, Some(BOUNCE_ON)).to_buffer()
}

#[must_use]
pub fn bounce_off(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_BOUNCE, seq, Some(BOUNCE_OFF)).to_buffer()
}

#[must_use]
pub fn flip_backward(seq: u16) -> Vec<u8> {
    TelloPacket::new(
        PT_FLIP,
        MSG_DO_FLIP,
        seq,
        Some(FlipType::FlipBackward as u8),
    )
    .to_buffer()
}

#[must_use]
pub fn flip_forward(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_FLIP, MSG_DO_FLIP, seq, Some(FlipType::FlipForward as u8)).to_buffer()
}

#[must_use]
pub fn flip_left(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_FLIP, MSG_DO_FLIP, seq, Some(FlipType::FlipLeft as u8)).to_buffer()
}

#[must_use]
pub fn flip_right(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_FLIP, MSG_DO_FLIP, seq, Some(FlipType::FlipRight as u8)).to_buffer()
}

#[must_use]
pub fn smart_video(seq: u16, cmd: SmartVideoCmd) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_SMART_VIDEO, seq, Some(cmd as u8)).to_buffer()
}

#[must_use]
pub fn query_attitude(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_QUERY_ATTITUDE, seq, None).to_buffer()
}

#[must_use]
pub fn query_low_battery_threshold(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_QUERY_LOW_BATT_THRESH, seq, None).to_buffer()
}

#[must_use]
pub fn query_height_limit(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_QUERY_HEIGHT_LIMIT, seq, None).to_buffer()
}

#[must_use]
pub fn query_ssid(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_QUERY_SSID, seq, None).to_buffer()
}

#[must_use]
pub fn query_version(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_QUERY_VERSION, seq, None).to_buffer()
}

#[must_use]
pub fn query_video_bitrate(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_GET, MSG_QUERY_VIDEO_BITRATE, seq, None).to_buffer()
}

#[must_use]
pub fn query_video_spsfps() -> Vec<u8> {
    TelloPacket::new(
        PT_DATA2,
        MSG_QUERY_VIDEO_SPSPPS,
        0, /* not ctrl seq here! */
        None,
    )
    .to_buffer()
}

// set low battery warning threshold, val is battery percentage
#[must_use]
pub fn set_low_battery_threshold(seq: u16, val: u8) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_SET_LOW_BATT_THRESH, seq, Some(val)).to_buffer()
}

#[must_use]
pub fn set_vbr(seq: u16, bitrate: VBR) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_SET_VIDEO_BITRATE, seq, Some(bitrate as u8)).to_buffer()
}

#[must_use]
pub fn set_video_normal(seq: u16) -> Vec<u8> {
    TelloPacket::new(
        PT_SET,
        MSG_SWITCH_PIC_VIDEO,
        seq,
        Some(VideoMode::NORMAL as u8),
    )
    .to_buffer()
}

#[must_use]
pub fn set_video_wide(seq: u16) -> Vec<u8> {
    TelloPacket::new(
        PT_SET,
        MSG_SWITCH_PIC_VIDEO,
        seq,
        Some(VideoMode::WIDE as u8),
    )
    .to_buffer()
}

#[must_use]
pub fn take_picture(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_SET, MSG_DO_TAKE_PIC, seq, None).to_buffer()
}

#[must_use]
pub fn ack_file_size(seq: u16) -> Vec<u8> {
    TelloPacket::new(PT_DATA1, MSG_FILE_SIZE, seq, Some(0)).to_buffer()
}

#[must_use]
pub fn ack_file_piece(seq: u16, done: bool, f_id: u16, piece_no: u32) -> Vec<u8> {
    TelloPacket::ack_file_piece(seq, done, f_id, piece_no).to_buffer()
}

#[must_use]
pub fn file_done(seq: u16, f_id: u16, size: u32) -> Vec<u8> {
    TelloPacket::file_done(seq, f_id, size).to_buffer()
}

#[must_use]
pub fn connect(video_port: u16) -> Vec<u8> {
    //con_req:lh
    vec![
        99,
        111,
        110,
        110,
        95,
        114,
        101,
        113,
        58,
        (video_port & 0xff) as u8, // l = low video port byte
        (video_port >> 8) as u8,   // h = high video port byte
    ]
}

#[must_use]
pub fn send_stick_update(
    rx: i16,
    ry: i16,
    lx: i16,
    ly: i16,
    sports_mode: bool,
    hour: u8,
    minute: u8,
    second: u8,
    millis: u16,
) -> Vec<u8> {
    let mut packed_axes = utils::js_int16_to_tello(rx) & 0x7ff;
    packed_axes |= (utils::js_int16_to_tello(ry) & 0x07ff) << 11;
    packed_axes |= (utils::js_int16_to_tello(ly) & 0x07ff) << 22;
    packed_axes |= (utils::js_int16_to_tello(lx) & 0x07ff) << 33;

    // let mut packedAxes =
    //     (rx as u64) | ((ry as u64) << 11) | ((ly as u64) << 22) | ((lx as u64) << 33);
    if sports_mode {
        packed_axes |= 1 << 44;
    }

    // println!("packedAxes={}", packedAxes);
    let payload = vec![
        packed_axes as u8,
        (packed_axes >> 8) as u8,
        (packed_axes >> 16) as u8,
        (packed_axes >> 24) as u8,
        (packed_axes >> 32) as u8,
        (packed_axes >> 40) as u8,
        hour,
        minute,
        second,
        (millis & 0xff) as u8,
        (millis >> 8) as u8,
    ];
    TelloPacket::new_with_payload(PT_DATA2, MSG_SET_STICK, 0, payload).to_buffer()
}

#[must_use]
pub fn send_date_time(
    seq: u16,
    year: u16,
    month: u16,
    day: u16,
    hour: u16,
    minute: u16,
    second: u16,
    ms: u16,
) -> Vec<u8> {
    let method_name = "send_date_time";
    tracing::debug!(
        method_name,
        year,
        month,
        day,
        hour,
        minute,
        second,
        ms,
        "forming set time packet"
    );
    let payload = vec![
        0,                   // 0 - reserved
        year as u8,          //1
        (year >> 8) as u8,   //2
        month as u8,         //3
        (month >> 8) as u8,  //4
        day as u8,           //5
        (day >> 8) as u8,    //6
        hour as u8,          //7
        (hour >> 8) as u8,   //8
        minute as u8,        //9
        (minute >> 8) as u8, //10
        second as u8,        //11
        (second >> 8) as u8, //12
        ms as u8,            //13
        (ms >> 8) as u8,     //14
    ];
    TelloPacket::new_with_payload(PT_DATA1, MSG_SET_DATE_TIME, seq, payload).to_buffer()
}

#[cfg(test)]
mod tests {
    use crate::{tello::Tello, UpdateData, UpdateDataPublishChannel};

    use super::*;
    use base64::prelude::*;

    #[test]
    fn test_do_takeoff() {
        let pkt = do_takeoff(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 104, 84, 0, 123, 0, 222, 157], pkt);
    }

    #[test]
    fn test_throw_takeoff() {
        let pkt = throw_takeoff(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 93, 0, 123, 0, 44, 4], pkt);
    }

    #[test]
    fn test_do_land() {
        let pkt = do_land(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 85, 0, 123, 0, 0, 0, 71], pkt);
    }

    #[test]
    fn test_cancel_land() {
        let pkt = cancel_land(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 85, 0, 123, 0, 1, 137, 86], pkt);
    }

    #[test]
    fn test_palm_land() {
        let pkt = palm_land(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 94, 0, 123, 0, 0, 236, 0], pkt);
    }

    #[test]
    fn test_bounce_on() {
        let pkt = bounce_on(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 83, 16, 123, 0, 48, 186, 142], pkt);
    }

    #[test]
    fn test_bounce_off() {
        let pkt = bounce_off(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 83, 16, 123, 0, 49, 51, 159], pkt);
    }

    #[test]
    fn test_flips() {
        let pkt = flip_backward(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 112, 92, 0, 123, 0, 2, 158, 86], pkt);

        let pkt = flip_forward(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 112, 92, 0, 123, 0, 0, 140, 117], pkt);

        let pkt = flip_right(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 112, 92, 0, 123, 0, 3, 23, 71], pkt);

        let pkt = flip_left(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 112, 92, 0, 123, 0, 1, 5, 100], pkt);
    }

    #[test]
    fn test_smart_video_commands() {
        let pkt = smart_video(123, SmartVideoCmd::Sv360);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 128, 0, 123, 0, 4, 71, 216], pkt);

        let pkt = smart_video(123, SmartVideoCmd::SvCircle);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 128, 0, 123, 0, 8, 43, 18], pkt);

        let pkt = smart_video(123, SmartVideoCmd::SvUpOut);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 128, 0, 123, 0, 12, 15, 84], pkt);
    }

    #[test]
    fn test_query_attitude() {
        let pkt = query_attitude(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 89, 16, 123, 0, 85, 243], pkt);
    }

    #[test]
    fn test_query_low_battery_threshold() {
        let pkt = query_low_battery_threshold(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 87, 16, 123, 0, 23, 93], pkt);
    }

    #[test]
    fn test_query_height_limit() {
        let pkt = query_height_limit(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 86, 16, 123, 0, 172, 65], pkt);
    }

    #[test]
    fn test_query_ssid() {
        let pkt = query_ssid(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 17, 0, 123, 0, 175, 133], pkt);
    }

    #[test]
    fn test_query_version() {
        let pkt = query_version(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 69, 0, 123, 0, 85, 34], pkt);
    }

    #[test]
    fn test_query_video_bitrate() {
        let pkt = query_video_bitrate(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 72, 40, 0, 123, 0, 62, 48], pkt);
    }

    #[test]
    fn test_query_video_spsfps() {
        let pkt = query_video_spsfps();
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 96, 37, 0, 0, 0, 108, 149], pkt);
    }

    #[test]
    fn test_set_low_battery_threshold() {
        let pkt = set_low_battery_threshold(123, 20);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 85, 16, 123, 0, 20, 4, 210], pkt);
    }

    #[test]
    fn test_set_vbr() {
        let pkt = set_vbr(123, VBR::Vbr4M);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 32, 0, 123, 0, 5, 10, 35], pkt);
    }

    #[test]
    fn test_set_video_normal() {
        let pkt = set_video_normal(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 49, 0, 123, 0, 0, 163, 203], pkt);
    }

    #[test]
    fn test_set_video_wide() {
        let pkt = set_video_wide(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 104, 49, 0, 123, 0, 1, 42, 218], pkt);
    }

    #[test]
    fn test_ack_log_header() {
        let id: &[u8; 2] = &[0x22, 0x33];
        let pkt = TelloPacket::ack_log(123, id).to_buffer();
        println!("{:?}", pkt);
        assert_eq!(
            vec![204, 112, 0, 203, 80, 80, 16, 123, 0, 0, 34, 51, 224, 179],
            pkt
        );
    }

    #[test]
    fn test_take_picture() {
        let pkt = take_picture(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 88, 0, 124, 104, 48, 0, 123, 0, 214, 118], pkt);
    }

    #[test]
    fn test_ack_file_size() {
        let pkt = ack_file_size(123);
        println!("{:?}", pkt);
        assert_eq!(vec![204, 96, 0, 39, 80, 98, 0, 123, 0, 0, 133, 69], pkt);
    }

    #[test]
    fn test_ack_file_piece() {
        let pkt = ack_file_piece(123, true, 0x1234, 0x56789abc);
        println!("{:?}", pkt);
        assert_eq!(
            vec![204, 144, 0, 190, 80, 99, 0, 123, 0, 1, 52, 18, 188, 154, 120, 86, 242, 179],
            pkt
        );
    }

    #[test]
    fn test_file_done() {
        let pkt = file_done(123, 0xabcd, 0x12345678);
        println!("{:?}", pkt);
        assert_eq!(
            vec![204, 136, 0, 36, 80, 100, 0, 123, 0, 205, 171, 120, 86, 52, 18, 194, 163],
            pkt
        );
    }

    #[test]
    fn test_connect() {
        let pkt = connect(8899);
        println!("{:?}", pkt);
        assert_eq!(vec![99, 111, 110, 110, 95, 114, 101, 113, 58, 195, 34], pkt);
    }

    #[test]
    fn test_stick_update() {
        // stick = (0,0)
        let pkt = send_stick_update(1024, 1024, 1024, 1024, false, 20, 20, 30, 3209);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 4, 32, 0, 1, 8, 20, 20, 30, 137, 12, 49, 146
            ],
            pkt
        );

        // forward (ry=max)
        let pkt = send_stick_update(1024, 1684, 1024, 1024, false, 20, 25, 4, 8614);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 164, 52, 0, 1, 8, 20, 25, 4, 166, 33, 127,
                123
            ],
            pkt
        );

        // backward (ry=min)
        let pkt = send_stick_update(1024, 1024 - 660, 1024, 1024, false, 21, 21, 18, 10803);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 100, 11, 0, 1, 8, 21, 21, 18, 51, 42, 125, 0
            ],
            pkt
        );

        //left (rx=min)
        let pkt = send_stick_update(1024 - 660, 1024, 1024, 1024, false, 21, 27, 31, 38430);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 108, 1, 32, 0, 1, 8, 21, 27, 31, 30, 150, 29, 79
            ],
            pkt
        );

        //right (rx=max)
        let pkt = send_stick_update(1024 + 660, 1024, 1024, 1024, false, 21, 29, 34, 59594);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 148, 6, 32, 0, 1, 8, 21, 29, 34, 202, 232, 122,
                107
            ],
            pkt
        );

        //up (ly=max)
        let pkt = send_stick_update(1024, 1024, 1024, 1024 + 660, false, 21, 32, 50, 55246);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 4, 32, 165, 1, 8, 21, 32, 50, 206, 215, 176,
                136
            ],
            pkt
        );

        //down (ly=min)
        let pkt = send_stick_update(1024, 1024, 1024, 1024 - 660, false, 21, 35, 28, 22354);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 4, 32, 91, 0, 8, 21, 35, 28, 82, 87, 235, 102
            ],
            pkt
        );

        //turn clock wise (lx=max)
        let pkt = send_stick_update(1024, 1024, 1024 + 660, 1024, false, 21, 38, 29, 33174);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 4, 32, 0, 41, 13, 21, 38, 29, 150, 129, 201,
                227
            ],
            pkt
        );

        //turn anti clock wise (lx=min)
        let pkt = send_stick_update(1024, 1024, 1024 - 660, 1024, false, 21, 41, 13, 43731);
        println!("{:?}", pkt);
        assert_eq!(
            vec![
                204, 176, 0, 127, 96, 80, 0, 0, 0, 0, 4, 32, 0, 217, 2, 21, 41, 13, 211, 170, 114,
                217
            ],
            pkt
        );
    }

    #[test]
    fn test_panic_on_buffer() {
        let subscriber = tracing_subscriber::fmt()
            .with_max_level(tracing::Level::TRACE)
            .finish();
        let r = tracing::subscriber::set_global_default(subscriber);
        let tello = Tello::new();
        for pkt_no in 0..=1542 {
            let frame = format!("dump_comm/ctrl_comm/dump_comm_1720968871/packet_{pkt_no}");
            if !PathBuf::from(&frame).exists() {
                println!("skip: {}", frame);
                continue;
            }
            // println!("{}", frame);
            let buff = fs::read(frame).unwrap();
            let (update_tx, update_rx) = crate::comm_channel();

            let pkt = TelloPacket::from_buffer(&buff);
            tello.process_packet(&pkt, &update_tx);
            println!("pkt={:?}", pkt);
        }
    }

    #[test]
    fn test_log_data() {
        let (update_tx, update_rx) = crate::comm_channel();

        test_packet_log(update_tx, "9Ow4iV9aNNJi74whCABFAAQQAKcAAP8RIeLAqAoBwKgKAiK50R8D/G1IzKAfT4hREFcBAFWEAM8ACIY8fACGhoaGhoaGhoaGhoaGhoaGPRSgwjAhXjrg4ds9ryvzOfNbCToQ9o+6fPLduoaGhgaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhorehICNhoaGhoaGoYYC2VVMAA0QCOw8fADs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7OzsbOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozk7Ozs+Ozs7OzsgjRVNADNoAgYPXwAGBgYGBgYGBgYGBgYGBgYGBgYGJgYGBgYGBgYGBgYGBgYGBgYDBgYGGMxVTgAgOgDNVd/AD81NTU1NTU1NTU1NTU1NTU1NTU1NTU1NTU1wTS9JjU1NTU1NTU1JDAvJDU1E9dVHAB66QNHV38AR0dHR0dHR0dHR0dHR0ebQvBeVYQAzwAIDdB/AA0NDQ0NDQ0NDQ0NDQ0NDQ0ngytJSZG1sTqFkLFR9HayX80usWWazjeZogYxDQ0NjQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ0NDQ30BlUPAQYNDQ0NDQ1CDW4MVUwADRAIc9B/AHNzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Pzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3Nzc3tzc3Fbc3Nzc3M57FU0AM2gCKDQfwCgoKCgoKCgoKCgoKCgoKCgoKCgIKCgoKCgoKCgoKCgoKCgoKCIoKCg4MZVEAA3ZieX/n8Al5eXlwvIVVwA4WUnxP5/AMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTExMTEnd9VhADPAAiSY4MAkpKSkpKSkpKSkpKSkpKSkgsbtNZVrC0u0YkNLuxn6S021roupEkXKCc2n66SkpISkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkpKSkmGZypCDmZKSkpKSkuWSsCRVTAANEAj5Y4MA+fn5+fn5+fn5+fn5+fn5+fn5+fn5+fn5+fn5+fn5+Xn5+fn5+fn5+fn5+fn5+fn5+fn5+fn58fn5+cX5+fn5+UmGVTQAzaAIJmSDACYmJiYmJiYmJiYmJiYmJiYmJiamJiYmJiYmJiYmJiYmJiYmJhomJianmFU4AIDoA0d+hgBTR0dHR0dHR0dHR0dHR0dHR0e5rA==");
        let update = update_rx.recv().expect("log data");
        assert!(update.log.is_some());
        let log_data = update.log.unwrap();
        assert!(log_data.imu.is_some());
        let imu = log_data.imu.unwrap();
        assert_eq!(30, imu.temperature);
        assert!(imu.pitch.abs() < f64::EPSILON);
        assert!(imu.roll.abs() < f64::EPSILON);
        assert!(imu.yaw.abs() < f64::EPSILON);
    }

    #[test]
    fn test_log_data2() {
        let packet_data = "9Ow4iV9aNNJi74whCABFAAHtHh8AAP8RBo3AqAoBwKgKAiK50R8B2WrwzIgOO4hREOAFAFUSAKauBjxbmAo8jGZ5PDzzFFUiAIsaBZdnmQoil3SXEWjXhyKXdJcRaNeHS4aXl3+UvABVMQAyGQUZeZkKG6XtAiOeTHklodTqpQm6sCWjMmAlfTdEojOHiSVbqOci7QegoxGWVYQAzwAICICZCggICAgICAgICAgICAgICAhamy5MvyIgM93bb7TlYXC3w8/fNFTfATTb3JYzLpMuTPP4dzfgKgE0v8EqM2AqqLQZcQG1uNe7swgKmDWFMxq1PocGNfJzBTRh2PYzjd+As6h++zMICAgICAhsHIwLMwYICAgICAhbecOlVUwADRAITICZCsF3XvF6w0JxtjdBcBhjIvFYqjbyatdqiAHCnncsl9ByTExMTExMTExMTExMTExMTExMTEyXTGRMGEz8dExMTExTt1U0AM2gCIGAmQrScvO+JTo/uhjTCzsa2yA//xOnxYUsbb15Uxq8gYGBgYGBgYExuYGBAZhVXADhHQAspZkKZina0zAsKSyGmpJsVjVqbhMKhGySG6oZLCwsLFisRm+SG6oZLCwsLJIbqhmSG6oZLCwsLPQbLRGSG6oZLGKMEZIbqhk3A7GSvlJXEVPVLCZFQZsA";
        let (update_tx, update_rx) = crate::comm_channel();
        test_packet_log(update_tx, &packet_data);
        let update = update_rx.recv().expect("log data");
        println!("{:?}", update);

        assert!(update.log.is_some());
        let log_data = update.log.unwrap();
        assert!(log_data.imu.is_some());
        assert!(log_data.mvo.is_some());
        let imu = log_data.imu.unwrap();
        let mvo = log_data.mvo.unwrap();
        assert_eq!(52, imu.temperature);
        assert!((imu.pitch - 0.303324619588394).abs() < f64::EPSILON);
        assert!((imu.roll - 0.953420753718995).abs() < f64::EPSILON);
        assert!((imu.yaw + 2.2377007352849043).abs() < f64::EPSILON);
        assert_eq!(-5, mvo.vz.unwrap());
        assert!(mvo.position.is_none());
        assert!(mvo.vx.is_none());
        assert!(mvo.vy.is_none());
    }

    #[test]
    fn test_log_data3() {
        let packet_data = "9Ow4iV9aNNJi74whCABFAALmHm0AAP8RBUbAqAoBwKgKAiK50R8C0mb8zFAWSohREOYFAFUSAKauBjOCnwozg2l2MzNCHlUiAIsaBXeOoAoZd4J3zog5Zxl3gnfOiDln5mZ3d590Zs1VMQAyGQVEoqAKRuQvtX6mw8d42trp+ENppXgAPdb4BVyX/9nUq3j0ivT/Ailp+EiTVYQAzwAIY6qgCmNjY2NjY2NjY2NjY2NjY2OJ9UUncieP2ANMjdgBABncaYdzXmlqyt+baVVZavhFJ/OTHFyl5GpfhtMLWHY8wt9VJnDd40/vXmPzLV/IsWje2iVnXr5gRF+sUZxYDHzr2O8NkFhjY2NjY2MMd2NjWG1jY2NjY2PAEixvVUwADRAIqKqgCgN6oxUR7qyVdauPlEkbyxV3TNAWoTOObFe+6ZSI2zOWqKioqKioqKioqKioqKioqKioqKhzqICo/KhwkKioqKhaqVU0AM2gCN6qoAoTLKzhK4gU5YTL2GfHg39gd0z4mvKxEuKWsFLi3t7e3t7e3t4G5t7exq9VXADhHQD0y6AKuvEBC+70/fRjPUq08+iytvTbXLRKw3LB9PT09J+on7dKw3LB9PT09ErDcsFKw3LB9PT09PklVslKw3LB9OjdyErDcsGW5GxKnrmHyYsN9P4+yFUkACGwBDzpoAo/PDwb6hp4PDw8PDw8PMx9PDw8PDw8PDyFrVUtAJOyBFPpoApSU1PPrFNTrKxSUlNTUlJSUlNTU1NTU1NTU1NTU0uvU1MWXVUTAGKzBF7poApcW16hoRJODmZVlQDnFAWG6aAKgoSHrOF5O6Bg3TiGhoaGhoaGhrwLCztOc2G6hoaGBoaGhoaAhoaGhoaGhoaGhYaGhISHdqM7hoaGhod2ozuMUSW9hobchnyGBYV8htyG3IbehAWF3IbchnyGfIYFhdyG3IaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoaGhoZ3eYaGhoaGDpUQegum";
        let (update_tx, update_rx) = crate::comm_channel();
        test_packet_log(update_tx, &packet_data);
        let update = update_rx.recv().expect("log data");
        println!("{:?}", update);

        assert!(update.log.is_some());
        let log_data = update.log.unwrap();
        assert!(log_data.imu.is_some());
        assert!(log_data.mvo.is_some());
        let imu = log_data.imu.unwrap();
        let mvo = log_data.mvo.unwrap();
        assert_eq!(52, imu.temperature);
        assert!((imu.pitch - 0.42572309438825817).abs() < f64::EPSILON);
        assert!((imu.roll - 0.9537320659957234).abs() < f64::EPSILON);
        assert!((imu.yaw + 2.253996543499859).abs() < f64::EPSILON);
        assert_eq!(-11, mvo.vx.unwrap());
        assert_eq!(26, mvo.vy.unwrap());
        assert_eq!(-9, mvo.vz.unwrap());
        assert!(mvo.position.is_none());
    }

    #[test]
    fn test_log_data4() {
        let packet_data = "9Ow4iV9aNNJi74whCABFAAHtHrsAAP8RBfHAqAoBwKgKAiK50R8B2au4zIgOO4hREOwFAFUSAKauBj2ppgo9jWd4PT3rFVUiAIsaBY21pwoWjUCNb3L+nRaNQI1vcv6dHZyNjWWO2GBVMQAyGQVmx6cKZModcFuyr/FaL+t72uLz+tq/oxdaQvafXMCvZdvKJ81aCbvJ3Q53VYQAzwAIdM6nCnR0dHR0dHR0dHR0dHR0dHRx5FIw/YdmyMJ1F8goUQ/LoQ9LyFzTvU8e6HBIqu5SMAeEC0tezYBPJxb+T0Vx0MiCdEzKBD+Cz3TgKMgeMnvJTYV2SdQVLEgA+YtPCA3zz2Uch090dHR0dHQfYPB3S3p0dHR0dHSHBYPGVUwADRAIt86nCt3xuAqORrWKF9bvi8Nh7AqmD8EJaS2RcygzJotqfC+Jt7e3t7e3t7e3t7e3t7e3t7e3t7dst5+347e3jre3t7dqulU0AM2gCOvOpwo8B5nUtfhf0Ijh0tFiaUpVIHnNr4R0BNd7L+VX6+vr6+vr6+vr0uvrmPVVXADhHQD58qcKq/wPBuP59/nwI0e5aOe/u8S6UblHzn/M+fn5+Xj2l7pHzn/M+fn5+UfOf8xHzn/M+fn5+abNa8RHzn/M+b3vRUfOf8zSfm9HvCGJxIYA+fOiF43E";
        let (update_tx, update_rx) = crate::comm_channel();
        test_packet_log(update_tx, &packet_data);
        let update = update_rx.recv().expect("log data");
        println!("{:?}", update);

        assert!(update.log.is_some());
        let log_data = update.log.unwrap();
        assert!(log_data.imu.is_some());
        assert!(log_data.mvo.is_some());
        let imu = log_data.imu.unwrap();
        let mvo = log_data.mvo.unwrap();
        assert_eq!(52, imu.temperature);
        assert!((imu.roll - 0.8459818959213642).abs() < f64::EPSILON);
        assert!((imu.pitch - 0.5009636027037578).abs() < f64::EPSILON);
        assert!((imu.yaw + 2.2908922271035723).abs() < f64::EPSILON);
        assert_eq!(-10, mvo.vx.unwrap());
        assert_eq!(26, mvo.vy.unwrap());
        assert_eq!(-14, mvo.vz.unwrap());
        assert!(mvo.position.is_none());
    }

    #[test]
    fn test_log_data5() {
        let packet_data = "9Ow4iV9aNNJi74whCABFAAMyJVcAAP8R/g/AqAoBwKgKAiK50R8DHkhQzLAYIIhREO4HAFUSAKauBj55/ww+bmd7Pj4lu1UiAIsaBTCFAA3pyyjP1uqKJenLKM94xIolNxUwMNgzRwRVMQAyGQXGlwANxOoDdfmTdBP5HckPeftv2Pl+nVb4Cj+gefm8PfjIgj/6wyiLeTz5VYQAzwAItJ4ADbS0tLS0tLS0tLS0tLS0tLTtzZLwZ+z7CshnUoks+cULc2aoi0sOyYoaOtELbDCS8G2jwYt1uncIJejkCfu8JYoIojcLqKtYCrQI64i0tLS0tLS0tHdMAoqjxU2PQhAGDqJFWo+0tLS0tLQ+orS0V7q0tLS0tLSHONNAVUwADRAI954ADff39/f39/f3NA9BySmuvzdw169IL3PRM9iJ+8nF47vK9/f39/f39/f39/f39/f39/f39/c399/3o/fXsff39/cWXVU0AM2gCC6fAA0ic1ER7JIDkx4db5MNpNWSwbwIarWOSxCOXi2TLi4uLi4uLi4OaC4ulJ9VXADhHQCzwwANEbW9s09MtbO5u0KIV+Mqj8lmJwgNhDWGs7Ozs7OzM4wNhDWGs7Ozsw2ENYYNhDWGs7Ozs6UuCg0NhDWGs27ijg2ENYZ+f38OuGQQj7uzs7lX5VUkACGwBO/fAA3s7+/IOcmr7+/v7+/v7x+u7+/v7+/v7+8oo1UtAJOyBAXgAA0EBQWZ+gUF2PoEBAUFBAQEBAUFBQUFBQUFBQUFBR35BQXUMlUTAGKzBBDgAA0SgBDv78MFz5VVlQDnFAUy4AANNzIyGFXNjxTUaYwyMjIyMjIyMnJnMA1muPIPMjIysjIyMjI2MDIyMjIyMjIyMTIyMDAzwhePMjIyMjPCF4845ZEJMjJoMsgysTHIMmgyaDJqMLExaDJoMt4yyDKxMWgyaDIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjLDzTIyMjIyuiFuMFVMAA0XBWTgAA1kZABkZGRkZGRkAGRkZGRkZGQAZGRkZGRkZABkZGRkZGRkMGRRZPmbZGSvmzBk+ZtkZMibr5v5m2RkUWTIm/mb2ZgEXQ==";
        let (update_tx, update_rx) = crate::comm_channel();
        test_packet_log(update_tx, &packet_data);
        let update = update_rx.recv().expect("log data");
        println!("{:?}", update);

        assert!(update.log.is_some());
        let log_data = update.log.unwrap();
        assert!(log_data.imu.is_some());
        assert!(log_data.mvo.is_none());
        let imu = log_data.imu.unwrap();

        assert_eq!(57, imu.temperature);
        assert!((imu.roll + 4.282562497871982).abs() < f64::EPSILON);
        assert!((imu.pitch + 4.813630495148491).abs() < f64::EPSILON);
        assert!((imu.yaw - 33.14411926958298).abs() < f64::EPSILON);
    }

    fn test_packet_log(update_tx: UpdateDataPublishChannel, packet_data: &str) {
        // let tello = Tello::new();

        let bytes_buffer = BASE64_STANDARD.decode(packet_data.as_bytes()).unwrap()[42..].to_vec();

        for bb in bytes_buffer.iter() {
            print!("{:x} ", bb);
        }

        let pkt = TelloPacket::from_buffer(&bytes_buffer);
        let subscriber = tracing_subscriber::fmt()
            .with_max_level(tracing::Level::TRACE)
            .finish();
        let r = tracing::subscriber::set_global_default(subscriber);

        // tello.process_packet(&pkt, &update_tx);
        let log_data = LogData::new(&pkt.payload);
        let _ = update_tx.send(UpdateData::from_log_data(log_data));
        // println!("pkt={:?}", pkt);
    }
}
