use std::{
    fs::OpenOptions,
    io::Write,
    net::UdpSocket,
    process::{ChildStdin, Command, Stdio},
    time::{SystemTime, UNIX_EPOCH},
};

pub fn js_int16_to_tello(sv: i16) -> u64 {
    // sv is in range -32768 to 32767, we need 660 to 1388 where 0 => 1024
    //return uint64((sv / 90) + 1024)
    // Changed this as new info (Oct 18) suggests range should be 364 to 1684...
    //return ((sv as f64) / 49.672 + 1024.0) as u64;
    return sv as u64;
}

pub fn fatal(message: &str) -> ! {
    tracing::error!(message);
    std::process::exit(-1);
}

pub fn udp_sock(bind_addr: &str) -> UdpSocket {
    let sock = UdpSocket::bind(bind_addr);
    if sock.is_err() {
        let err_str = format!(
            "can't create udp socket for {bind_addr} : {}",
            sock.err().unwrap()
        );
        fatal(&err_str);
    }
    sock.unwrap()
}

pub fn contains_any(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

pub fn now_secs() -> u64 {
    let tm = SystemTime::now().duration_since(UNIX_EPOCH);
    if tm.is_err() {
        fatal("can't obtain time");
    }
    let tm = tm.unwrap();
    tm.as_secs()
}

pub fn now_msecs() -> u128 {
    let tm = SystemTime::now().duration_since(UNIX_EPOCH);
    if tm.is_err() {
        fatal("can't obtain time");
    }
    let tm = tm.unwrap();
    tm.as_millis()
}

pub fn udp_sock_clone(s: &UdpSocket) -> UdpSocket {
    let r = s.try_clone();
    if r.is_err() {
        fatal(&format!("can't clone socket: {}", r.unwrap_err()));
    }
    r.unwrap()
}

pub fn append_to_file(path: &str, buffer: Vec<u8>) {
    let mut file = OpenOptions::new()
        .create(true)
        .write(true)
        .append(true)
        .open(path)
        .unwrap();

    let r = file.write_all(&buffer);
    if r.is_err() {
        tracing::error!("error writing video to file: {}", r.unwrap_err());
    }
}

pub fn start_mplayer_with_stdin(use_x11: bool) -> Option<ChildStdin> {
    let method_name = "start_mplayer";
    let args = if use_x11 {
        vec!["-nosound", "-vo", "x11", "-fps", "60", "-"]
    } else {
        vec!["-nosound", "-fps", "60", "-"]
    };
    let mplayer = Command::new("mplayer")
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .spawn();
    if mplayer.is_err() {
        let e = mplayer.err().unwrap();
        tracing::warn!(method_name, "can't execute mplayer: {}", e);
        return None;
    }

    let stdin = mplayer.unwrap().stdin.take();
    if stdin.is_none() {
        tracing::warn!(method_name, "can't open mplayer stdin");
    }
    stdin
}

#[derive(Debug, PartialEq)]
pub(crate) struct Vec3<T> {
    x: T,
    y: T,
    z: T,
}

impl<T> Vec3<T> {
    pub(crate) fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

const ONE_DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

// QuatToEulerDeg converts a quaternion set into pitch, roll & yaw expressed in degrees
pub(crate) fn quat_to_euler_deg(q_x: f32, q_y: f32, q_z: f32, q_w: f32) -> (f64, f64, f64) {
    let qq_x = q_x as f64;
    let qq_y = q_y as f64;
    let qq_z = q_z as f64;
    let qq_w = q_w as f64;
    let sq_x = qq_x * qq_x;
    let sq_y = qq_y * qq_y;
    let sq_z = qq_z * qq_z;

    let sin_r = 2.0 * (qq_w * qq_x + qq_y * qq_z);
    let cos_r = 1.0 - 2.0 * (sq_x + sq_y);
    let roll = sin_r.atan2(cos_r) / ONE_DEG_TO_RAD;

    //roll = int(math.Round(math.Atan2(sinR, cosR) / degree))

    let mut sin_p = 2.0 * (qq_w * qq_y - qq_z * qq_x);
    if sin_p > 1.0 {
        sin_p = 1.0;
    }
    if sin_p < -1.0 {
        sin_p = -1.0;
    }
    let pitch = sin_p.asin() / ONE_DEG_TO_RAD;

    let sin_y = 2.0 * (qq_w * qq_z + qq_x * qq_y);
    let cos_y = 1.0 - 2.0 * (sq_y + sq_z);
    //yaw = int(math.Round(math.Atan2(sinY, cosY) / degree))
    let yaw = sin_y.atan2(cos_y) / ONE_DEG_TO_RAD;

    (pitch, roll, yaw)
}

fn _u16(lo: u8, hi: u8) -> u16 {
    (lo as u16) + ((hi as u16) << 8)
}

pub(crate) fn get_u16(buff: &Vec<u8>, lo_idx: usize, hi_idx: usize) -> u16 {
    _u16(buff[lo_idx], buff[hi_idx])
}

fn _i16(lo: u8, hi: u8) -> i16 {
    ((lo as u16) + ((hi as u16) << 8)) as i16
}

pub(crate) fn get_i16(buff: &Vec<u8>, lo_idx: usize, hi_idx: usize) -> i16 {
    _i16(buff[lo_idx], buff[hi_idx])
}

pub(crate) fn bytes_to_f32(buff: &Vec<u8>, index: usize) -> Result<f32, String> {
    if index + 3 >= buff.len() {
        return Err("buffer len not sufficient to contain f32 number".to_owned());
    }
    let b = [
        buff[index],
        buff[index + 1],
        buff[index + 2],
        buff[index + 3],
    ];
    tracing::debug!("bytes_f32= {:?}", b);
    Ok(f32::from_le_bytes(b))
}

pub(crate) fn decode_buffer(
    data: &Vec<u8>,
    xor_val: u8,
    position: usize,
    rec_len: usize,
) -> Vec<u8> {
    let mut i: usize = 0;
    let mut out = Vec::new();
    loop {
        if i >= rec_len || (position + i) >= data.len() {
            break;
        }
        out.push(data[position + i] ^ xor_val);
        i += 1;
    }
    out
}
