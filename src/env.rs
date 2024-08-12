use std::str::FromStr;

lazy_static! {
    pub static ref ENV_TELLO_ADDR: String =
        get_env_str("ENV_TELLO_ADDR", String::from("192.168.10.1"));
    pub static ref ENV_TELLO_CTRL_PORT: u16 = get_env("ENV_TELLO_CTRL_PORT", 8889);
    pub static ref ENV_TELLO_LOCAL_PORT: u16 = get_env("ENV_TELLO_LOCAL_PORT", 8800);
    pub static ref ENV_TELLO_VIDEO_PORT: u16 = get_env("ENV_TELLO_VIDEO_PORT", 6038);
    pub static ref ENV_TELLO_DUMP_DIR: String =
        get_env_str("ENV_TELLO_DUMP_DIR", "./dump_comm/".to_owned());
    pub static ref ENV_TELLO_PICS_DIR: String =
        get_env_str("ENV_TELLO_PICS_DIR", "./save_pics/".to_owned());
}

pub fn get_env_str(name: &str, value: String) -> String {
    return std::env::var(name).unwrap_or(value);
}

pub fn get_env<T: FromStr>(name: &str, value: T) -> T {
    let r = std::env::var(name);
    if r.is_err() {
        return value;
    }
    let r = r.unwrap().parse::<T>();
    if let Ok(res) = r {
        res
    } else {
        value
    }
}
