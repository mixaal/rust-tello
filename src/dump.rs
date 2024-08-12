use std::{fs, path::PathBuf, sync::atomic::AtomicU64};

use crate::{env, utils};

#[derive(Debug, Clone)]
pub struct ConnDumper {
    dir: PathBuf,
    packet_no: &'static AtomicU64,
}

impl ConnDumper {
    pub fn new(name: &str, packet_counter: &'static AtomicU64) -> Self {
        let mut dir = env::ENV_TELLO_DUMP_DIR.clone();
        if !dir.ends_with("/") {
            dir = dir + "/";
        }
        dir = dir + name;
        if !name.ends_with("/") {
            dir = dir + "/";
        }
        dir = dir + &format!("dump_comm_{}", utils::now_secs());
        let r = fs::create_dir_all(&dir);
        if r.is_err() {
            utils::fatal(&format!("can't create dir: {}", r.unwrap_err()));
        }
        let dir = PathBuf::from(dir);
        Self {
            dir,
            packet_no: packet_counter,
        }
    }

    pub fn dump(&self, buff: &[u8]) {
        let packet_no = self
            .packet_no
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let f_name = format!("packet_{packet_no}");
        let file_path = self.dir.join(f_name);
        let r = std::fs::write(file_path, buff);
        if r.is_err() {
            tracing::warn!("can't dump the packet contents: {}", r.unwrap_err());
        }
    }
}
