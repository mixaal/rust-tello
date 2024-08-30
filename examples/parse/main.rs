use std::{
    env::args,
    fs::{self, File},
    io::{self, Read},
    path::PathBuf,
    time::SystemTime,
};

use rust_tello::messages::{self, FlightData, LightData, LogData, WifiData};

fn main() -> Result<(), io::Error> {
    tracing_subscriber::fmt().init();
    let dir = args().nth(1);
    let mut entries = fs::read_dir(dir.unwrap())?
        .map(|res| {
            res.map(|e| {
                (
                    e.path(),
                    e.metadata()
                        .unwrap()
                        .modified()
                        .unwrap()
                        .duration_since(SystemTime::UNIX_EPOCH)
                        .unwrap(),
                )
            })
        })
        .collect::<Result<Vec<_>, io::Error>>()?;
    entries.sort_by(|a, b| a.1.cmp(&b.1));
    // println!("entries={:?}", entries);
    for entry in entries {
        let path = entry.0;
        let packet_data = get_file_as_byte_vec(&path);
        let pkt = messages::TelloPacket::from_buffer(&packet_data);
        process_packet(pkt);
    }
    Ok(())
}

fn get_file_as_byte_vec(filename: &PathBuf) -> Vec<u8> {
    let mut f = File::open(filename).expect("no file found");
    let metadata = fs::metadata(&filename).expect("unable to read metadata");
    let mut buffer = vec![0; metadata.len() as usize];
    f.read(&mut buffer).expect("buffer overflow");

    buffer
}

fn process_packet(pkt: messages::TelloPacket) {
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
        }
        messages::MSG_FILE_DATA => {
            tracing::info!(method_name, "file data received");
        }
        messages::MSG_FLIGHT_STATUS => {
            tracing::info!(method_name, "flight status received");
            let flight_data = FlightData::new(&pkt.payload);
            tracing::info!(method_name, "flight_data: {:?}", flight_data);
        }
        messages::MSG_LIGHT_STRENGTH => {
            tracing::info!(method_name, "light strength received");
            let light_strength = LightData::new(&pkt.payload);
            tracing::info!(method_name, "light data: {:?}", light_strength);
        }
        messages::MSG_LOG_CONFIG => {
            tracing::info!(method_name, "log config received");
        }
        messages::MSG_LOG_HEADER => {
            tracing::info!(method_name, "log header received");
        }
        messages::MSG_LOG_DATA => {
            tracing::info!(method_name, "log data received");
            let log_data = LogData::new(&pkt.payload);
            tracing::info!("log_data={:?}", log_data);
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
        }
        _ => {
            let cmd = pkt.message_id;
            tracing::info!("Not yet supported: {:x}", cmd);
        }
    };
}
