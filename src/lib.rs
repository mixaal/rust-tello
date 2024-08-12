use std::sync::mpsc::{Receiver, Sender};

pub(crate) mod crc;
pub(crate) mod dump;
pub(crate) mod env;
pub(crate) mod messages;
pub mod tello;
pub(crate) mod utils;

#[macro_use]
extern crate lazy_static;

pub type VideoRecvChannel = Receiver<Vec<u8>>;
pub type VideoPublishChannel = Sender<Vec<u8>>;
pub type VideoChannel = (VideoPublishChannel, VideoRecvChannel);

// pub fn add(left: usize, right: usize) -> usize {
//     left + right
// }

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }
