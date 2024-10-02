pub mod board;
pub mod params;
pub mod rustflight;
mod sensors;
mod state_machine;
pub(crate) mod units;
// TODO: Change tests below to actual tests.
//
// pub fn add(left: u64, right: u64) -> u64 {
//     left + right
// }
//
// #[cfg(test)]
// mod tests {
//     use super::*;
//
//     #[test]
//     fn it_works() {
//         let result = add(2, 2);
//         assert_eq!(result, 4);
//     }
// }
