#![cfg(feature = "default")]
#![cfg_attr(not(feature = "use_std"), no_std)] // if the feature "use_std" is not enabled, definitely turn off the entire std environment using the compiler directive "no_std"


use rustflight_alpha::{board::dummy::DummyBoard, rustflight::ROSFlight, comm_manager, params, sensors};

#[test]
fn main_test() {
    let mut firmware = ROSFlight::init(1, DummyBoard {});

    // Because this is a test, we don't want this to loop forever.
    // In the actual firmware, this would be a loop{ ... } instead.

    // High level: Create the variables we'll need
    let mut p = params::Params::new();
    let mut mavlink = crate::comm_manager::mavlink::Mavlink::new();
    let mut comm_manager = crate::comm_manager::CommManager::new(mavlink);
    let mut sensors = sensors::Sensors::new();

    // highest level loop:
    for i in 0..100 {
        sensors.run(&firmware.board);
    }

    assert_eq!(true, true);
}