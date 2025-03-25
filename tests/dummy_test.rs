#![cfg(feature = "dummy_test")]
#![cfg_attr(not(feature = "use_std"), no_std)] // if the feature "use_std" is not enabled, definitely turn off the entire std environment using the compiler directive "no_std"

use rustflight_alpha::board::dummy::DummyBoard;
use rustflight_alpha::rustflight::ROSFlight;

#[test]
fn main_test() {
    let mut firmware = ROSFlight::init(0, DummyBoard {});

    // Because this is a test, we don't want this to loop forever.
    // In the actual firmware, this would be a loop{ ... } instead.
    for _i in 0..10 {
        firmware.run();
    }

    assert_eq!(true, true);
}