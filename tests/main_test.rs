use rustflight_alpha::rustflight::ROSFlight;
use rustflight_alpha::board::DummyBoard;

#[test]
fn main_test() {

    let firmware = ROSFlight::init(0, Box::new(DummyBoard{}));

    // Because this is a test, we don't want this to loop forever.
    // In the actual firmware, this would be a loop{ ... } instead.
    for _i in 0..10 {
        firmware.run();
    }

    assert_eq!(true, true);
}
