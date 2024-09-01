use rhoflight::rosflight::ROSFlight;

#[test]
fn main_test() {

    let firmware = ROSFlight::init(0);

    for _i in 0..10 {
        firmware.run();
    }

    assert_eq!(true, true);
}