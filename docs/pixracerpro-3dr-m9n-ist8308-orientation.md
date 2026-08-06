# Pixracer Pro with 3DR NEO-M9N/IST8308 magnetometer orientation

This guide documents the expected magnetometer orientation when a 3DR GPS
u-blox NEO-M9N/IST8308 module is connected to a Pixracer Pro running Veloxity or
the compatible ROSflight C firmware.

It applies to the following physical installation:

- the GNSS antenna faces upward;
- the gold arrow on the top of the module points toward the aircraft nose;
- the module is mounted level with the aircraft; and
- the IST8308 is connected through the Pixracer Pro external I2C interface.

The orientation must still be confirmed from live measurements before flight.
Product revisions, alternate mounting arrangements, or a differently oriented
module require different orientation parameters.

## The three coordinate frames

There are three distinct frames involved:

1. The IST8308 sensor's native coordinate frame.
2. The coordinate convention produced by the low-level IST8308 driver.
3. The aircraft body frame expected by ROSflight and Veloxity: X forward, Y
   right, and Z down.

The coordinate graphic is printed on the underside of the 3DR module. When the
module is viewed from below with its gold arrow still pointing forward, printed
left corresponds to aircraft-right when viewed normally from above. The
underside graphic therefore indicates the following relationship for this
mounting:

| Frame | Positive X | Positive Y | Positive Z |
| --- | --- | --- | --- |
| IST8308 native frame | Aft | Right | Down |
| After the driver's Z inversion | Aft | Right | Up |
| Required aircraft body frame | Forward | Right | Down |

The driver's Z inversion and the module mounting rotation solve different
problems:

- The low-level driver converts `[X, Y, Z]` to `[X, Y, -Z]`. This matches the
  ROSflight C IST8308 driver and corrects the sensor coordinate convention to a
  right-handed frame.
- The antenna-up, arrow-forward module is then related to the aircraft frame by
  a 180-degree pitch rotation. That rotation converts `[X, Y, -Z]` to
  `[-X, Y, Z]`.

The complete pre-calibration transformation is therefore:

```text
IST8308 register values:       [ X,  Y,  Z]
After low-level driver:        [ X,  Y, -Z]
After MAG_PITCH=180 degrees:   [-X,  Y,  Z]
```

The final Z value happens to have the same numerical sign as the original raw
sensor Z because two separate Z inversions occur. The first is the driver's
sensor-convention correction; the second is part of the sensor-to-aircraft
180-degree pitch rotation.

In this document, **corrected Z** means the final Z value after the driver
conversion, mounting rotation, and magnetometer calibration. It is the value
published on `/magnetometer`, not the unprocessed IST8308 register value.

## Expected orientation parameters

For the antenna-up, gold-arrow-forward installation described above, use:

```text
MAG_ROLL  = 0
MAG_PITCH = 180
MAG_YAW   = 0
```

Do not replace the low-level driver's Z inversion with this rotation. The
driver conversion preserves ROSflight C compatibility, while the `MAG_*`
parameters describe how the external module is physically mounted.

Veloxity applies the mounting rotation before the hard-iron bias and soft-iron
matrix. Set and verify the orientation before calibrating. Changing the
orientation afterward invalidates the existing magnetometer calibration.

## Set and save the orientation

With `rosflight_io` connected to the firmware, set the parameters:

```bash
ros2 service call /param_set rosflight_msgs/srv/ParamSet \
  "{name: MAG_ROLL, value: 0.0}"

ros2 service call /param_set rosflight_msgs/srv/ParamSet \
  "{name: MAG_PITCH, value: 180.0}"

ros2 service call /param_set rosflight_msgs/srv/ParamSet \
  "{name: MAG_YAW, value: 0.0}"
```

Save the parameters to persistent firmware storage:

```bash
ros2 service call /param_write std_srvs/srv/Trigger
```

Read the parameters back after a reboot before relying on them.

## Perform the magnetometer calibration

Perform the calibration outdoors and away from vehicles, steel tables,
reinforced concrete, loudspeakers, phones, tools, and high-current wiring. The
vehicle should contain the same wiring and equipment it will carry in flight.

Start the ROSflight magnetometer calibration:

```bash
ros2 service call /calibrate_mag std_srvs/srv/Trigger
```

Follow the `rosflight_io` feedback. Its calibration procedure identifies all six
vehicle faces using the accelerometer and requests rotations that cover a full
circle for each orientation. Continue until it reports that all orientations
are covered and calibration is complete.

Calibration fits hard-iron offsets and a soft-iron correction matrix. A
successful fit does not, by itself, prove that the magnetometer axes match the
aircraft axes. Perform the following orientation tests after calibration.

## Inspect the corrected magnetic vector

Display the final, rotated, and calibrated aircraft-frame measurement:

```bash
ros2 topic echo /magnetometer
```

The `magnetic_field.x`, `.y`, and `.z` fields are the values to test. They are
published in tesla.

## Four-cardinal orientation test

Place the aircraft level with the GNSS antenna upward. Establish magnetic north,
then move any handheld compass or phone several feet away from the
magnetometer. Point the module's gold arrow in each cardinal direction and
allow the readings to settle.

| Gold arrow points toward | Expected dominant horizontal result |
| --- | --- |
| Magnetic north | X positive |
| Magnetic east | Y negative |
| Magnetic south | X negative |
| Magnetic west | Y positive |

The other horizontal component should be comparatively small near each
cardinal direction. Local magnetic declination changes the relationship between
magnetic and true north, so use magnetic north for this test.

For a level aircraft, estimate magnetic heading from the corrected vector with:

```text
heading_degrees = degrees(atan2(-Y, X))
```

Add 360 degrees if the result is negative. The expected results are
approximately:

```text
North:   0 degrees
East:   90 degrees
South: 180 degrees
West:  270 degrees
```

Check all four directions. An incorrect 180-degree yaw correction can make one
direction appear plausible while reversing the response in other directions.

## Tilt test

Point the gold arrow toward magnetic north and establish a stable heading. Keep
the vehicle in approximately the same yaw direction while separately:

- pitching the nose up and down by roughly 30 to 45 degrees; and
- rolling the vehicle left and right by roughly 30 to 45 degrees.

The estimator's tilt-compensated magnetic heading should remain reasonably
stable. A large heading change, especially one approaching 90 or 180 degrees,
suggests an incorrect axis mapping, incorrect Z sign, or poor calibration.

## Field-magnitude test

While slowly rotating the vehicle through several orientations, calculate:

```text
field_magnitude = sqrt(X*X + Y*Y + Z*Z)
```

The magnitude should remain approximately constant. A variation within roughly
10 to 15 percent is a useful initial target, although the achievable result
depends on the installation and local environment. Large orientation-dependent
changes indicate residual hard-iron or soft-iron error, nearby magnetic
material, or an inadequate calibration.

The Earth's field is commonly on the order of 25 to 65 microtesla, or
`2.5e-5` to `6.5e-5` tesla. Treat that only as a broad sanity range; local field
strength varies geographically.

## Interpreting corrected Z

When the aircraft is level, the corrected Z sign depends on magnetic
inclination. Across much of the Northern Hemisphere, the Earth's magnetic field
points downward, so aircraft-frame Z is generally positive. Across much of the
Southern Hemisphere it is generally negative. Near the magnetic equator the Z
component may be small.

Do not use corrected Z sign alone to decide whether the module orientation is
correct. The four-cardinal and tilt tests are stronger checks.

## Motor and electrical-interference test

After orientation and calibration pass, remove all propellers and secure the
vehicle. Compare magnetic heading and field magnitude with the propulsion
system unpowered and while stepping through representative motor commands.

A significant heading or magnitude change with motor current indicates magnetic
interference from motors, ESCs, battery leads, power distribution, fasteners, or
other vehicle hardware. That is an installation problem rather than an axis
orientation problem. Increase separation, reroute paired high-current wiring,
and recalibrate after changing the installation.

## Relevant implementation

- The Veloxity IST8308 driver performs the C-compatible Z inversion in
  `platforms/stm_32/src/peripherals/ist8308.rs`.
- Veloxity applies `MAG_ROLL`, `MAG_PITCH`, and `MAG_YAW` before hard-iron and
  soft-iron calibration in
  `crates/veloxity_core/src/sensors/processors.rs`.
- The ROSflight C Pixracer Pro IST8308 driver likewise negates Z and describes
  it as the right-hand-rule adjustment.

Hardware references:

- [3DR GPS u-blox NEO-M9N - IST8308 product page](https://store.3dr.com/gps-u-blox-neo-m9n-ist8308/)
- [3DR GPS Classic M9N - IST8308 documentation](https://docs.3dr.com/gps/classic-m9n-ist8308/)
- [iSentek IST8308 product information](https://www.isentek.com/service_view.php/products/jG_ai35A7l)
