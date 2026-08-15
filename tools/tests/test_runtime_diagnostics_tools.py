import importlib.util
import pathlib
import unittest


def load_capture_module():
    path = pathlib.Path(__file__).parents[1] / "capture_runtime_diagnostics.py"
    spec = importlib.util.spec_from_file_location("capture_runtime_diagnostics", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


capture = load_capture_module()


class RuntimeDiagnosticsCaptureTests(unittest.TestCase):
    def test_counter_delta_wraps_at_u32(self):
        self.assertEqual(
            capture.delta({"counter": 0xFFFF_FFFE}, {"counter": 3})["counter"],
            5,
        )

    def test_observed_uses_absolute_max_and_delta_for_accumulator(self):
        deltas = {
            "VELOXITY_DIAG_THING_COUNT": 7,
            "VELOXITY_DIAG_THING_MAX_US": 2,
            "VELOXITY_DIAG_THING_DEPTH_MAX": 1,
        }
        after = {
            "VELOXITY_DIAG_THING_COUNT": 20,
            "VELOXITY_DIAG_THING_MAX_US": 42,
            "VELOXITY_DIAG_THING_DEPTH_MAX": 6,
        }
        values = capture.observed(deltas, after)
        self.assertEqual(values["VELOXITY_DIAG_THING_COUNT"], 7)
        self.assertEqual(values["VELOXITY_DIAG_THING_MAX_US"], 42)
        self.assertEqual(values["VELOXITY_DIAG_THING_DEPTH_MAX"], 6)

    def test_sensor_rows_derives_rates_and_pipeline_fields(self):
        values = {
            "VELOXITY_DIAG_MAG_PUBLISH": 100,
            "VELOXITY_DIAG_MAG_ERROR_PUBLISH": 2,
            "VELOXITY_DIAG_MAG_SIGNAL_OVERWRITE": 3,
            "VELOXITY_DIAG_MAG_CONSUME": 97,
            "VELOXITY_DIAG_MAG_CONSUME_AGE_SUM_US": 970,
            "VELOXITY_DIAG_MAG_CONSUME_AGE_MAX_US": 20,
            "VELOXITY_DIAG_MAG_PROCESS_INPUT": 97,
            "VELOXITY_DIAG_MAG_PROCESS_OUTPUT": 95,
            "VELOXITY_DIAG_MAG_UNSENT_OVERWRITE": 1,
            "VELOXITY_DIAG_TELEM_MAG_SENT": 94,
            "VELOXITY_DIAG_MAG_CONVERSION_COMMAND": 100,
            "VELOXITY_DIAG_MAG_DRDY_READY": 98,
            "VELOXITY_DIAG_MAG_DRDY_MISS": 2,
            "VELOXITY_DIAG_MAG_I2C_ERROR": 0,
        }
        row = next(row for row in capture.sensor_rows(values, 2.0) if row["sensor"] == "MAG")
        self.assertEqual(row["publish_hz"], 50.0)
        self.assertEqual(row["consumed_hz"], 48.5)
        self.assertEqual(row["consume_age_avg_us"], 10.0)
        self.assertEqual(row["processed_out"], 95)
        self.assertEqual(row["telemetry_sent"], 94)
        self.assertEqual(row["conversion_commands"], 100)
        self.assertEqual(row["drdy_ready"], 98)
        self.assertEqual(row["drdy_misses"], 2)
        self.assertEqual(row["i2c_errors"], 0)

    def test_sensor_rows_derives_rc_queue_pressure(self):
        values = {
            "VELOXITY_DIAG_RC_PUBLISH": 400,
            "VELOXITY_DIAG_RC_QUEUE_FULL_WAITS": 2,
            "VELOXITY_DIAG_RC_QUEUE_WAIT_SUM_US": 50,
            "VELOXITY_DIAG_RC_QUEUE_WAIT_MAX_US": 30,
            "VELOXITY_DIAG_RC_QUEUE_DEPTH_MAX": 8,
        }
        row = next(
            row for row in capture.sensor_rows(values, 2.0) if row["sensor"] == "RC"
        )
        self.assertEqual(row["publish_hz"], 200.0)
        self.assertEqual(row["queue_full_waits"], 2)
        self.assertEqual(row["queue_wait_avg_us"], 25.0)
        self.assertEqual(row["queue_wait_max_us"], 30)
        self.assertEqual(row["queue_depth_max"], 8)


if __name__ == "__main__":
    unittest.main()
