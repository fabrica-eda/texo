import unittest

from tools import ecp5_bitstream


class NativeBitstreamTests(unittest.TestCase):
    def test_fold_lut_input_replicates_the_selected_truth_table_plane(self):
        self.assertEqual(ecp5_bitstream.fold_lut_input(0xAAAA, 0, False), 0x0000)
        self.assertEqual(ecp5_bitstream.fold_lut_input(0xAAAA, 0, True), 0xFFFF)

    def test_trellis_wire_name_is_relative_to_the_configuration_tile(self):
        self.assertEqual(
            ecp5_bitstream.trellis_wire_name((5, 7), "R6C8/JLOCAL"),
            "N1E3_JLOCAL",
        )
        self.assertEqual(
            ecp5_bitstream.trellis_wire_name((5, 7), "R6C8/G_HPBX0000"),
            "G_HPBX0000",
        )

    def test_release_gate_accepts_only_schema_two_timing_closed_ecp5(self):
        checkpoint = {
            "schema_version": 2,
            "evidence": sorted(ecp5_bitstream.REQUIRED_EVIDENCE),
            "timing": {"met_timing": True},
            "target": {"family": "ECP5"},
        }
        ecp5_bitstream.validate_checkpoint(checkpoint)

        checkpoint["schema_version"] = 1
        with self.assertRaisesRegex(RuntimeError, "schema version 2"):
            ecp5_bitstream.validate_checkpoint(checkpoint)


if __name__ == "__main__":
    unittest.main()
