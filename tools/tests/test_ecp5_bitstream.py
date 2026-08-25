import unittest
from types import SimpleNamespace

from tools import ecp5_bitstream


class FakeTileConfig:
    def __init__(self):
        self.enums = {}
        self.words = {}

    def add_enum(self, name, value):
        self.enums[name] = value

    def add_word(self, name, values):
        self.words[name] = list(values)


class FakePytrellis:
    BoolVector = list
    TileConfig = FakeTileConfig


class FakeChip:
    def __init__(self):
        self.tiles = {
            (20, 10): [SimpleNamespace(info=SimpleNamespace(name="MIB_R20C10:MIB_EBR0", type="MIB_EBR0"))],
            (20, 11): [SimpleNamespace(info=SimpleNamespace(name="MIB_R20C11:MIB_EBR1", type="MIB_EBR1"))],
        }

    def get_tiles_by_position(self, y, x):
        return self.tiles.get((y, x), [])


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

    def test_release_gate_accepts_only_schema_three_timing_closed_ecp5(self):
        checkpoint = {
            "schema_version": 3,
            "evidence": sorted(ecp5_bitstream.REQUIRED_EVIDENCE),
            "timing": {"met_timing": True},
            "target": {"family": "ECP5"},
        }
        ecp5_bitstream.validate_checkpoint(checkpoint)

        checkpoint["schema_version"] = 2
        with self.assertRaisesRegex(RuntimeError, "schema version 3"):
            ecp5_bitstream.validate_checkpoint(checkpoint)

    def test_dp16kd_writer_emits_tile_group_cib_ties_and_zero_initialization(self):
        config = SimpleNamespace(tiles={})
        placement = {
            "cell": "words",
            "cell_id": 7,
            "bel": "R20C10/EBR0",
            "bel_z": 0,
            "x": 10,
            "y": 20,
            "bel_pins": [
                {"name": "WEB", "cib_tie": {"tile": "CIB_R20C10:CIB_EBR", "mux": "JLSR0"}},
                {"name": "DIA8", "cib_tie": {"tile": "CIB_R20C10:CIB_EBR", "mux": "JD0"}},
                {"name": "CSA0", "cib_tie": {"tile": "CIB_R20C10:CIB_EBR", "mux": "JCE0"}},
            ],
        }
        configuration = {
            "kind": "block_ram",
            "depth": 256,
            "word_width": 8,
            "physical_width": 9,
            "edge": "rising",
            "write_enable": "high",
            "read_enable": None,
        }
        packed = {"cell": 7, "wid": 3, "depth": 256, "word_width": 8, "physical_width": 9}
        tile_groups = []
        bram_data = {}

        ecp5_bitstream.write_bram(
            FakePytrellis,
            config,
            FakeChip(),
            placement,
            configuration,
            packed,
            {"WEB": False, "DIA8": False, "CSA0": False},
            tile_groups,
            bram_data,
        )

        tiles, group = tile_groups[0]
        self.assertEqual(tiles, ["MIB_R20C10:MIB_EBR0", "MIB_R20C11:MIB_EBR1"])
        self.assertEqual(group.enums["EBR0.MODE"], "DP16KD")
        self.assertEqual(group.enums["EBR0.DP16KD.DATA_WIDTH_A"], "9")
        self.assertEqual(group.enums["EBR0.CEAMUX"], "CEA")
        self.assertEqual(group.words["EBR0.CSDECODE_A"], [False, False, True])
        self.assertEqual(
            group.words["EBR0.WID"],
            ecp5_bitstream.integer_bits(ecp5_bitstream.reverse_bits(3, 9), 9),
        )
        self.assertEqual(config.tiles["CIB_R20C10:CIB_EBR"].enums["CIB.JLSR0MUX"], "0")
        self.assertEqual(config.tiles["CIB_R20C10:CIB_EBR"].enums["CIB.JD0MUX"], "0")
        self.assertEqual(config.tiles["CIB_R20C10:CIB_EBR"].enums["CIB.JCE0MUX"], "1")
        self.assertEqual(len(bram_data[3]), 2048)
        self.assertFalse(any(bram_data[3]))


if __name__ == "__main__":
    unittest.main()
