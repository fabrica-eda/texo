"""Checks for the DSP timing import boundary; no Trellis installation needed."""
import copy
import unittest
from export_ecp5 import export_multiplier_timing


class MultiplierTimingTests(unittest.TestCase):
    def setUp(self):
        self.records = [
            {"type": "IOPath", "from_pin": source, "to_pin": "P",
             "rising": [1930, 2234, 2538], "falling": [1920, 2240, 2530]}
            for source in ["A", "B", "SIGNEDA", "SIGNEDB"]
        ]

    def export(self, records):
        return export_multiplier_timing({"MULT18X18D:REGS=NONE": records})

    def test_scalar_surface_and_independent_delay_corners(self):
        timing = self.export(self.records)
        arcs = {(arc["from_pin"], arc["to_pin"]): arc["delay"] for arc in timing["arcs"]}
        self.assertEqual(len(arcs), 38 * 36)
        for source in ["A0", "A17", "B0", "B17", "SIGNEDA", "SIGNEDB"]:
            for destination in ["P0", "P17", "P35"]:
                self.assertEqual(arcs[source, destination], {"min_ps": 1920, "max_ps": 2538})
        self.assertEqual(timing["setup_holds"], [])

    def test_rejects_missing_and_duplicate_arcs(self):
        for records in [[], self.records[:-1], self.records + self.records[:1]]:
            with self.assertRaises(ValueError):
                self.export(records)

    def test_rejects_unmodeled_modes_and_ports(self):
        for field, value in [("type", "SetupHold"), ("from_pin", "C"), ("to_pin", "ROA")]:
            records = copy.deepcopy(self.records)
            records[0][field] = value
            with self.assertRaises(ValueError):
                self.export(records)


if __name__ == "__main__":
    unittest.main()
