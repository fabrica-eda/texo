//! Fast ECP5 placement-delay prediction.
//!
//! This deliberately mirrors the inexpensive predictor in nextpnr-ecp5 0.6
//! (`ecp5/arch.cc`) rather than searching the multi-million-wire routing graph
//! for every placement candidate. It recognizes ECP5 dedicated LUT/FF, carry,
//! and wide-LUT connections, then uses the same speed-grade-scaled distance
//! model as nextpnr for general routing.

use std::error::Error;
use std::fmt;

use texo_model::{BelId, BelPinId};

use super::Ecp5Architecture;

const NEAR_DISTANCE_TILES: u64 = 5;
const PLACEMENT_FIXED_UNITS: u64 = 3;
const LOGIC_CELL_Z_SHIFT: i32 = 2;

/// A construction error for [`Ecp5PlacementDelayPredictor`].
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Ecp5DelayPredictorError {
    /// The selected speed grade has no nextpnr-compatible distance scale.
    UnsupportedSpeedGrade(String),
}

impl fmt::Display for Ecp5DelayPredictorError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedSpeedGrade(grade) => {
                write!(formatter, "unsupported ECP5 speed grade `{grade}`")
            }
        }
    }
}

impl Error for Ecp5DelayPredictorError {}

/// Lightweight ECP5 delay predictor used by detailed placement.
///
/// Construction retains only the architecture and selected speed scale. It
/// does not inspect the physical PIP table, so even the largest ECP5 device
/// has constant-time placement-predictor setup.
pub struct Ecp5PlacementDelayPredictor<'a> {
    architecture: &'a Ecp5Architecture,
    distance_scale_ps: u64,
}

impl<'a> Ecp5PlacementDelayPredictor<'a> {
    /// Builds a lightweight placement predictor for one speed grade.
    ///
    /// # Errors
    ///
    /// Returns an error when the speed-grade name has no nextpnr-compatible
    /// distance scale.
    pub fn new(
        architecture: &'a Ecp5Architecture,
        speed_grade: &str,
    ) -> Result<Self, Ecp5DelayPredictorError> {
        let distance_scale_ps = distance_scale_ps(speed_grade).ok_or_else(|| {
            Ecp5DelayPredictorError::UnsupportedSpeedGrade(speed_grade.to_owned())
        })?;
        Ok(Self {
            architecture,
            distance_scale_ps,
        })
    }

    /// Predicts one placed BEL-pin arc in picoseconds.
    ///
    /// The dedicated-connection cases and the three-unit general-routing
    /// access cost match nextpnr-ecp5 0.6's `Arch::predictDelay`. The caller
    /// supplies concrete physical pins, so packed candidate-specific pin
    /// choices such as FF `DI` versus `M` remain visible to the predictor.
    #[must_use]
    pub fn predict_placement_delay_ps(
        &self,
        source_bel: BelId,
        source_pin: BelPinId,
        sink_bel: BelId,
        sink_pin: BelPinId,
    ) -> Option<u64> {
        let device = self.architecture.device();
        let source_bel_data = device.bels().get(source_bel.0)?;
        let sink_bel_data = device.bels().get(sink_bel.0)?;
        let source_pin_data = device.bel_pins().get(source_pin.0)?;
        let sink_pin_data = device.bel_pins().get(sink_pin.0)?;
        if source_pin_data.bel != source_bel || sink_pin_data.bel != sink_bel {
            return None;
        }

        let source_name = source_pin_data.name.as_str();
        let sink_name = sink_pin_data.name.as_str();
        if (source_name == "FCO" && sink_name == "FCI")
            || matches!(sink_name, "FXA" | "FXB")
            || (source_name == "F" && sink_name == "DI")
        {
            return Some(0);
        }

        if source_bel_data.point == sink_bel_data.point && is_lut_input(sink_name) {
            let source_logic_cell =
                self.architecture.bel_metadata(source_bel).z >> LOGIC_CELL_Z_SHIFT;
            let sink_logic_cell = self.architecture.bel_metadata(sink_bel).z >> LOGIC_CELL_Z_SHIFT;
            if (source_name == "Q" && source_logic_cell == sink_logic_cell)
                || (source_name == "F" && !matches!(source_logic_cell, 1 | 6))
            {
                return Some(0);
            }
        }

        Some(distance_delay_ps(
            self.distance_scale_ps,
            PLACEMENT_FIXED_UNITS,
            source_bel_data.point.x.abs_diff(sink_bel_data.point.x),
            source_bel_data.point.y.abs_diff(sink_bel_data.point.y),
        ))
    }
}

impl texo_pnr::PlacementDelayEstimator for Ecp5PlacementDelayPredictor<'_> {
    fn estimate_delay_ps(
        &self,
        driver_bel: BelId,
        driver_pin: BelPinId,
        sink_bel: BelId,
        sink_pin: BelPinId,
    ) -> u64 {
        self.predict_placement_delay_ps(driver_bel, driver_pin, sink_bel, sink_pin)
            .expect("validated ECP5 placement supplies matching BEL pins")
    }
}

fn is_lut_input(name: &str) -> bool {
    matches!(name, "A" | "B" | "C" | "D")
}

/// nextpnr encodes speed grades as indices `6 -> 0`, `7 -> 1`, `8 -> 2`,
/// and the 5G-only grade `8_5G -> 3`, then uses `120 - 22 * index` ps.
fn distance_scale_ps(speed_grade: &str) -> Option<u64> {
    let speed_index = match speed_grade {
        "6" => 0,
        "7" => 1,
        "8" => 2,
        "8_5G" => 3,
        _ => return None,
    };
    Some(120 - 22 * speed_index)
}

fn distance_delay_ps(scale_ps: u64, fixed_units: u64, dx: u32, dy: u32) -> u64 {
    let dx = u64::from(dx);
    let dy = u64::from(dy);
    let near = dx.min(NEAR_DISTANCE_TILES) + dy.min(NEAR_DISTANCE_TILES);
    let far = dx.saturating_sub(NEAR_DISTANCE_TILES) + dy.saturating_sub(NEAR_DISTANCE_TILES);
    scale_ps.saturating_mul(fixed_units + near.saturating_mul(2) + far)
}

#[cfg(test)]
mod tests {
    use texo_model::{BelId, BelPinId};

    use super::{
        Ecp5DelayPredictorError, Ecp5PlacementDelayPredictor, PLACEMENT_FIXED_UNITS,
        distance_delay_ps, distance_scale_ps,
    };
    use crate::read_architecture;

    const FIXTURE: &str = include_str!("../fixtures/minimal-ecp5.json");

    #[test]
    fn nextpnr_speed_indices_select_the_same_distance_scales() {
        assert_eq!(distance_scale_ps("6"), Some(120));
        assert_eq!(distance_scale_ps("7"), Some(98));
        assert_eq!(distance_scale_ps("8"), Some(76));
        assert_eq!(distance_scale_ps("8_5G"), Some(54));
        assert_eq!(distance_scale_ps("5"), None);
    }

    #[test]
    fn placement_model_keeps_nextpnr_fixed_costs() {
        let scale = distance_scale_ps("8_5G").unwrap();

        // `Arch::predictDelay` uses the same curve with three access units.
        assert_eq!(PLACEMENT_FIXED_UNITS, 3);
        assert_eq!(distance_delay_ps(scale, PLACEMENT_FIXED_UNITS, 0, 0), 162);
        assert_eq!(distance_delay_ps(scale, PLACEMENT_FIXED_UNITS, 1, 0), 270);
    }

    #[test]
    fn dedicated_lut_ff_and_same_logic_cell_arcs_are_free() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        let device = architecture.device();
        let predictor = Ecp5PlacementDelayPredictor::new(&architecture, "6").unwrap();
        let lut0 = bel(device, "R0C0/SLICEA.K0");
        let lut1 = bel(device, "R0C0/SLICEA.K1");
        let ff0 = bel(device, "R0C0/SLICEA.FF0");

        assert_eq!(
            predictor.predict_placement_delay_ps(
                lut0,
                pin(device, lut0, "F"),
                ff0,
                pin(device, ff0, "DI"),
            ),
            Some(0)
        );
        assert_eq!(
            predictor.predict_placement_delay_ps(
                lut0,
                pin(device, lut0, "FCO"),
                lut1,
                pin(device, lut1, "FCI"),
            ),
            Some(0)
        );
        assert_eq!(
            predictor.predict_placement_delay_ps(
                lut0,
                pin(device, lut0, "F"),
                lut1,
                pin(device, lut1, "FXA"),
            ),
            Some(0)
        );
        assert_eq!(
            predictor.predict_placement_delay_ps(
                ff0,
                pin(device, ff0, "Q"),
                lut0,
                pin(device, lut0, "A"),
            ),
            Some(0)
        );
        assert_eq!(
            predictor.predict_placement_delay_ps(
                ff0,
                pin(device, ff0, "Q"),
                lut1,
                pin(device, lut1, "A"),
            ),
            Some(360)
        );
    }

    #[test]
    fn rejects_unknown_speed_grades() {
        let architecture = read_architecture(FIXTURE.as_bytes()).unwrap();
        assert_eq!(
            Ecp5PlacementDelayPredictor::new(&architecture, "9").err(),
            Some(Ecp5DelayPredictorError::UnsupportedSpeedGrade("9".into()))
        );
    }

    fn bel(device: &texo_model::Device, name: &str) -> BelId {
        device
            .bels()
            .iter()
            .position(|bel| bel.name == name)
            .map(BelId)
            .unwrap()
    }

    fn pin(device: &texo_model::Device, bel: BelId, name: &str) -> BelPinId {
        device.bels()[bel.0]
            .pins()
            .iter()
            .copied()
            .find(|pin| device.bel_pins()[pin.0].name == name)
            .unwrap()
    }
}
