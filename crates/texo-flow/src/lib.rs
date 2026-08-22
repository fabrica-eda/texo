//! Flow orchestration and explicit verification evidence.

use std::collections::BTreeSet;
use std::error::Error;
use std::fmt;

use texo_model::{Design, Device};
use texo_pnr::{PnrError, PnrResult, place_and_route};

/// Evidence required before a programmable artifact may be released.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum Gate {
    /// Source-level functional simulation passed.
    RtlSimulation,
    /// Synthesized logic is equivalent to the RTL reference.
    SynthesisEquivalence,
    /// No unresolved mapped primitive remains.
    MappedNetlistComplete,
    /// Celox post-map simulation passed.
    PostMapSimulation,
    /// `PnR` completed and independent physical checks passed.
    PhysicalImplementation,
    /// Static timing constraints were met.
    TimingClosure,
}

/// Accumulated immutable-style verification record.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Evidence {
    passed: BTreeSet<Gate>,
}

impl Evidence {
    /// Creates an empty evidence set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            passed: BTreeSet::new(),
        }
    }

    /// Records a passed gate.
    pub fn record(&mut self, gate: Gate) {
        self.passed.insert(gate);
    }

    /// Whether a gate has passed.
    #[must_use]
    pub fn contains(&self, gate: Gate) -> bool {
        self.passed.contains(&gate)
    }

    /// Checks all bitstream release gates.
    ///
    /// # Errors
    ///
    /// Returns every missing gate.
    pub fn authorize_bitstream(&self) -> Result<(), MissingEvidence> {
        let missing = REQUIRED_GATES
            .into_iter()
            .filter(|gate| !self.contains(*gate))
            .collect::<Vec<_>>();
        if missing.is_empty() {
            Ok(())
        } else {
            Err(MissingEvidence { missing })
        }
    }
}

const REQUIRED_GATES: [Gate; 6] = [
    Gate::RtlSimulation,
    Gate::SynthesisEquivalence,
    Gate::MappedNetlistComplete,
    Gate::PostMapSimulation,
    Gate::PhysicalImplementation,
    Gate::TimingClosure,
];

/// Runs the physical implementation stage and records its evidence.
///
/// # Errors
///
/// Propagates placement or routing failures without recording the gate.
pub fn implement(
    design: &Design,
    device: &Device,
    evidence: &mut Evidence,
) -> Result<PnrResult, PnrError> {
    let result = place_and_route(design, device)?;
    evidence.record(Gate::PhysicalImplementation);
    Ok(result)
}

/// Missing bitstream release evidence.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissingEvidence {
    missing: Vec<Gate>,
}

impl MissingEvidence {
    /// Missing gates in stable pipeline order.
    #[must_use]
    pub fn gates(&self) -> &[Gate] {
        &self.missing
    }
}

impl fmt::Display for MissingEvidence {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} verification gate(s) are missing", self.missing.len())
    }
}

impl Error for MissingEvidence {}

#[cfg(test)]
mod tests {
    use texo_model::{Design, Device, PinDirection, ResourceKind};

    use super::{Evidence, Gate, implement};

    #[test]
    fn implementation_records_only_its_own_gate() {
        let mut design = Design::new();
        let a = design.add_cell("a", ResourceKind::Logic);
        let a_out = design.add_pin(a, "out", PinDirection::Output).unwrap();
        let b = design.add_cell("b", ResourceKind::Logic);
        let b_in = design.add_pin(b, "in", PinDirection::Input).unwrap();
        design.add_net("n", a_out, [b_in]).unwrap();
        let device = Device::rectangular_logic(4, 4).unwrap();
        let mut evidence = Evidence::new();

        implement(&design, &device, &mut evidence).unwrap();

        assert!(evidence.contains(Gate::PhysicalImplementation));
        assert!(!evidence.contains(Gate::TimingClosure));
        assert!(evidence.authorize_bitstream().is_err());
    }
}
