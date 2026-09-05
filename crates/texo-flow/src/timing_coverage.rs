//! Explicit review of modeled endpoints omitted from setup/hold analysis.

use std::collections::BTreeMap;
use std::error::Error;
use std::fmt;

use serde::{Deserialize, Serialize};
use texo_model::Design;
use texo_timing::TimingReport;

/// One exact, reviewed exception to modeled endpoint timing coverage.
///
/// Only `no_synchronous_launch` may be excepted. A missing capture clock or
/// period must be fixed in the design/constraints, not waived. Names are
/// literal (no glob matching), and every exception must be used by this run.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TimingEndpointException {
    /// Exact mapped cell name.
    pub cell: String,
    /// Exact data input pin name, such as `DI`.
    pub data_pin: String,
    /// Expected unchecked-endpoint diagnostic: `no_synchronous_launch`.
    pub reason: String,
    /// Nonempty explanation of the external timing or CDC verification.
    pub justification: String,
}

/// An unreviewed endpoint, invalid exception, or stale exception.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TimingCoverageError(String);

impl fmt::Display for TimingCoverageError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0.fmt(formatter)
    }
}

impl Error for TimingCoverageError {}

/// Requires an exact, reason-specific exception for every unchecked endpoint.
///
/// Each input tuple is `(cell name, data pin name, unchecked reason)`. This
/// common validator is used both by the physical flow and when reading a saved
/// checkpoint for bitgen. Exceptions record review; they do not create timing
/// checks or characterize primitives absent from the timing model.
///
/// # Errors
///
/// Rejects unreviewed endpoints, missing capture clocks/periods, duplicate
/// endpoints or exceptions, empty justifications, and unused exceptions.
pub fn validate_timing_coverage<'a>(
    unchecked: impl IntoIterator<Item = (&'a str, &'a str, &'a str)>,
    exceptions: &[TimingEndpointException],
) -> Result<(), TimingCoverageError> {
    let mut remaining = BTreeMap::new();
    for exception in exceptions {
        if exception.reason != "no_synchronous_launch"
            || exception.cell.is_empty()
            || exception.data_pin.is_empty()
            || exception.justification.trim().is_empty()
        {
            return Err(TimingCoverageError(format!(
                "invalid timing exception for {}.{}: only no_synchronous_launch \
                 with an exact cell/pin and a nonempty justification is permitted",
                exception.cell, exception.data_pin,
            )));
        }
        let key = (exception.cell.as_str(), exception.data_pin.as_str());
        if remaining.insert(key, exception).is_some() {
            return Err(TimingCoverageError(format!(
                "duplicate timing exception for {}.{}",
                key.0, key.1,
            )));
        }
    }
    let mut seen = std::collections::BTreeSet::new();
    for (cell, data_pin, reason) in unchecked {
        if !seen.insert((cell, data_pin)) {
            return Err(TimingCoverageError(format!(
                "duplicate unchecked endpoint {cell}.{data_pin}",
            )));
        }
        if reason != "no_synchronous_launch" {
            return Err(TimingCoverageError(format!(
                "unchecked endpoint {cell}.{data_pin}: {reason}; \
                 capture clocks must be connected and constrained",
            )));
        }
        if remaining.remove(&(cell, data_pin)).is_none() {
            return Err(TimingCoverageError(format!(
                "unreviewed timing endpoint {cell}.{data_pin}: {reason}; \
                 supply an exact timing exception with a justification",
            )));
        }
    }
    if let Some(((cell, data_pin), _)) = remaining.first_key_value() {
        return Err(TimingCoverageError(format!(
            "unused timing exception for {cell}.{data_pin}; \
             review exceptions against this implementation",
        )));
    }
    Ok(())
}

pub(crate) fn validate_report_coverage(
    design: &Design,
    timing: &TimingReport,
    exceptions: &[TimingEndpointException],
) -> Result<(), TimingCoverageError> {
    validate_timing_coverage(
        timing.unchecked_endpoints.iter().map(|endpoint| {
            (
                design.cells()[endpoint.cell.0].name.as_str(),
                design.pins()[endpoint.data_pin.0].name.as_str(),
                endpoint.reason.as_str(),
            )
        }),
        exceptions,
    )
}

#[cfg(test)]
mod tests {
    use super::{TimingEndpointException, validate_timing_coverage};

    fn cdc_exception() -> TimingEndpointException {
        TimingEndpointException {
            cell: "request_meta".into(),
            data_pin: "DI".into(),
            reason: "no_synchronous_launch".into(),
            justification: "First stage of a separately reviewed toggle synchronizer".into(),
        }
    }

    #[test]
    fn coverage_requires_an_exact_review_and_rejects_new_endpoints() {
        let cdc = ("request_meta", "DI", "no_synchronous_launch");
        assert!(validate_timing_coverage([], &[]).is_ok());
        assert!(validate_timing_coverage([cdc], &[]).is_err());
        assert!(validate_timing_coverage([cdc], &[cdc_exception()]).is_ok());
        for additional in [
            ("new_meta", "DI", "no_synchronous_launch"),
            ("request_meta", "CE", "no_synchronous_launch"),
        ] {
            assert!(validate_timing_coverage([cdc, additional], &[cdc_exception()]).is_err());
        }
    }

    #[test]
    fn missing_capture_clocks_and_unknown_reasons_cannot_be_excepted() {
        for reason in ["unconstrained_clock", "unconnected_clock", "unknown_reason"] {
            let endpoint = ("request_meta", "DI", reason);
            assert!(validate_timing_coverage([endpoint], &[cdc_exception()]).is_err());
            let mut exception = cdc_exception();
            exception.reason = reason.into();
            assert!(validate_timing_coverage([endpoint], &[exception]).is_err());
        }
    }

    #[test]
    fn stale_duplicate_and_unjustified_exceptions_are_errors() {
        let cdc = ("request_meta", "DI", "no_synchronous_launch");
        assert!(validate_timing_coverage([], &[cdc_exception()]).is_err());
        assert!(validate_timing_coverage([cdc], &[cdc_exception(), cdc_exception()]).is_err());
        assert!(validate_timing_coverage([cdc, cdc], &[cdc_exception()]).is_err());
        let mut exception = cdc_exception();
        exception.justification = " \n ".into();
        assert!(validate_timing_coverage([cdc], &[exception]).is_err());
    }

    #[test]
    fn exception_file_rejects_unknown_fields() {
        let mut record = serde_json::to_value(cdc_exception()).unwrap();
        record["justificaton"] = serde_json::json!("typo");
        assert!(serde_json::from_value::<TimingEndpointException>(record).is_err());
    }
}
