//! A soft setup-search limit owned by one flow invocation.

use std::time::{Duration, Instant};

pub(super) struct SetupBudget {
    limit: Option<Duration>,
    started: Option<Instant>,
    reported: bool,
    /// Budget checks so far. Setup searches check the budget once per trial
    /// boundary, so this is a deterministic measure of setup work.
    steps: u64,
    /// Optional cap on `steps`, independent of the time limit.
    step_limit: Option<u64>,
}

impl SetupBudget {
    pub(super) const fn new(limit: Option<Duration>) -> Self {
        Self {
            limit,
            started: None,
            reported: false,
            steps: 0,
            step_limit: None,
        }
    }

    /// Starts the time limit now, if it has not started yet.
    pub(super) fn start(&mut self) {
        self.started.get_or_insert_with(Instant::now);
    }

    /// Budget checks so far.
    pub(super) const fn steps(&self) -> u64 {
        self.steps
    }

    /// Also stops once `steps` more budget checks have been made, until
    /// [`Self::clear_step_limit`].
    pub(super) fn limit_steps(&mut self, steps: u64) {
        self.step_limit = Some(self.steps.saturating_add(steps));
    }

    /// Whether a step limit is set and has been reached. Unlike
    /// [`Self::exhausted`], this does not count as a step.
    pub(super) fn step_limit_reached(&self) -> bool {
        self.step_limit.is_some_and(|limit| self.steps >= limit)
    }

    pub(super) fn clear_step_limit(&mut self) {
        self.step_limit = None;
    }

    pub(super) fn exhausted(&mut self) -> bool {
        self.steps += 1;
        if self.step_limit.is_some_and(|limit| self.steps > limit) {
            return true;
        }
        let exhausted = self.exhausted_at(Instant::now());
        if exhausted && !self.reported {
            self.reported = true;
            eprintln!(
                "setup optimization budget exhausted; preserving the last fully checked implementation"
            );
        }
        exhausted
    }

    fn exhausted_at(&mut self, now: Instant) -> bool {
        self.limit
            .is_some_and(|limit| now.duration_since(*self.started.get_or_insert(now)) >= limit)
    }
}

#[cfg(test)]
mod tests {
    use super::{Duration, Instant, SetupBudget};

    #[test]
    fn step_limits_stop_a_probe_and_then_release_the_budget() {
        let mut budget = SetupBudget::new(None);
        assert!(!budget.exhausted());
        budget.limit_steps(2);
        assert!(!budget.step_limit_reached());
        assert!(!budget.exhausted());
        assert!(!budget.exhausted());
        assert!(budget.step_limit_reached());
        assert!(budget.exhausted());
        assert_eq!(budget.steps(), 4);
        budget.clear_step_limit();
        assert!(!budget.step_limit_reached());
        assert!(!budget.exhausted());
    }

    #[test]
    fn setup_limits_are_independent_between_flow_invocations() {
        let start = Instant::now();
        let mut first = SetupBudget::new(Some(Duration::from_secs(10)));
        let mut second = SetupBudget::new(Some(Duration::from_secs(20)));
        assert!(!first.exhausted_at(start));
        assert!(!second.exhausted_at(start));
        assert!(first.exhausted_at(start + Duration::from_secs(10)));
        assert!(!second.exhausted_at(start + Duration::from_secs(10)));
        assert!(second.exhausted_at(start + Duration::from_secs(20)));
        let mut later = SetupBudget::new(Some(Duration::from_secs(10)));
        assert!(!later.exhausted_at(start + Duration::from_secs(100)));
    }

    #[test]
    fn initial_implementation_does_not_spend_the_setup_search_budget() {
        let start = Instant::now();
        let mut budget = SetupBudget::new(Some(Duration::from_secs(1)));
        let first_setup_search = start + Duration::from_secs(60);
        assert!(!budget.exhausted_at(first_setup_search));
        assert!(budget.exhausted_at(first_setup_search + Duration::from_secs(1)));
    }

    #[test]
    fn zero_skips_search_and_none_is_unlimited() {
        let start = Instant::now();
        assert!(SetupBudget::new(Some(Duration::ZERO)).exhausted_at(start));
        let mut unlimited = SetupBudget::new(None);
        assert!(!unlimited.exhausted_at(start));
        assert!(!unlimited.exhausted_at(start + Duration::from_hours(24)));
    }
}
