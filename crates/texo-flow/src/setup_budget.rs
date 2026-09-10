//! A soft setup-search limit owned by one flow invocation.

use std::time::{Duration, Instant};

pub(super) struct SetupBudget {
    limit: Option<Duration>,
    started: Option<Instant>,
    reported: bool,
}

impl SetupBudget {
    pub(super) const fn new(limit: Option<Duration>) -> Self {
        Self {
            limit,
            started: None,
            reported: false,
        }
    }

    pub(super) fn exhausted(&mut self) -> bool {
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
