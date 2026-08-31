//! Deterministic implicit auction for a rectangular unit-to-object assignment.
//!
//! Candidate edges stay in the caller's spatial index. The auction asks for
//! only the two cheapest real objects at the current prices, which lets the
//! caller stop a Manhattan-ring walk once the next ring's price-free lower
//! bound exceeds the second-best reduced cost.

use std::collections::VecDeque;
use std::time::Instant;

/// One priced real object returned by an implicit candidate search.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct PricedObject {
    pub(super) object: usize,
    pub(super) base_cost: u64,
    /// `base_cost + object_price`.
    pub(super) reduced_cost: u64,
}

/// Exact two-best result of one implicit candidate search.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(super) struct BestObjects {
    pub(super) best: Option<PricedObject>,
    pub(super) second: Option<PricedObject>,
    pub(super) examined: u64,
}

/// Work performed by one complete auction.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(super) struct AuctionStats {
    pub(super) bids: u64,
    pub(super) candidates_examined: u64,
    pub(super) unmatched: usize,
}

/// Real-object assignment for every bidder.
#[derive(Clone, Debug, Eq, PartialEq)]
pub(super) struct AuctionResult {
    /// `None` denotes the bidder's private, deliberately expensive dummy.
    pub(super) assignment: Vec<Option<usize>>,
    pub(super) stats: AuctionStats,
}

/// Malformed implicit candidate data or integer range exhaustion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum AuctionError {
    ArithmeticOverflow,
    ObjectOutOfRange,
    BaseCostOutOfRange,
    IncorrectReducedCost,
    DuplicateBestObject,
    UnsortedBestObjects,
    SecondWithoutBest,
}

struct AuctionMetrics {
    started: Option<Instant>,
    bidder_count: usize,
    object_count: usize,
}

impl AuctionMetrics {
    fn new(bidder_count: usize, object_count: usize, max_base_cost: u64) -> Self {
        let started = std::env::var_os("TEXO_PNR_VERBOSE_METRICS").map(|_| Instant::now());
        if started.is_some() {
            eprintln!(
                "TEXO_PNR_VERBOSE_METRICS auction start bidders={bidder_count} objects={object_count} max_base_cost={max_base_cost}"
            );
        }
        Self {
            started,
            bidder_count,
            object_count,
        }
    }

    fn progress(&self, stats: AuctionStats, queue: usize, object_price: u64) {
        if stats.bids.is_power_of_two()
            && let Some(started) = self.started
        {
            eprintln!(
                "TEXO_PNR_VERBOSE_METRICS auction progress elapsed_ms={} bidders={} objects={} bids={} candidates_examined={} queue={queue} object_price={object_price}",
                started.elapsed().as_millis(),
                self.bidder_count,
                self.object_count,
                stats.bids,
                stats.candidates_examined,
            );
        }
    }

    fn complete(&self, stats: AuctionStats) {
        if let Some(started) = self.started {
            eprintln!(
                "TEXO_PNR_VERBOSE_METRICS auction complete elapsed_ms={} bidders={} objects={} bids={} candidates_examined={} unmatched={}",
                started.elapsed().as_millis(),
                self.bidder_count,
                self.object_count,
                stats.bids,
                stats.candidates_examined,
                stats.unmatched,
            );
        }
    }
}

fn empty_result() -> AuctionResult {
    AuctionResult {
        assignment: Vec::new(),
        stats: AuctionStats::default(),
    }
}

/// Solves the integer-cost assignment relaxation without materializing edges.
///
/// Base costs are nonnegative integers no larger than `max_base_cost`. The
/// epsilon-one result has exact maximum cardinality and total base cost at
/// most `bidder_count` above the minimum among assignments of that
/// cardinality. Thus its average *excess over that same-cardinality optimum*
/// is at most one tile per bidder; this is not a bound on the placement's
/// absolute displacement. Avoiding a bidder-count cost scale also avoids
/// multiplying every price-war step by bidder count.
///
/// Every bidder also owns a private dummy of cost
/// `bidder_count * max_base_cost + bidder_count + 1`. The gap exceeds both the
/// worst possible real-cost change and the auction's total epsilon error, so
/// an extra dummy can never replace a real match. Dummies make restricted
/// candidate graphs total and report the maximum-cardinality deficiency
/// without an iteration or search limit. It does not prove that any particular
/// bidder identity must be unmatched in every maximum matching.
pub(super) fn assign_implicitly(
    bidder_count: usize,
    object_count: usize,
    max_base_cost: u64,
    mut best_real_objects: impl FnMut(usize, &[u64], u64) -> Result<BestObjects, AuctionError>,
) -> Result<AuctionResult, AuctionError> {
    let metrics = AuctionMetrics::new(bidder_count, object_count, max_base_cost);
    if bidder_count == 0 {
        let result = empty_result();
        metrics.complete(result.stats);
        return Ok(result);
    }

    let bidder_count_u64 =
        u64::try_from(bidder_count).map_err(|_| AuctionError::ArithmeticOverflow)?;
    let cost_scale = 1_u64;
    // `D - n * max_base_cost = n + 1` is strictly larger than the total
    // `n * epsilon` error. Therefore the number of dummies is lexicographically
    // minimal even though equal-cardinality Manhattan cost is approximate.
    let dummy_cost = bidder_count_u64
        .checked_mul(max_base_cost)
        .and_then(|cost| cost.checked_add(bidder_count_u64))
        .and_then(|cost| cost.checked_add(1))
        .ok_or(AuctionError::ArithmeticOverflow)?;
    // A selected real object's new price is
    // `second_reduced_cost - base_cost + epsilon`, and the private dummy
    // bounds `second_reduced_cost`. Thus every price is at most `D + 1`.
    // Prove that adding the largest base cost remains representable before
    // using native-width arithmetic in the hot candidate loop.
    dummy_cost
        .checked_add(1)
        .and_then(|price| price.checked_add(max_base_cost))
        .ok_or(AuctionError::ArithmeticOverflow)?;
    let mut prices = vec![0_u64; object_count];
    let mut owners = vec![None::<usize>; object_count];
    let mut assignment = vec![None::<usize>; bidder_count];
    let mut queue = (0..bidder_count).collect::<VecDeque<_>>();
    let mut stats = AuctionStats::default();

    while let Some(bidder) = queue.pop_front() {
        let real = best_real_objects(bidder, &prices, cost_scale)?;
        if real.best.is_none() && real.second.is_some() {
            return Err(AuctionError::SecondWithoutBest);
        }
        stats.candidates_examined = stats
            .candidates_examined
            .checked_add(real.examined)
            .ok_or(AuctionError::ArithmeticOverflow)?;
        for choice in [real.best, real.second].into_iter().flatten() {
            if choice.object >= object_count {
                return Err(AuctionError::ObjectOutOfRange);
            }
            if choice.base_cost > max_base_cost {
                return Err(AuctionError::BaseCostOutOfRange);
            }
            let expected = cost_scale
                .checked_mul(choice.base_cost)
                .and_then(|cost| cost.checked_add(prices[choice.object]))
                .ok_or(AuctionError::ArithmeticOverflow)?;
            if choice.reduced_cost != expected {
                return Err(AuctionError::IncorrectReducedCost);
            }
        }
        if real
            .best
            .zip(real.second)
            .is_some_and(|(best, second)| best.object == second.object)
        {
            return Err(AuctionError::DuplicateBestObject);
        }
        if real.best.zip(real.second).is_some_and(|(best, second)| {
            (best.reduced_cost, best.object) > (second.reduced_cost, second.object)
        }) {
            return Err(AuctionError::UnsortedBestObjects);
        }

        let mut choices = [real.best, real.second, None];
        let mut count = usize::from(real.best.is_some()) + usize::from(real.second.is_some());
        // `usize::MAX` makes a real object win a stable equal-cost tie.
        choices[count] = Some(PricedObject {
            object: usize::MAX,
            base_cost: max_base_cost,
            reduced_cost: dummy_cost,
        });
        count += 1;
        choices[..count].sort_unstable_by_key(|choice| {
            let choice = choice.expect("populated auction choice");
            (choice.reduced_cost, choice.object)
        });
        let best = choices[0].expect("private dummy makes every auction total");
        if best.object == usize::MAX {
            assignment[bidder] = None;
            continue;
        }
        let second = choices[1].expect("private dummy supplies a second choice");
        let increment = second
            .reduced_cost
            .checked_sub(best.reduced_cost)
            .and_then(|difference| difference.checked_add(1))
            .ok_or(AuctionError::ArithmeticOverflow)?;
        prices[best.object] = prices[best.object]
            .checked_add(increment)
            .ok_or(AuctionError::ArithmeticOverflow)?;
        if let Some(displaced) = owners[best.object].replace(bidder) {
            assignment[displaced] = None;
            queue.push_back(displaced);
        }
        assignment[bidder] = Some(best.object);
        stats.bids = stats
            .bids
            .checked_add(1)
            .ok_or(AuctionError::ArithmeticOverflow)?;
        metrics.progress(stats, queue.len(), prices[best.object]);
    }

    stats.unmatched = assignment.iter().filter(|object| object.is_none()).count();
    metrics.complete(stats);
    Ok(AuctionResult { assignment, stats })
}

#[cfg(test)]
mod tests {
    use super::{BestObjects, PricedObject, assign_implicitly};

    fn explicit_best_two(
        domains: &[Vec<(usize, u64)>],
        bidder: usize,
        prices: &[u64],
        scale: u64,
    ) -> BestObjects {
        let mut candidates = domains[bidder]
            .iter()
            .map(|&(object, base_cost)| PricedObject {
                object,
                base_cost,
                reduced_cost: scale * base_cost + prices[object],
            })
            .collect::<Vec<_>>();
        candidates.sort_unstable_by_key(|choice| (choice.reduced_cost, choice.object));
        BestObjects {
            best: candidates.first().copied(),
            second: candidates.get(1).copied(),
            examined: candidates.len() as u64,
        }
    }

    fn assignment_quality(
        domains: &[Vec<(usize, u64)>],
        assignment: &[Option<usize>],
    ) -> (usize, u128) {
        let unmatched = assignment.iter().filter(|object| object.is_none()).count();
        let base_cost = assignment
            .iter()
            .enumerate()
            .filter_map(|(bidder, object)| {
                object.map(|object| {
                    let base = domains[bidder]
                        .iter()
                        .find_map(|&(candidate, cost)| (candidate == object).then_some(cost))
                        .expect("auction selects a domain object");
                    u128::from(base)
                })
            })
            .sum();
        (unmatched, base_cost)
    }

    fn brute_force_quality(domains: &[Vec<(usize, u64)>], object_count: usize) -> (usize, u128) {
        fn visit(
            bidder: usize,
            domains: &[Vec<(usize, u64)>],
            used: &mut [bool],
            unmatched: usize,
            cost: u128,
            best: &mut (usize, u128),
        ) {
            if (unmatched, cost) >= *best {
                return;
            }
            if bidder == domains.len() {
                *best = (unmatched, cost);
                return;
            }
            visit(bidder + 1, domains, used, unmatched + 1, cost, best);
            for &(object, base) in &domains[bidder] {
                if used[object] {
                    continue;
                }
                used[object] = true;
                visit(
                    bidder + 1,
                    domains,
                    used,
                    unmatched,
                    cost + u128::from(base),
                    best,
                );
                used[object] = false;
            }
        }

        let mut best = (usize::MAX, u128::MAX);
        visit(0, domains, &mut vec![false; object_count], 0, 0, &mut best);
        best
    }

    fn check_against_brute_force(
        domains: &[Vec<(usize, u64)>],
        object_count: usize,
        max_base_cost: u64,
    ) {
        let result = assign_implicitly(
            domains.len(),
            object_count,
            max_base_cost,
            |bidder, prices, scale| Ok(explicit_best_two(domains, bidder, prices, scale)),
        )
        .unwrap();
        let mut used = vec![false; object_count];
        for &object in result.assignment.iter().flatten() {
            assert!(
                !used[object],
                "duplicate object: assignment={:?}",
                result.assignment
            );
            used[object] = true;
        }
        let actual = assignment_quality(domains, &result.assignment);
        let optimum = brute_force_quality(domains, object_count);
        assert_eq!(
            actual.0, optimum.0,
            "cardinality: domains={domains:?} assignment={:?}",
            result.assignment,
        );
        assert!(
            actual.1 <= optimum.1 + domains.len() as u128,
            "cost bound: domains={domains:?} assignment={:?} actual={actual:?} optimum={optimum:?}",
            result.assignment,
        );
    }

    #[test]
    fn flexible_bidder_moves_instead_of_spilling_constrained_bidder() {
        let domains = vec![vec![(0, 0), (1, 1), (2, 100)], vec![(0, 0), (2, 100)]];
        let result = assign_implicitly(2, 3, 100, |bidder, prices, scale| {
            Ok(explicit_best_two(&domains, bidder, prices, scale))
        })
        .unwrap();

        assert_eq!(result.assignment, [Some(1), Some(0)]);
        assert_eq!(result.stats.unmatched, 0);
    }

    #[test]
    fn restricted_graph_uses_real_solution_instead_of_false_no_bel() {
        let domains = vec![vec![(0, 0), (1, 1)], vec![(0, 0)]];
        let result = assign_implicitly(2, 2, 1, |bidder, prices, scale| {
            Ok(explicit_best_two(&domains, bidder, prices, scale))
        })
        .unwrap();

        assert_eq!(result.assignment, [Some(1), Some(0)]);
        assert_eq!(result.stats.unmatched, 0);
    }

    #[test]
    fn private_dummy_gap_does_not_hide_a_perfect_matching() {
        let domains = vec![vec![(1, 0), (2, 0)], vec![(0, 0), (1, 0)], vec![(0, 0)]];
        let result = assign_implicitly(3, 3, 0, |bidder, prices, scale| {
            Ok(explicit_best_two(&domains, bidder, prices, scale))
        })
        .unwrap();

        assert_eq!(result.assignment, [Some(2), Some(1), Some(0)]);
        assert_eq!(result.stats.unmatched, 0);
    }

    #[test]
    fn candidate_after_legacy_sixty_four_limit_remains_visible() {
        let mut domains = (0..65).map(|object| vec![(object, 0)]).collect::<Vec<_>>();
        domains.push((0..66).map(|object| (object, 0)).collect());
        let first = assign_implicitly(66, 66, 0, |bidder, prices, scale| {
            Ok(explicit_best_two(&domains, bidder, prices, scale))
        })
        .unwrap();
        let second = assign_implicitly(66, 66, 0, |bidder, prices, scale| {
            Ok(explicit_best_two(&domains, bidder, prices, scale))
        })
        .unwrap();

        assert_eq!(first, second);
        assert_eq!(first.stats.unmatched, 0);
        assert_eq!(first.assignment[65], Some(65));
    }

    #[test]
    fn private_dummy_reports_maximum_cardinality_deficiency() {
        let domains = vec![vec![(0, 0)], vec![(0, 0)]];
        let result = assign_implicitly(2, 1, 0, |bidder, prices, scale| {
            Ok(explicit_best_two(&domains, bidder, prices, scale))
        })
        .unwrap();

        assert_eq!(result.assignment.iter().flatten().count(), 1);
        assert_eq!(result.stats.unmatched, 1);
    }

    #[test]
    fn malformed_second_without_best_is_reported() {
        let error = assign_implicitly(1, 1, 0, |_, _, _| {
            Ok(BestObjects {
                best: None,
                second: Some(PricedObject {
                    object: 0,
                    base_cost: 0,
                    reduced_cost: 0,
                }),
                examined: 1,
            })
        })
        .unwrap_err();

        assert_eq!(error, super::AuctionError::SecondWithoutBest);
    }

    #[test]
    fn native_price_range_exhaustion_is_reported_before_bidding() {
        let error = assign_implicitly(1, 0, u64::MAX, |_, _, _| {
            unreachable!("dummy-cost range is validated before the first bid")
        })
        .unwrap_err();

        assert_eq!(error, super::AuctionError::ArithmeticOverflow);
    }

    #[test]
    fn implicit_auction_has_exact_cardinality_and_bounded_cost_on_small_graphs() {
        // Exhaust absent/cost-zero/cost-one edges through 3x3.
        for bidders in 1..=3 {
            for objects in 0..=3 {
                let edge_count = bidders * objects;
                let exponent = u32::try_from(edge_count).expect("small exhaustive exponent");
                let cases = 3_usize.pow(exponent);
                for mut code in 0..cases {
                    let mut domains = vec![Vec::new(); bidders];
                    for bidder_domain in &mut domains {
                        for object in 0..objects {
                            let edge = code % 3;
                            code /= 3;
                            if edge != 0 {
                                bidder_domain.push((object, (edge - 1) as u64));
                            }
                        }
                    }
                    check_against_brute_force(&domains, objects, 1);
                }
            }
        }

        // Exhaust every 4x0..4 domain mask with deterministic zero/one costs.
        for objects in 0..=4 {
            let edge_count = 4 * objects;
            for mask in 0_u64..(1_u64 << edge_count) {
                let mut domains = vec![Vec::new(); 4];
                for (bidder, bidder_domain) in domains.iter_mut().enumerate() {
                    for object in 0..objects {
                        let edge = bidder * objects + object;
                        if mask & (1_u64 << edge) != 0 {
                            bidder_domain.push((object, ((bidder + object) & 1) as u64));
                        }
                    }
                }
                check_against_brute_force(&domains, objects, 1);
            }
        }
    }
}
