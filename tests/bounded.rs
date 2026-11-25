use navigate::graph_algos::bounded::*;
fn limits() -> SearchLimits {
    SearchLimits {
        max_nodes: 100,
        max_frontier: 100,
        max_expansions: 100,
        max_heap_pops: 100,
        max_neighbor_polls: 1000,
    }
}
fn found(report: SearchReport<usize>) -> (Vec<usize>, u64) {
    match report.outcome {
        SearchOutcome::Found { path, cost } => (path, cost),
        other => panic!("{other:?}"),
    }
}

#[test]
fn exhaustive_finite_graphs_match_independent_bellman_ford() {
    // Every directed graph on four vertices; nonnegative weights include zero cycles.
    for mask in 0..4096_u32 {
        let mut graph = vec![Vec::new(); 4];
        let mut bit = 0;
        for (a, adjacent) in graph.iter_mut().enumerate() {
            for b in 0..4 {
                if a == b {
                    continue;
                }
                if mask & (1 << bit) != 0 {
                    adjacent.push((b, ((a * 7 + b * 3) % 5) as u64));
                }
                bit += 1;
            }
        }
        let mut distances = [u64::MAX; 4];
        distances[0] = 0;
        for _ in 0..3 {
            for (a, adjacent) in graph.iter().enumerate() {
                for &(b, cost) in adjacent {
                    if distances[a] != u64::MAX {
                        distances[b] = distances[b].min(distances[a] + cost);
                    }
                }
            }
        }
        for report in [
            dijkstra(0, |n| graph[*n].clone(), |n| *n == 3, limits(), || false),
            a_star(
                0,
                |n| graph[*n].clone(),
                |_| 0,
                |n| *n == 3,
                limits(),
                || false,
            ),
        ] {
            match report.outcome {
                SearchOutcome::Found { path, cost } => {
                    assert_eq!(cost, distances[3], "mask={mask}");
                    assert_eq!(path.first(), Some(&0));
                    assert_eq!(path.last(), Some(&3));
                    let measured: u64 = path
                        .windows(2)
                        .map(|pair| {
                            graph[pair[0]]
                                .iter()
                                .find(|(n, _)| *n == pair[1])
                                .unwrap()
                                .1
                        })
                        .sum();
                    assert_eq!(measured, cost);
                }
                SearchOutcome::NoPath => assert_eq!(distances[3], u64::MAX),
                other => panic!("mask={mask}: {other:?}"),
            }
        }
    }
}

#[test]
fn inconsistent_admissible_heuristics_reopen_nodes() {
    let graph = [
        vec![(1, 3), (2, 1)],
        vec![(3, 2)],
        vec![(1, 1), (3, 100)],
        vec![],
    ];
    let h = [4, 0, 3, 0];
    let result = a_star(
        0,
        |n| graph[*n].clone(),
        |n| h[*n],
        |n| *n == 3,
        limits(),
        || false,
    );
    assert_eq!(result.stats.expanded, 4);
    assert_eq!(found(result), (vec![0, 2, 1, 3], 4));
}

#[test]
fn equal_priorities_are_fifo_and_reproducible_for_ordered_neighbors() {
    let graph = [vec![(1, 1), (2, 1)], vec![(3, 1)], vec![(3, 1)], vec![]];
    for _ in 0..100 {
        assert_eq!(
            found(a_star(
                0,
                |n| graph[*n].clone(),
                |_| 0,
                |n| *n == 3,
                limits(),
                || false
            ))
            .0,
            vec![0, 1, 3]
        );
    }
    let reverse = [vec![(2, 1), (1, 1)], vec![(3, 1)], vec![(3, 1)], vec![]];
    assert_eq!(
        found(dijkstra(
            0,
            |n| reverse[*n].clone(),
            |n| *n == 3,
            limits(),
            || false
        ))
        .0,
        vec![0, 2, 3]
    );
}

#[test]
fn stale_entries_consume_pop_budget_without_reexpansion() {
    let graph = [vec![(1, 10), (2, 1)], vec![], vec![(1, 1)]];
    let result = dijkstra(0, |n| graph[*n].clone(), |_| false, limits(), || false);
    assert_eq!(result.outcome, SearchOutcome::NoPath);
    assert_eq!(
        (
            result.stats.discovered,
            result.stats.expanded,
            result.stats.heap_pops
        ),
        (3, 3, 4)
    );
    let bounded = SearchLimits {
        max_heap_pops: 3,
        ..limits()
    };
    assert_eq!(
        dijkstra(0, |n| graph[*n].clone(), |_| false, bounded, || false).outcome,
        SearchOutcome::BudgetExhausted(SearchBudget::HeapPops)
    );
}

#[test]
fn every_budget_and_start_goal_boundary_is_explicit() {
    let cases = [
        (
            SearchLimits {
                max_nodes: 0,
                ..limits()
            },
            SearchBudget::Nodes,
        ),
        (
            SearchLimits {
                max_frontier: 0,
                ..limits()
            },
            SearchBudget::Frontier,
        ),
        (
            SearchLimits {
                max_expansions: 0,
                ..limits()
            },
            SearchBudget::Expansions,
        ),
        (
            SearchLimits {
                max_heap_pops: 0,
                ..limits()
            },
            SearchBudget::HeapPops,
        ),
        (
            SearchLimits {
                max_neighbor_polls: 0,
                ..limits()
            },
            SearchBudget::NeighborPolls,
        ),
        (
            SearchLimits {
                max_nodes: 1,
                ..limits()
            },
            SearchBudget::Nodes,
        ),
        (
            SearchLimits {
                max_frontier: 1,
                ..limits()
            },
            SearchBudget::Frontier,
        ),
    ];
    for (limits, budget) in cases {
        let result = dijkstra(0, |_| [(1, 1), (2, 1)], |_| false, limits, || false);
        assert_eq!(result.outcome, SearchOutcome::BudgetExhausted(budget));
        assert!(
            result.stats.discovered <= limits.max_nodes
                && result.stats.peak_frontier <= limits.max_frontier
        );
        assert!(
            result.stats.expanded <= limits.max_expansions
                && result.stats.heap_pops <= limits.max_heap_pops
                && result.stats.neighbor_polls <= limits.max_neighbor_polls
        );
    }
    let zero_work = SearchLimits {
        max_expansions: 0,
        max_neighbor_polls: 0,
        ..limits()
    };
    assert_eq!(
        found(dijkstra(
            7,
            |_| -> Vec<(usize, u64)> { panic!("start is goal") },
            |n| *n == 7,
            zero_work,
            || false
        )),
        (vec![7], 0)
    );
}

#[test]
fn infinite_iterators_and_cooperative_cancellation_terminate() {
    let bounded = SearchLimits {
        max_neighbor_polls: 17,
        ..limits()
    };
    let report = dijkstra(
        0,
        |_| std::iter::repeat((1, 0)),
        |_| false,
        bounded,
        || false,
    );
    assert_eq!(
        report.outcome,
        SearchOutcome::BudgetExhausted(SearchBudget::NeighborPolls)
    );
    assert_eq!(report.stats.neighbor_polls, 17);
    assert_eq!(
        dijkstra(0, |_| std::iter::empty(), |_| false, limits(), || true).outcome,
        SearchOutcome::Cancelled
    );
    let mut checks = 0;
    let report = dijkstra(
        0,
        |_| std::iter::repeat((1, 0)),
        |_| false,
        limits(),
        || {
            checks += 1;
            checks > 12
        },
    );
    assert_eq!(report.outcome, SearchOutcome::Cancelled);
    assert!(report.stats.neighbor_polls < 17);
    let stop = std::cell::Cell::new(false);
    let report = dijkstra(
        0,
        |n| [(*n + 1, 1)],
        |n| {
            if *n == 3 {
                stop.set(true);
                true
            } else {
                false
            }
        },
        limits(),
        || stop.get(),
    );
    assert_eq!(report.outcome, SearchOutcome::Cancelled); // Before returning/reconstructing a stale result.
}

#[test]
fn costs_and_priorities_are_checked_at_the_numeric_boundary() {
    assert_eq!(
        found(dijkstra(
            0,
            |_| [(1, u64::MAX)],
            |n| *n == 1,
            limits(),
            || false
        ))
        .1,
        u64::MAX
    );
    assert_eq!(
        dijkstra(
            0,
            |n| [(*n + 1, if *n == 0 { u64::MAX } else { 1 })],
            |n| *n == 2,
            limits(),
            || false
        )
        .outcome,
        SearchOutcome::CostOverflow
    );
    assert_eq!(
        a_star(
            0,
            |_| [(1, u64::MAX)],
            |_| 1,
            |n| *n == 1,
            limits(),
            || false
        )
        .outcome,
        SearchOutcome::CostOverflow
    );
    assert_eq!(
        a_star(
            0,
            |_| std::iter::empty(),
            |_| 1,
            |_| true,
            limits(),
            || false
        )
        .outcome,
        SearchOutcome::InvalidHeuristic
    );
}

#[test]
fn cancellation_in_goal_check_does_not_count_an_expansion() {
    let stop = std::cell::Cell::new(false);
    for max_expansions in [0, 1] {
        stop.set(false);
        let result = dijkstra(
            0,
            |_| -> Vec<(usize, u64)> { panic!("successors must not be requested") },
            |_| {
                stop.set(true);
                false
            },
            SearchLimits {
                max_expansions,
                ..limits()
            },
            || stop.get(),
        );
        assert_eq!(result.outcome, SearchOutcome::Cancelled);
        assert_eq!(result.stats.expanded, 0);
    }
}
