use rustc_hash::FxHashMap;

use crate::{
    connected::find_ordered_connected_components,
    heuristic_v4::utils::{reorder_optimal_fixed, EdgeType, OrderedEdgeSet},
    max_flow_basic::SinkType,
    optim_max_flow::OptimMaxFlow,
    time_limit::TimeLimit,
    topological::TopologicalMapping,
};

/// This step takes in input an initial ordering (usually based on greedy costs ordering) and outputs an improved
/// solution based on min-cut more convenient splittings of the current graph.
/// At the end no 'early (ex. first found)' min cut can improve the solution
pub fn heuristic_v4_step1(
    mapping: &TopologicalMapping,
    initial_order: &[usize],
    timelimit: &mut TimeLimit,
    fast: bool,
) -> Vec<usize> {
    let mut best_order = initial_order.to_vec();

    let mut last_best_prefix = vec![];
    let start_cost = mapping.compute_cost(&best_order);
    let mut predicted_cost = start_cost;
    struct IterationParams {
        do_not_go_back: bool,
        full_optim: bool,
        break_on_unchanged: bool,
    }

    let mut iter_params = vec![
        // First step, do not try to consider alternative paths, do not go back
        IterationParams {
            do_not_go_back: true,
            full_optim: false,
            break_on_unchanged: false,
        },
    ];
    if !fast {
        iter_params.extend(
            [
                // Second step, do not try to consider alternative paths, go back
                IterationParams {
                    do_not_go_back: false,
                    full_optim: false,
                    break_on_unchanged: false,
                },
                // Third step, try to consider alternative paths, do not go back
                IterationParams {
                    do_not_go_back: true,
                    full_optim: true,
                    break_on_unchanged: true,
                },
                // Fourth step, try to consider alternative paths, go back
                IterationParams {
                    do_not_go_back: false,
                    full_optim: true,
                    break_on_unchanged: true,
                },
            ]
            .into_iter(),
        )
    };

    let mut iter_count = 0;
    let mut restart_count = 0;

    'restart_optim: loop {
        let mut rev_candidate_solution = vec![];
        let mut candidate_solution_has_element = vec![false; mapping.n1];
        let mut candidate_solution_edges_set = OrderedEdgeSet::new(initial_order.len());
        let mut nodes_ordering = FxHashMap::default();

        let params = &iter_params[iter_count];

        restart_count += 1;

        let mut is_the_same = true;
        let mut paid_edges_weight = 0;
        for &element in best_order.iter().rev() {
            let was_the_same = is_the_same;
            is_the_same &= last_best_prefix
                .pop()
                .map(|last| params.do_not_go_back || last == element)
                .unwrap_or(false);
            if was_the_same && !is_the_same {
                log::info!("Resuming from size: {}", rev_candidate_solution.len());
            }

            if !is_the_same && rev_candidate_solution.len() % 50 == 0 {
                log::info!("Checking position: {}", rev_candidate_solution.len());
            }

            nodes_ordering.insert(element, rev_candidate_solution.len());
            rev_candidate_solution.push(element);
            candidate_solution_has_element[element] = true;

            let mut current_paid_edges = vec![];
            for (node, weight) in mapping.adj_list[element].iter() {
                if candidate_solution_has_element[*node as usize] {
                    candidate_solution_edges_set.edges[element]
                        .insert(*node, EdgeType::Paid { cost: *weight });
                    current_paid_edges.push((*weight, (element, *node)));
                    paid_edges_weight += *weight as u64;
                }
            }

            for (node, weight) in mapping.min_adj_list[element].iter() {
                if candidate_solution_has_element[*node as usize] {
                    candidate_solution_edges_set.edges[element]
                        .insert(*node, EdgeType::Saved { cost: *weight });
                }
            }

            if !is_the_same {
                let mut optim_flow = OptimMaxFlow::new(&candidate_solution_edges_set);
                let mut cache = optim_flow.create_cache();

                for &(weight, (source, target)) in &current_paid_edges {
                    let source_pos = *nodes_ordering.get(&source).unwrap();
                    let sink_pos = *nodes_ordering.get(&(target as usize)).unwrap();

                    let (max_flow_adjusted, max_flow, min_cut, removed_edges) = optim_flow
                        .compute_max_flow_directed(
                            &candidate_solution_edges_set,
                            &mut cache,
                            source,
                            SinkType::Single(target as usize),
                            true,
                            Some(if params.full_optim {
                                paid_edges_weight as u64
                            } else {
                                weight as u64 - 1
                            }),
                            Some(&rev_candidate_solution[sink_pos..=source_pos]),
                        );

                    if max_flow_adjusted < 0 {
                        let min_cut = min_cut.unwrap();

                        log::info!(
                            "Actual cost reduction: {} (total flow: {}, weight: {}) from {} to {}",
                            max_flow_adjusted,
                            max_flow,
                            weight,
                            element,
                            target
                        );

                        for (start, end) in removed_edges {
                            let removed_weight = candidate_solution_edges_set.edges[start as usize]
                                .remove(&(end as u32))
                                .unwrap();
                            candidate_solution_edges_set.edges[end]
                                .insert(start as u32, EdgeType::Saved { cost: weight });

                            let weight = match removed_weight {
                                EdgeType::Paid { cost } => cost as u64,
                                EdgeType::Saved { .. } => unreachable!(),
                            };

                            predicted_cost -= weight as u64;
                        }

                        for &(nodea, nodeb) in min_cut.iter() {
                            let added_weight = candidate_solution_edges_set.edges[nodea]
                                .remove(&(nodeb as u32))
                                .unwrap();
                            let cost = match added_weight {
                                EdgeType::Paid { .. } => unreachable!(),
                                EdgeType::Saved { cost } => cost as u64,
                            };
                            predicted_cost += cost;
                            candidate_solution_edges_set.edges[nodeb]
                                .insert(nodea as u32, EdgeType::Paid { cost: cost as u32 });
                            // log::info!("Added cost: {} ({} to {})", cost, nodea, nodeb);
                        }

                        let new_order =
                            find_ordered_connected_components(&candidate_solution_edges_set, None);
                        let mut new_solution = vec![];
                        for component in new_order {
                            // assert_eq!(component.0.len(), 1);
                            if candidate_solution_has_element[component.0[0]] {
                                new_solution.extend(component.0);
                            }
                        }

                        last_best_prefix.clear();
                        last_best_prefix.extend_from_slice(
                            &best_order[best_order.len() - new_solution.len()..],
                        );

                        best_order.truncate(best_order.len() - new_solution.len());
                        best_order.extend(new_solution);

                        let reordered = reorder_optimal_fixed(mapping, &best_order);
                        best_order = reordered;
                        log::info!(
                            "new cost: {}  start: {} fixed last: {}",
                            predicted_cost,
                            start_cost,
                            last_best_prefix.len()
                        );

                        if !timelimit.can_progress() {
                            break 'restart_optim;
                        }

                        continue 'restart_optim;
                    }
                }
            }
        }

        // No changes in this run, we can exit
        if restart_count <= 1 && params.break_on_unchanged {
            break;
        }
        restart_count = 0;

        iter_count += 1;
        last_best_prefix.clear();

        if iter_count >= iter_params.len() {
            break;
        }
    }

    let mut nodes = best_order.clone();
    nodes.sort_unstable();
    // for (&a, &b) in nodes.iter().zip(nodes.iter().skip(1)) {
    //     assert_eq!(a, b - 1);
    // }
    let sol_cost = mapping.compute_cost(&best_order);

    log::info!(
        "Heuristic step 1 DONE: current cost {} vs previous {}!",
        sol_cost,
        start_cost
    );

    best_order
}
