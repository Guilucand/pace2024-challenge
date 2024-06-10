use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    heuristic_v4::utils::{EdgeType, OrderedEdgeSet},
    optim_max_flow::OptimMaxFlow,
    time_limit::TimeLimit,
    topological::TopologicalMapping,
};

/// This function tries to prove as many edges as it can using greedy flow updates
/// with partial rollback to try to improve the number of proven edges
pub fn heuristic_v4_step2(
    mapping: &TopologicalMapping,
    best_solution: &[usize],
    time_limit: &mut TimeLimit,
) -> (
    FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
    Vec<(usize, usize)>,
    OrderedEdgeSet,
) {
    let mut sorted_paid_edges = mapping.get_ordered_paid_edges(&best_solution);
    sorted_paid_edges.reverse();
    let sorted_paid_edges = sorted_paid_edges;

    let mut nodes_positions = vec![0; mapping.n1];
    for (idx, &node) in best_solution.iter().enumerate() {
        nodes_positions[node] = idx;
    }

    let excluded_edges_set = sorted_paid_edges
        .iter()
        .map(|(a, b, _)| (*a.min(b), *a.max(b)))
        .collect::<FxHashSet<_>>();

    let mut directed_graph = OrderedEdgeSet::new(mapping.n1);
    for node in 0..mapping.n1 {
        for (target, weight) in mapping.min_adj_list[node].iter() {
            let a = node.min(*target as usize);
            let b = node.max(*target as usize);

            if !excluded_edges_set.contains(&(a, b)) {
                directed_graph.edges[node]
                    .insert(*target as u32, EdgeType::Saved { cost: *weight });
            }
        }
    }
    // Make it read only
    let directed_graph = directed_graph;

    let mut associated_flowgraphs = FxHashMap::default();

    let mut solution_stack = vec![];

    let mut remaining_edges = sorted_paid_edges.clone();

    let mut optimized_flow = OptimMaxFlow::new(&directed_graph);
    let mut cache = optimized_flow.create_cache();

    let max_weight = remaining_edges
        .iter()
        .map(|(_, _, w)| *w)
        .max()
        .unwrap_or(0);

    let mut persistent_flows = vec![];

    const CHOOSE_BY_FLOW: bool = false;

    // find_needed_flows(&directed_graph, &sorted_paid_edges);

    if CHOOSE_BY_FLOW {
        for &(tsource, tsink, nw) in &remaining_edges {
            let rem_flow = optimized_flow.compute_max_flow(
                tsource as u32,
                tsink as u32,
                &mut cache,
                Some(max_weight * 10),
                true,
                None,
            ) as i64;

            persistent_flows.push(rem_flow - (nw as i64));
        }
    }

    let mut not_proven_edges = vec![];

    while remaining_edges.len() > 0 {
        let chosen_edge = if CHOOSE_BY_FLOW {
            let mut flows_order: Vec<_> = (0..persistent_flows.len()).collect();
            flows_order.sort_by_key(|&idx| persistent_flows[idx]);

            let mut minimum_recomputed = i64::MAX;

            for flow_idx in flows_order {
                let flow = &mut persistent_flows[flow_idx];
                let &(source, sink, weight) = &remaining_edges[flow_idx];
                if *flow < minimum_recomputed {
                    let rem_flow = optimized_flow.compute_max_flow(
                        source as u32,
                        sink as u32,
                        &mut cache,
                        Some(max_weight * 10),
                        true,
                        None,
                    ) as i64;
                    *flow = rem_flow - weight as i64;
                    minimum_recomputed = minimum_recomputed.min(*flow);
                } else {
                    break;
                }
            }

            let edge_idx = remaining_edges
                .iter()
                .enumerate()
                .min_by_key(|(idx, _)| persistent_flows[*idx])
                .unwrap()
                .0;

            let chosen_edge = remaining_edges.swap_remove(edge_idx);
            persistent_flows.swap_remove(edge_idx);

            if persistent_flows.len() > 0 {
                persistent_flows
                    .iter_mut()
                    .zip(remaining_edges.iter())
                    .for_each(|(flow, _)| {
                        *flow -= chosen_edge.2 as i64;
                    });

                let mut sorted_flows = persistent_flows.clone();
                sorted_flows.sort_unstable();

                let mut counts = vec![(1, sorted_flows[0])];
                for (idx, &flow) in sorted_flows.iter().enumerate().skip(1) {
                    if flow != sorted_flows[idx - 1] {
                        counts.push((1, flow));
                    } else {
                        counts.last_mut().unwrap().0 += 1;
                    }
                }

                println!(
                    "Computed new order with min flows: {:?} (sol len: {})",
                    &counts[0..counts.len().min(10)],
                    solution_stack.len()
                );
            }
            chosen_edge
        } else {
            remaining_edges.pop().unwrap()
        };

        let (source, sink, weight) = chosen_edge;

        let sink_pos = nodes_positions[sink];
        let source_pos = nodes_positions[source];

        let flow = optimized_flow.compute_max_flow(
            source as u32,
            sink as u32,
            &mut cache,
            Some(weight),
            true,
            Some(&best_solution[source_pos..=sink_pos]),
        ) as u64;

        if flow >= weight {
            solution_stack.push(chosen_edge);
            let flow_graph = optimized_flow.get_flow_graph(&mut cache, source as u32);

            associated_flowgraphs.insert((source, sink), flow_graph.to_vec());

            optimized_flow.remove_flow(flow_graph);
        } else {
            // Do the analysis only if we have enough time
            if time_limit.can_progress() {
                // Remove edge
                log::info!(
                    "Cannot find flow for edge {} -> {} with weight {} with {}/{} proven edges",
                    source,
                    sink,
                    weight,
                    solution_stack.len(),
                    sorted_paid_edges.len(),
                );

                let mut still_taken_edges = vec![];
                let mut removed_edges = vec![];
                let mut found_placement = false;

                let rollback_backup_flow = optimized_flow.clone();
                let mut flow_compute_instance = optimized_flow.clone();
                let rollback_backup_solution = solution_stack.clone();

                while let Some((placed_source, placed_sink, placed_weight)) = solution_stack.pop() {
                    let placed_source_pos = nodes_positions[placed_source];
                    let placed_sink_pos = nodes_positions[placed_sink];

                    let edge_flow = flow_compute_instance.compute_max_flow(
                        placed_source as u32,
                        placed_sink as u32,
                        &mut cache,
                        Some(weight),
                        true,
                        Some(&best_solution[placed_source_pos..=placed_sink_pos]),
                    ) as i64;

                    if edge_flow >= (weight as i64) {
                        optimized_flow.add_back_flow(
                            &associated_flowgraphs
                                .get(&(placed_source, placed_sink))
                                .unwrap(),
                        );
                        removed_edges.push((placed_source, placed_sink, placed_weight));

                        let sink_pos = nodes_positions[sink];
                        let source_pos = nodes_positions[source];

                        // Check if the node can be placed now
                        let new_flow = optimized_flow.compute_max_flow(
                            source as u32,
                            sink as u32,
                            &mut cache,
                            Some(weight),
                            false,
                            Some(&best_solution[source_pos..=sink_pos]),
                        ) as u64;
                        if new_flow >= weight {
                            found_placement = true;
                            break;
                        }
                    } else {
                        still_taken_edges.push((placed_source, placed_sink, placed_weight));
                    }
                }

                if found_placement {
                    // Place again in the stack the elements that were not touched
                    while let Some(placed) = still_taken_edges.pop() {
                        solution_stack.push(placed);
                    }

                    for edge in removed_edges.into_iter().rev() {
                        associated_flowgraphs.remove(&(edge.0, edge.1)).unwrap();
                        remaining_edges.push(edge);
                        if CHOOSE_BY_FLOW {
                            persistent_flows.push(i64::MIN);
                        }
                    }

                    // Add back the current element, as we found a way to place it in the proof
                    remaining_edges.push(chosen_edge);

                    if CHOOSE_BY_FLOW {
                        persistent_flows.push(i64::MIN);
                    }
                } else {
                    // Restore the backup
                    optimized_flow = rollback_backup_flow;
                    solution_stack = rollback_backup_solution;

                    not_proven_edges.push((chosen_edge.0, chosen_edge.1));
                    log::info!("Cannot remove any more elements, guessing the edge as optimal!");
                }
            }
        }
    }
    (associated_flowgraphs, not_proven_edges, directed_graph)
}
