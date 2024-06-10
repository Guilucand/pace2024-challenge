use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    heuristic_v4::{
        step3::{add_flow_graph_to_capacities, build_capacities, find_alternative_path},
        utils::{check_flows_correctness, EdgeType},
    },
    max_flow_basic::FlowGraph,
    optim_max_flow::OptimMaxFlow,
    topological::TopologicalMapping,
};

use super::utils::OrderedEdgeSet;

pub(crate) fn build_full_dag(
    mapping: &TopologicalMapping,
    paid_associated_flowgraphs: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> OrderedEdgeSet {
    let mut graph = OrderedEdgeSet::new(mapping.n1);

    for node in 0..mapping.n1 {
        for (target, weight) in mapping.min_adj_list[node].iter() {
            if !paid_associated_flowgraphs.contains_key(&(*target as usize, node)) {
                graph.edges[node].insert(*target as u32, EdgeType::Saved { cost: *weight });
            }
            assert!(!paid_associated_flowgraphs.contains_key(&(node, *target as usize)));
        }
    }
    graph
}

fn add_flowgraphs(dest: &mut [Vec<(u32, u32)>], src: &[Vec<(u32, u32)>], subtract: bool) {
    for (dest, src) in dest.iter_mut().zip(src.iter()) {
        for (edge, weight) in src.iter().copied() {
            let dest_edge = dest.iter_mut().position(|(e, _)| *e == edge);
            if let Some(dest_edge_idx) = dest_edge {
                let dest_edge = &mut dest[dest_edge_idx];
                if subtract {
                    dest_edge.1 -= weight;
                    if dest_edge.1 == 0 {
                        dest.remove(dest_edge_idx);
                    }
                } else {
                    dest_edge.1 += weight;
                }
            } else if !subtract {
                dest.push((edge, weight));
            } else {
                log::error!("Subtract edge not found in destination flowgraph!");
            }
        }
    }
}

fn remove_flow(
    full_graph: &OptimMaxFlow,
    capacities: &mut [u32],
    capacities_details: &mut [FxHashMap<(usize, usize), u32>],
    oc_edge_source: u32,
    oc_edge_dest: u32,
    cap_detail_edge: (usize, usize),
    input_flow: &mut [Vec<(u32, u32)>],
    mut removed_flow_out: Option<&mut [Vec<(u32, u32)>]>,
) {
    let oc_edge_index = full_graph
        .get_edge_index(oc_edge_source, oc_edge_dest)
        .unwrap();
    let cap_to_remove = capacities_details[oc_edge_index as usize]
        .get(&cap_detail_edge)
        .copied()
        .unwrap_or(0);

    if cap_to_remove > 0 {
        let mut flow_graph = OptimMaxFlow::new(input_flow);
        let mut cache = flow_graph.create_cache();

        println!(
            "Removing {} capacity from {} => {} with associated edge: {} => {}",
            cap_to_remove, oc_edge_source, oc_edge_dest, cap_detail_edge.0, cap_detail_edge.1
        );
        println!(
            "Input graph edges: {:?}",
            input_flow
                .iter()
                .enumerate()
                .map(|(i, e)| e.iter().map(move |(e, w)| (i, *e, *w)))
                .flatten()
                .collect::<Vec<_>>()
        );

        if oc_edge_dest != (cap_detail_edge.1 as u32) {
            let after_flow = flow_graph.compute_max_flow(
                oc_edge_dest,
                cap_detail_edge.1 as u32,
                &mut cache,
                Some(cap_to_remove as u64),
                true,
                None,
            );
            assert_eq!(after_flow, cap_to_remove);
            let after_flow_graph = flow_graph.get_flow_graph(&mut cache, oc_edge_dest);
            add_flowgraphs(input_flow, &after_flow_graph, true);

            println!(
                "After flow graph: {:?}",
                after_flow_graph
                    .iter()
                    .enumerate()
                    .map(|(i, e)| e.iter().map(move |(e, w)| (i, *e, *w)))
                    .flatten()
                    .collect::<Vec<_>>()
            );

            if let Some(out_flow) = removed_flow_out.as_mut() {
                add_flowgraphs(out_flow, &after_flow_graph, false);
            }
            add_flow_graph_to_capacities(
                full_graph,
                capacities,
                capacities_details,
                &after_flow_graph,
                cap_detail_edge,
                true,
                false,
            );
        }

        if oc_edge_source != (cap_detail_edge.0 as u32) {
            let before_flow = flow_graph.compute_max_flow(
                cap_detail_edge.0 as u32,
                oc_edge_source,
                &mut cache,
                Some(cap_to_remove as u64),
                true,
                None,
            );
            assert_eq!(before_flow, cap_to_remove);

            let before_flow_graph = flow_graph.get_flow_graph(&mut cache, cap_detail_edge.0 as u32);
            add_flowgraphs(input_flow, &before_flow_graph, true);
            if let Some(out_flow) = removed_flow_out.as_mut() {
                add_flowgraphs(out_flow, &before_flow_graph, false);
            }
            add_flow_graph_to_capacities(
                full_graph,
                capacities,
                capacities_details,
                before_flow_graph,
                cap_detail_edge,
                true,
                false,
            );

            println!(
                "Before flow graph: {:?}",
                before_flow_graph
                    .iter()
                    .enumerate()
                    .map(|(i, e)| e.iter().map(move |(e, w)| (i, *e, *w)))
                    .flatten()
                    .collect::<Vec<_>>()
            );
        }

        // Remove the capacity from the current edge
        capacities[oc_edge_index as usize] -= cap_to_remove;
        capacities_details[oc_edge_index as usize].remove(&cap_detail_edge);
        let input_flow_oc = &mut input_flow[oc_edge_source as usize]
            .iter_mut()
            .find(|(e, _)| *e == oc_edge_dest)
            .unwrap()
            .1;

        *input_flow_oc -= cap_to_remove;

        if *input_flow_oc == 0 {
            input_flow[oc_edge_source as usize].retain(|(e, _)| *e != oc_edge_dest);
        }

        if let Some(out_flow) = removed_flow_out.as_mut() {
            out_flow[cap_detail_edge.1 as usize].push((cap_detail_edge.0 as u32, cap_to_remove));
        }
        println!(
            "Missing edge: {} => {} with weight {}",
            cap_detail_edge.1, cap_detail_edge.0, cap_to_remove
        );
    }
}

/// TODO: Replace/augment step 4 with an alternate step
/// 1) Find cuts that should have cost <= 0 for each paying edge
/// 2) Exclude (or penalize) edges that have extra flow and cannot pay 0
/// 3) For each overcapacity, try to check if all the edges passing in that edge have a way to obtain a real cost <= 0:
///     - Fix the edge with overcapacity
///     - For each edge in the cut that pays more than expected, try to find a way to move other cuts towards that edge to lower the paid cost,
///         without increasing the cost for the other paying edges
///     - If no configuration is possible,

/// Updates the flowgraphs to minimize all the overcapacities, possibly changing the set of chosen edges
pub fn heuristic_v4_step4(
    mapping: &TopologicalMapping,
    paid_associated_flowgraphs: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>> {
    let mut paid_associated_flowgraphs = paid_associated_flowgraphs.clone();
    while heuristic_v4_step4_internal(mapping, &mut paid_associated_flowgraphs) {
        // Continue
    }
    paid_associated_flowgraphs
}

fn find_improvement_from_edge(
    mapping: &TopologicalMapping,
    graph: &OrderedEdgeSet,
    full_graph: &OptimMaxFlow,
    capacities: &mut [u32],
    capacities_details: &mut [FxHashMap<(usize, usize), u32>],
    all_associated_flowgraphs: &mut FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
    s: usize,
    d: usize,
    _allow_flow_removal: bool,
) -> bool {
    const MAX_ROLLBACKS: usize = 70;

    let mut steps: Vec<((usize, usize), Vec<(usize, usize)>, Vec<(u32, u32)>)> = vec![];
    steps.push(((s, d), vec![], vec![]));

    let mut rollbacks_count = 0;

    'rollback: loop {
        let mut residual_graph = OptimMaxFlow::new(graph);

        let mut new_paid_edges = FxHashSet::default();
        let mut new_paid_edges_indices = FxHashSet::default();
        let mut new_saved_edges = FxHashSet::default();

        let mut saved_to_be_paid = vec![];

        let last_step = steps.last().unwrap();

        let mut paid_to_be_saved_set = FxHashSet::default();
        for step in &steps {
            let cap_details = capacities_details[full_graph
                .get_edge_index(step.0 .0 as u32, step.0 .1 as u32)
                .unwrap() as usize]
                .clone();
            for (edge, _) in cap_details {
                paid_to_be_saved_set.insert(edge);
            }
        }

        // Restore the paid-to-be-saved edges
        let mut paid_to_be_saved: Vec<_> = last_step
            .1
            .iter()
            .copied()
            .filter(|edge| paid_to_be_saved_set.contains(edge))
            .collect();
        new_saved_edges.extend(paid_to_be_saved.iter().copied());

        // Restore the saved-to-be-paid edges
        for step in &steps {
            saved_to_be_paid.push((step.0 .0, step.0 .1));
            new_paid_edges.insert((step.0 .0, step.0 .1));
            new_paid_edges_indices.insert(
                full_graph
                    .get_edge_index(step.0 .0 as u32, step.0 .1 as u32)
                    .unwrap(),
            );
            residual_graph.remove_edge(step.0 .0 as u32, step.0 .1 as u32);

            for path in step.2.iter() {
                residual_graph.saturating_decrease_capacity(
                    residual_graph.get_edge_index(path.0, path.1).unwrap(),
                    1,
                );
            }
        }

        let last_removed_idx = full_graph
            .get_edge_index(last_step.0 .0 as u32, last_step.0 .1 as u32)
            .unwrap();
        for cap_detail in capacities_details[last_removed_idx as usize].iter() {
            let (&(source, dest), &_weight) = cap_detail;
            paid_to_be_saved.push((source, dest));
            new_saved_edges.insert((source, dest));
        }

        // println!("STBP: {:?}, PTBS: {:?}", saved_to_be_paid, paid_to_be_saved);

        let mut new_cache = residual_graph.create_cache();

        while let Some((source, dest)) = paid_to_be_saved.pop() {
            // let flow = residual_graph.compute_max_flow(
            //     source as u32,
            //     dest as u32,
            //     &mut new_cache,
            //     None,
            //     true,
            //     None,
            // );

            // println!("Flow from {} to {} is {}", source, dest, flow);

            let path =
                residual_graph.find_single_path(source as u32, dest as u32, &mut new_cache, None);

            if let Some(path) = path {
                let mut path_taken: Vec<(u32, u32)> = vec![];

                let mut found_edge = false;
                'found_edge_lowering: for edge in path.iter().copied() {
                    // println!("Edge: {} -> {}", edge.0, edge.1);
                    let edge_idx = residual_graph.get_edge_index(edge.0, edge.1).unwrap();
                    let used_capacity = capacities[edge_idx as usize];
                    let avail_capacity = residual_graph.get_edge_capacity(edge_idx);

                    // TODO: Skip previous path elements to ensure algorithm progress even if previous nodes are
                    // blocked when no path flow is subtracted from the graph

                    if used_capacity >= avail_capacity {
                        // Try to lower the capacity as much as it can be lowered
                        let cd = capacities_details[edge_idx as usize].clone();
                        for cap_detail in cd {
                            while find_alternative_path(
                                &residual_graph,
                                capacities,
                                capacities_details,
                                0,
                                all_associated_flowgraphs,
                                edge.0,
                                edge.1,
                                cap_detail.0,
                                Some(&new_paid_edges_indices),
                            ) {
                                if capacities[edge_idx as usize] < avail_capacity {
                                    path_taken.push((edge.0, edge.1));
                                    continue 'found_edge_lowering;
                                }
                            }
                        }

                        // println!("Found edge to remove: {} -> {}", edge.0, edge.1);

                        saved_to_be_paid.push((edge.0 as usize, edge.1 as usize));
                        residual_graph.remove_edge(edge.0, edge.1);

                        // TODO: Optionally remove all the flows going trough this edge

                        new_paid_edges.insert((edge.0 as usize, edge.1 as usize));
                        new_paid_edges_indices.insert(edge_idx);

                        // Add back the paid edge
                        paid_to_be_saved.push((source, dest));

                        for edge in &path_taken {
                            let edge_idx = residual_graph.get_edge_index(edge.0, edge.1).unwrap();
                            residual_graph.saturating_decrease_capacity(edge_idx, 1);
                        }

                        // Add the step before the paid-to-be-saved update, since it will be done again on rollback initialization
                        steps.push((
                            (edge.0 as usize, edge.1 as usize),
                            paid_to_be_saved.clone(),
                            path_taken,
                        ));

                        for cap_detail in capacities_details[edge_idx as usize].iter() {
                            let (&(source, dest), &_weight) = cap_detail;
                            paid_to_be_saved.push((source, dest));
                            new_saved_edges.insert((source, dest));
                        }

                        found_edge = true;
                        break;
                    }

                    path_taken.push((edge.0, edge.1));
                }

                if !found_edge {
                    // println!(
                    //     "No edge found to remove for ({}, {}) => Path: {:?}!",
                    //     source, dest, path
                    // );

                    let Some(first_step_with_this_edge) = steps.iter().position(|step| {
                        let step_edge_idx = full_graph
                            .get_edge_index(step.0 .0 as u32, step.0 .1 as u32)
                            .unwrap();
                        capacities_details[step_edge_idx as usize].contains_key(&(source, dest))
                    }) else {
                        // Required edges changed, restarting from the beginning
                        rollbacks_count += 1;
                        steps.truncate(1);
                        if rollbacks_count < MAX_ROLLBACKS {
                            continue 'rollback;
                        } else {
                            break 'rollback;
                        };
                    };

                    let first_edge = steps[first_step_with_this_edge].0;

                    // println!(
                    //     "Finding alternative path for edge: {} => {} and flow {} => {}",
                    //     first_edge.0, first_edge.1, source, dest
                    // );
                    let first_edge_idx = full_graph
                        .get_edge_index(first_edge.0 as u32, first_edge.1 as u32)
                        .unwrap();

                    if capacities[first_edge_idx as usize]
                        >= full_graph.get_edge_capacity(first_edge_idx)
                    {
                        for edge in &path {
                            let edge_idx = full_graph.get_edge_index(edge.0, edge.1).unwrap();
                            assert!(
                                capacities[edge_idx as usize]
                                    < full_graph.get_edge_capacity(edge_idx)
                            );
                        }

                        // println!("No edge found to remove for ({}, {})", source, dest);
                        let found_alternative = find_alternative_path(
                            &residual_graph,
                            capacities,
                            capacities_details,
                            0,
                            all_associated_flowgraphs,
                            first_edge.0 as u32,
                            first_edge.1 as u32,
                            (source, dest),
                            Some(&new_paid_edges_indices),
                        );
                        assert!(found_alternative);
                    }

                    if capacities[first_edge_idx as usize]
                        >= full_graph.get_edge_capacity(first_edge_idx)
                    {
                        // The found alternative path is not enough to free a path for an edge in the removed stack,
                        // so we should try again to see if more reductions are possible

                        // Add back the paid edge
                        paid_to_be_saved.push((source, dest));
                    } else {
                        // println!(
                        //     "Rollback path: {:?}",
                        //     steps[first_step_with_this_edge].2
                        // );
                        // for path in steps[first_step_with_this_edge].2.iter() {
                        //     println!(
                        //         "Edge: {} => {} with weight avail {} vs used {} vs residual {}",
                        //         path.0,
                        //         path.1,
                        //         full_graph.get_edge_capacity(
                        //             full_graph.get_edge_index(path.0, path.1).unwrap()
                        //         ),
                        //         capacities[full_graph
                        //             .get_edge_index(path.0, path.1)
                        //             .unwrap()
                        //             as usize],
                        //             residual_graph.get_edge_capacity(
                        //                 full_graph.get_edge_index(path.0, path.1).unwrap()
                        //             )
                        //         );
                        // }

                        steps.truncate(first_step_with_this_edge);

                        // println!(
                        //     "Rollback {}=>{} with {} steps",
                        //     first_edge.0,
                        //     first_edge.1,
                        //     steps.len()
                        // );
                        rollbacks_count += 1;

                        if steps.len() > 0 && rollbacks_count < MAX_ROLLBACKS {
                            continue 'rollback;
                        } else if rollbacks_count >= MAX_ROLLBACKS {
                            println!("Rollback limit reached!");
                            break 'rollback;
                        } else {
                            println!("Improvement found!");
                            break 'rollback;
                        }
                    }
                }
                // assert!(found_edge);
            }

            // println!("Remaining flow for {} -> {}: {}", source, dest, flow);
        }

        let mut tot_saved_cost = 0;
        for &(source, dest) in new_saved_edges.iter() {
            tot_saved_cost += mapping.adj_list[source].get(&(dest as u32)).unwrap();
            // match graph.get_edge_type(source, dest).unwrap() {
            //     EdgeType::Paid { cost } => {
            //         println!(
            //             "Found saved edge: {} -> {} with cost {}",
            //             source, dest, cost
            //         );
            //         tot_saved_cost += cost;
            //     }
            //     EdgeType::Saved { cost } => unreachable!(),
            // }
        }

        let mut tot_paid_cost = 0;
        for &(source, dest) in new_paid_edges.iter() {
            match graph.get_edge_type(source, dest).unwrap() {
                EdgeType::Paid { .. } => unreachable!(),
                EdgeType::Saved { cost } => tot_paid_cost += cost,
            }
        }

        if tot_paid_cost < tot_saved_cost {
            println!(
                "Found improvement SAVED: {} PAID: {}",
                tot_saved_cost, tot_paid_cost
            );
            println!("Saved edges: {:?}", new_saved_edges);
            println!("Paid edges: {:?}", new_paid_edges);

            let mut new_flows = FxHashMap::default();

            for (source, dest) in new_paid_edges.iter().copied() {
                let edge_idx = full_graph
                    .get_edge_index(source as u32, dest as u32)
                    .unwrap();
                let cap_details = capacities_details[edge_idx as usize].clone();
                for (edge, _) in cap_details {
                    remove_flow(
                        &full_graph,
                        capacities,
                        capacities_details,
                        source as u32,
                        dest as u32,
                        edge,
                        &mut all_associated_flowgraphs.get_mut(&edge).unwrap(),
                        Some(
                            new_flows
                                .entry((dest, source))
                                .or_insert_with(|| vec![vec![]; mapping.n1]),
                        ),
                    );
                }
                assert!(capacities_details[edge_idx as usize].is_empty());
                assert_eq!(capacities[edge_idx as usize], 0);
            }

            for edge in new_saved_edges.iter().copied() {
                let flow = all_associated_flowgraphs.remove(&edge).unwrap();
                assert!(
                    flow.iter().all(|e| {
                        if !e.is_empty() {
                            println!("Edge: {} => {} with flow: {:?}", edge.0, edge.1, e);
                        }

                        e.is_empty()
                    }),
                    "Remaining edges: {:?}",
                    flow.iter()
                        .enumerate()
                        .filter(|(_, e)| !e.is_empty())
                        .collect::<Vec<_>>()
                );
            }

            assert!(check_flows_correctness(
                &full_graph,
                &capacities,
                &capacities_details,
                &all_associated_flowgraphs,
            ));

            println!(
                "New flows: {:?}",
                new_flows
                    .iter()
                    .map(|(k, v)| (
                        k,
                        v.iter()
                            .enumerate()
                            .map(|(i, e)| e.iter().map(move |(e, w)| (i, *e, *w)))
                            .flatten()
                            .collect::<Vec<_>>()
                    ))
                    .collect::<Vec<_>>()
            );

            // todo!("Apply the changes");

            for (edge, flows) in new_flows {
                all_associated_flowgraphs.insert(edge, flows);
            }

            return true;
        } else {
            println!(
                "Improvement 2 checking failed SAVED: {} PAID: {}",
                tot_saved_cost, tot_paid_cost
            );
        }
        break; // 'rollback;
    }
    false
}

fn heuristic_v4_step4_internal(
    mapping: &TopologicalMapping,
    all_associated_flowgraphs: &mut FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> bool {
    let graph = build_full_dag(mapping, all_associated_flowgraphs);
    let full_graph = OptimMaxFlow::new(&graph);

    let mut capacities = vec![0; full_graph.get_edges_count()];
    let mut capacities_details: Vec<FxHashMap<(usize, usize), u32>> =
        vec![FxHashMap::default(); full_graph.get_edges_count()];

    build_capacities(
        &full_graph,
        &mut capacities,
        &mut capacities_details,
        all_associated_flowgraphs,
    );

    // Step 4, try to lower overcapacities
    loop {
        let mut edge_pairs = vec![];
        for oc_edge_source in 0..graph.get_node_count() {
            for (oc_edge_dest, oc_weight) in graph.get_node_adj_list(oc_edge_source) {
                edge_pairs.push((oc_edge_source, oc_edge_dest, oc_weight));
            }
        }

        edge_pairs.sort_by_cached_key(|&(s, d, _)| {
            let idx = full_graph.get_edge_index(s as u32, d as u32).unwrap();
            let used_capacity = capacities[idx as usize];
            let avail_capacity = full_graph.get_edge_capacity(idx);
            (avail_capacity as i32) - (used_capacity as i32)
        });

        for (s, d, _w) in edge_pairs {
            let idx = full_graph.get_edge_index(s as u32, d as u32).unwrap();
            let used_capacity = capacities[idx as usize];
            let avail_capacity = full_graph.get_edge_capacity(idx);
            let delta = (used_capacity as i32) - (avail_capacity as i32);

            // Try to remove this delta
            if delta > 0 {
                println!(
                    "Analysis on edge {} -> {} with avail: {} and used: {} delta: {}",
                    s, d, avail_capacity, used_capacity, delta
                );
                if find_improvement_from_edge(
                    mapping,
                    &graph,
                    &full_graph,
                    &mut capacities,
                    &mut capacities_details,
                    all_associated_flowgraphs,
                    s,
                    d,
                    false,
                ) {
                    return true;
                } else {
                    let mut capacities = capacities.clone();
                    let mut capacities_details = capacities_details.clone();
                    let mut associated_graphs = all_associated_flowgraphs.clone();

                    if find_improvement_from_edge(
                        mapping,
                        &graph,
                        &full_graph,
                        &mut capacities,
                        &mut capacities_details,
                        &mut associated_graphs,
                        s,
                        d,
                        true,
                    ) {
                        return true;
                    }
                }
            }
        }

        {
            let mut final_sorted_overcaps: Vec<_> = capacities
                .iter()
                .copied()
                .enumerate()
                .map(|(i, c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
                .collect();
            final_sorted_overcaps.sort_unstable();
            final_sorted_overcaps.reverse();
            while final_sorted_overcaps.last() == Some(&0) {
                final_sorted_overcaps.pop();
            }
            println!(
                "Final Sorted overcapacities: {:?}",
                &final_sorted_overcaps[..]
            );
            println!(
                "Overcapacity: {:?} SUM: {}",
                capacities
                    .iter()
                    .enumerate()
                    .map(|(i, &c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
                    .sum::<u32>(),
                capacities.iter().sum::<u32>(),
            );
        }
        break;
    }
    false
}
