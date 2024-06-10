use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    heuristic_v4::utils::OrderedEdgeSet, max_flow_basic::FlowGraph, optim_max_flow::OptimMaxFlow,
    topological::TopologicalMapping,
};

/*
    New proof heuristic steps:
        - Compute normal maximal provable edges
        - Put the other non-provable edges in the graph, minimizing the max-overcapacity of each added path
    If no edge has overcapacity => optimal
    Else:
        - For each edge with overcapacity:
            - Try to check if there is some alternative path that can lower the current overcapacity
                without using another edge with an overcapacity >= of the one of this node
                => If the final overcapacity is 0 ==> optimal, continue
                => Else: the current edge has overcapacity and it is a cut for each flow passing across it, unless there is a blocking flow passing by this edge.
                    put the edge with overcapacity in a queue Q

                    - Remove the edge with overcapacity from the graph and add it to a {removed nodes} set,
                            add the flows to a queue Q of to-be-cut flows and remove the flows passing in the edge from the graph
                    - Start with a saving score of +overcapacity
                    X: while Q is not empty:
                        - Check for each of the flows in Q if now there exists a new path that improves the flow of these nodes
                            - lower the  saving score by 1 for each found path, and add the path to the graph
                                if the saving score is <= 0, abort (TODO: Try to prove the optimality if possible)
                                    * Optimality proof 2 choices:
                                        - If no flow is removed and the current exploring path is removed from the graph,
                                            if it is not possible to find a cut there exists a way to improve the lower bound.
                                        - If flow of removed edges is removed and the current path is not removed, the proof consists in the path that gives an augmentation in the flow.
                        peek the first element of Q:
                            find a path form el_s and el_t in the original graph / {removed nodes}
                            - If no such path exists, continue
                            - else:
                                - Follow the path, stop on the first full node (node with overcapacity >= 0):
                                    - Try to check if it is possible to lower the node's capacity by 1 by finding an alternative path for one of the passing flows
                                        - If it is possible, continue following the path
                                        - Else remove the edge, add all the edges related to the passing flows to Q and clear the flows from the graph.
                                            - If the overcapacity of the edge is > 0, add it to the saving score
                                            - Continue X
*/

pub(crate) fn add_flow_graph_to_capacities(
    graph: &OptimMaxFlow,
    capacities: &mut [u32],
    capacities_details: &mut [FxHashMap<(usize, usize), u32>],
    flow_graph: &[Vec<(u32, u32)>],
    proved_edge: (usize, usize),
    subtract: bool,
    _check_not_overcapacity: bool,
) {
    for (source, edges) in flow_graph.iter().enumerate() {
        for (target, weight) in edges {
            let Some(edge_idx) = graph.get_edge_index(source as u32, *target) else {
                panic!(
                    "Edge not found: {} -> {} rev: {:?}",
                    source,
                    target,
                    graph.get_edge_index(*target, source as u32)
                );
            };
            if subtract {
                capacities[edge_idx as usize] -= *weight;
                *capacities_details[edge_idx as usize]
                    .get_mut(&proved_edge)
                    .unwrap() -= *weight;

                if capacities_details[edge_idx as usize]
                    .get(&proved_edge)
                    .unwrap()
                    == &0
                {
                    capacities_details[edge_idx as usize].remove(&proved_edge);
                }
            } else {
                capacities[edge_idx as usize] += *weight;
                *capacities_details[edge_idx as usize]
                    .entry(proved_edge)
                    .or_default() += *weight;
            }

            let first_sum = capacities[edge_idx as usize];
            let detail_sum = capacities_details[edge_idx as usize]
                .iter()
                .map(|(_, &w)| w)
                .sum::<u32>();
            assert_eq!(first_sum, detail_sum);
        }
    }
}

pub(crate) fn find_alternative_path(
    full_graph: &OptimMaxFlow,
    capacities: &mut [u32],
    capacities_details: &mut [FxHashMap<(usize, usize), u32>],
    final_max_overcapacity: i32,
    all_associated_graphs: &mut FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
    oc_edge_source: u32,
    oc_edge_dest: u32,
    cap_detail_edge: (usize, usize),
    excluded_edges_indices: Option<&FxHashSet<u32>>,
) -> bool {
    let mut residual_graph = full_graph.clone();
    for eidx in 0..capacities.len() {
        if excluded_edges_indices
            .as_ref()
            .map(|ex| ex.contains(&(eidx as u32)))
            .unwrap_or(false)
        {
            residual_graph.remove_edge_idx(eidx as u32);
        } else {
            residual_graph.saturating_decrease_capacity(
                eidx as u32,
                capacities[eidx].saturating_add_signed(-final_max_overcapacity),
            );
        }
    }
    let mut cache = residual_graph.create_cache();

    let oc_edge_idx = full_graph
        .get_edge_index(oc_edge_source, oc_edge_dest)
        .unwrap();

    let current_oc_contribution = capacities_details[oc_edge_idx as usize]
        .get(&cap_detail_edge)
        .copied()
        .unwrap_or(0);

    if current_oc_contribution == 0 {
        return false;
    }

    residual_graph.add_back_reverse_flow(
        &mut cache,
        &all_associated_graphs[&cap_detail_edge],
        oc_edge_idx,
    );

    let alternate_flow = residual_graph.compute_max_flow(
        oc_edge_source as u32,
        oc_edge_dest as u32,
        &mut cache,
        Some(current_oc_contribution as u64),
        true,
        None,
    );

    if alternate_flow > 0 {
        let mut new_flow = residual_graph
            .get_flow_graph(&mut cache, cap_detail_edge.0 as u32)
            .to_vec();

        assert!(current_oc_contribution <= capacities[oc_edge_idx as usize]);

        // Subtract old flow
        add_flow_graph_to_capacities(
            &full_graph,
            capacities,
            capacities_details,
            &all_associated_graphs[&cap_detail_edge],
            cap_detail_edge,
            true,
            false,
        );

        let oc_flow_idx = new_flow[oc_edge_source as usize]
            .iter()
            .position(|(t, _)| *t == oc_edge_dest);

        if let Some(oc_flow_idx) = oc_flow_idx {
            new_flow[oc_edge_source as usize][oc_flow_idx].1 -= alternate_flow as u32;
            if new_flow[oc_edge_source as usize][oc_flow_idx].1 == 0 {
                new_flow[oc_edge_source as usize].remove(oc_flow_idx);
            }
        }

        *all_associated_graphs.get_mut(&cap_detail_edge).unwrap() = new_flow.to_vec();

        // Add the new flow
        add_flow_graph_to_capacities(
            &full_graph,
            capacities,
            capacities_details,
            &all_associated_graphs[&cap_detail_edge],
            cap_detail_edge,
            false,
            true,
        );

        true
    } else {
        false
    }
}

pub(crate) fn build_capacities(
    full_graph: &OptimMaxFlow,
    capacities: &mut [u32],
    capacities_details: &mut [FxHashMap<(usize, usize), u32>],
    associated_graphs: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) {
    for (edge, associated) in associated_graphs {
        add_flow_graph_to_capacities(
            full_graph,
            capacities,
            capacities_details,
            associated,
            *edge,
            false,
            true,
        );
    }
}

/// Places all the flows on the graph optimizing the overcapacities (preferring smaller ocs instead of larger ones)
pub fn heuristic_v4_step3(
    mapping: &mut TopologicalMapping,
    graph: &OrderedEdgeSet,
    proved_edges_associated_flowgraphs: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
    not_proven_edges: &[(usize, usize)],
    fast_mode: bool,
    keep_only_largest_oc: bool,
) -> FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>> {
    let mut full_graph = OptimMaxFlow::new(graph);
    // full_graph.sanity_check();
    let mut cache = full_graph.create_cache();

    let mut capacities = vec![0; full_graph.get_edges_count()];
    let mut capacities_details: Vec<FxHashMap<(usize, usize), u32>> =
        vec![FxHashMap::default(); full_graph.get_edges_count()];

    build_capacities(
        &full_graph,
        &mut capacities,
        &mut capacities_details,
        proved_edges_associated_flowgraphs,
    );

    let mut all_associated_graphs = proved_edges_associated_flowgraphs.clone();

    for &edge in not_proven_edges {
        let (source, sink) = edge;

        let Some(&weight) = mapping.adj_list[source].get(&(sink as u32)) else {
            log::error!("Saved edge not found: {} -> {}", source, sink);
            continue;
        };
        let weight = weight as u64;

        let flow = full_graph.compute_max_flow(
            source as u32,
            sink as u32,
            &mut cache,
            Some(weight),
            true,
            None,
        );
        assert!(flow as u64 >= weight, "Flow {} < weight {}", flow, weight);

        let flow_graph = full_graph.get_flow_graph(&mut cache, source as u32);
        all_associated_graphs.insert((source, sink), flow_graph.to_vec());

        add_flow_graph_to_capacities(
            &full_graph,
            &mut capacities,
            &mut capacities_details,
            flow_graph,
            (edge.0, edge.1),
            false,
            false,
        );
    }

    if fast_mode {
        return all_associated_graphs;
    }

    loop {
        let start_overcapacity = capacities
            .iter()
            .enumerate()
            .map(|(i, &c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
            .sum::<u32>();

        let mut start_sorted_overcaps: Vec<_> = capacities
            .iter()
            .copied()
            .enumerate()
            .map(|(i, c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
            .collect();
        start_sorted_overcaps.sort_unstable();
        start_sorted_overcaps.reverse();
        while start_sorted_overcaps.last() == Some(&0) {
            start_sorted_overcaps.pop();
        }

        println!(
            "Start Sorted overcapacities: {:?}",
            &start_sorted_overcaps[..]
        );

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

        let mut changed = false;
        for &(oc_edge_source, oc_edge_dest, oc_weight) in &edge_pairs {
            if oc_weight == 0 {
                continue;
            }

            let oc_edge_idx = full_graph
                .get_edge_index(oc_edge_source as u32, oc_edge_dest as u32)
                .unwrap();

            let current_cap_details = capacities_details[oc_edge_idx as usize].clone();
            for cap_detail in &current_cap_details {
                let cap_detail = (*cap_detail.0, *cap_detail.1);
                let used_capacity = capacities[oc_edge_idx as usize];
                let avail_capacity = full_graph.get_edge_capacity(oc_edge_idx);

                if true {
                    // used_capacity > avail_capacity {
                    let final_max_overcapacity =
                        (used_capacity as i32) - (avail_capacity as i32) - 1;

                    // Edge is overcommitted
                    if find_alternative_path(
                        &full_graph,
                        &mut capacities,
                        &mut capacities_details,
                        final_max_overcapacity,
                        &mut all_associated_graphs,
                        oc_edge_source as u32,
                        oc_edge_dest as u32,
                        cap_detail.0,
                        None,
                    ) {
                        changed = true;
                    }
                    // assert_eq!(alternate_flow, 0);

                    // Is there an alternative path from s to t that does not add new overcommitted edges?
                } else {
                    break;
                }
            }
        }

        let mut sorted_overcaps: Vec<_> = capacities
            .iter()
            .copied()
            .enumerate()
            .map(|(i, c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
            .collect();
        sorted_overcaps.sort_unstable();
        sorted_overcaps.reverse();
        while sorted_overcaps.last() == Some(&0) {
            sorted_overcaps.pop();
        }

        println!("Sorted overcapacities: {:?}", &sorted_overcaps[..]);

        println!(
            "Overcapacity: {:?} vs init {} SUM: {}",
            capacities
                .iter()
                .enumerate()
                .map(|(i, &c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
                .sum::<u32>(),
            start_overcapacity,
            capacities.iter().sum::<u32>(),
        );
        if !changed || (start_sorted_overcaps == sorted_overcaps) {
            if false && keep_only_largest_oc {
                let mut overcaps: Vec<_> = capacities
                    .iter()
                    .copied()
                    .enumerate()
                    .map(|(i, c)| c.saturating_sub(full_graph.get_edge_capacity(i as u32)))
                    .collect();

                let mut skip_before = 18;
                let mut non_fixable = 3;

                for (node, list) in mapping.min_adj_list.iter_mut().enumerate() {
                    for (sink, weight) in list.iter_mut() {
                        if let Some(edge_idx) = full_graph.get_edge_index(*sink as u32, node as u32)
                        {
                            let overcap = &mut overcaps[edge_idx as usize];
                            if *overcap > 0 {
                                if skip_before > 0 || non_fixable == 0 {
                                    *weight += *overcap as u32;
                                    *mapping.adj_list[*sink as usize]
                                        .get_mut(&(node as u32))
                                        .unwrap() += *overcap as u32;

                                    *overcap = 0;
                                } else {
                                    println!("Not fixing edge: {} -> {}", node, *sink);
                                }

                                if skip_before > 0 {
                                    skip_before -= 1;
                                } else if non_fixable > 0 {
                                    non_fixable -= 1;
                                }
                            }
                        }
                    }
                }
            }

            break;
        }
    }

    all_associated_graphs
}
