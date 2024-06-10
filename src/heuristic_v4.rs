use log::info;
use rustc_hash::FxHashMap;
use step4::build_full_dag;

use crate::{
    connected::find_ordered_connected_components,
    heuristic_v4::utils::check_simple_flows_correctness, time_limit::TimeLimit,
    topological::TopologicalMapping,
};

use self::{
    step0::heuristic_v4_step0, step1::heuristic_v4_step1, step2::heuristic_v4_step2,
    step3::heuristic_v4_step3,
};

pub mod step0;
pub mod step1;
pub mod step2;
pub mod step3;
pub mod step4;
pub mod utils;

pub fn execute_heuristic_v4(
    mapping: &mut TopologicalMapping,
    _order: &[usize],
    time_limit: &mut TimeLimit,
) -> (Vec<usize>, FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>) {
    let order = (0..mapping.n1).collect::<Vec<_>>();
    let order = heuristic_v4_step0(mapping, order);
    let order = heuristic_v4_step1(mapping, &order, time_limit, false);
    info!("Finished step 1");
    let (partial_proof_flows, not_proven_edges, directed_graph) =
        heuristic_v4_step2(mapping, &order, time_limit);
    info!("Finished step 2");
    let all_flows = heuristic_v4_step3(
        mapping,
        &directed_graph,
        &partial_proof_flows,
        &not_proven_edges,
        true, // mapping.n1 > 1500,
        true,
    );
    info!("Finished step 3");

    // check_simple_flows_correctness(mapping.n1, &all_flows);

    let new_flows = all_flows;

    // let new_flows = heuristic_v4_step4(mapping, &all_flows);

    check_simple_flows_correctness(mapping.n1, &new_flows);

    let graph = build_full_dag(mapping, &new_flows);
    let final_order: Vec<_> = find_ordered_connected_components(&graph, None)
        .into_iter()
        .map(|c| {
            if c.0.len() > 1 {
                log::error!(
                    "Found a connected component with more than one node: {:?}",
                    c
                );
            }
            c.0.into_iter()
        })
        .flatten()
        .collect();

    log::info!(
        "Heuristic completed. best solution cost: {}",
        mapping.compute_cost(&final_order)
    );

    // log::info!(
    //     "[PROVEN {} out of {} paid edges ({:.2}%)] Total proof edges: {} total proof bounds: {} best bound: {} vs best cost: {} {:.2}%",
    //     solution_stack.len(),
    //     sorted_paid_edges.len(),
    //     (solution_stack.len() as f64 / sorted_paid_edges.len() as f64) * 100.0,
    //     total_proof_edges,
    //     needed_triplets.len(),
    //     proven_minimal_cost,
    //     current_total_cost,
    //     ((current_total_cost - proven_minimal_cost) as f64 / proven_minimal_cost as f64) * 100.0
    // );

    (final_order, new_flows)
}

pub fn heuristic_v4_flows_to_constraints(
    flows: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> Vec<(u32, u32, u32)> {
    let mut edges_that_prove_optimality = FxHashMap::default();

    for (&(source, sink), flow_graph) in flows {
        edges_that_prove_optimality.insert(
            (source, sink),
            flow_graph
                .iter()
                .enumerate()
                .map(|(source, edges)| {
                    edges
                        .iter()
                        .map(move |(target, _)| (source as u32, *target))
                })
                .flatten()
                .collect::<Vec<_>>(),
        );
    }

    let mut needed_triplets = vec![];

    for ((source, _), proof) in edges_that_prove_optimality {
        for proof_edge in proof {
            let mut i = source as u32;
            let mut j = proof_edge.0;
            let mut k = proof_edge.1;

            if i > j {
                std::mem::swap(&mut i, &mut j);
            }
            if i > k {
                std::mem::swap(&mut i, &mut k);
            }
            if j > k {
                std::mem::swap(&mut j, &mut k);
            }

            if i == j || j == k {
                continue;
            }

            needed_triplets.push((i, j, k));
        }
    }

    needed_triplets.sort_unstable();
    needed_triplets.dedup();

    log::info!("Heuristic proof bounds: {}", needed_triplets.len(),);

    needed_triplets
}

pub fn heuristic_v4_flows_to_cycles(
    flows: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> Vec<Vec<u32>> {
    let mut cycles = vec![];

    for (&(source, sink), flow) in flows {
        let mut flow = flow.clone();

        // Find a path from source to sink and subtract the corresponding flow

        'new_path: loop {
            let mut path = vec![source as u32];
            let mut min_flow = u32::MAX;

            while path.last() != Some(&(sink as u32)) {
                let last = path.last().unwrap();
                let Some(&(next, weight)) = flow[*last as usize]
                    .iter()
                    .max_by_key(|(_, weight)| *weight)
                else {
                    // No more paths left
                    break 'new_path;
                };

                min_flow = min_flow.min(weight);
                path.push(next);
            }
            for edge in path.windows(2) {
                let source = edge[0] as usize;
                flow[source].retain_mut(|(target, weight)| {
                    if *target == edge[1] as u32 {
                        *weight -= min_flow;
                        *weight > 0
                    } else {
                        true
                    }
                });
            }
            cycles.push(path);
        }
        assert!(flow.iter().all(|n| n.is_empty()));
    }
    cycles
}
