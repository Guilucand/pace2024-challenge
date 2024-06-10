use rustc_hash::FxHashMap;

use crate::{
    connected::GraphRepresentation, max_flow_basic::FlowGraph, optim_max_flow::OptimMaxFlow,
    topological::TopologicalMapping,
};

#[derive(Clone, Copy, Debug)]
pub enum EdgeType {
    Paid { cost: u32 },
    Saved { cost: u32 },
}

#[derive(Clone, Debug)]
pub struct OrderedEdgeSet {
    pub edges: Vec<FxHashMap<u32, EdgeType>>,
}

impl OrderedEdgeSet {
    pub fn new(size: usize) -> Self {
        Self {
            edges: vec![FxHashMap::default(); size],
        }
    }
}

impl GraphRepresentation for OrderedEdgeSet {
    fn nodes_count(&self) -> usize {
        self.edges.len()
    }

    fn get_node_adj(&self, node: usize) -> impl Iterator<Item = usize> {
        self.edges[node].iter().map(|(k, _)| *k as usize)
    }
}

impl FlowGraph for OrderedEdgeSet {
    fn get_node_count(&self) -> usize {
        self.edges.len()
    }

    fn get_node_adj_list(&self, node: usize) -> impl Iterator<Item = (usize, u64)> {
        self.edges[node].iter().map(|(k, v)| {
            (
                *k as usize,
                match v {
                    EdgeType::Paid { .. } => 0,
                    EdgeType::Saved { cost } => *cost as u64,
                },
            )
        })
    }

    fn get_edge_type(&self, source: usize, target: usize) -> Option<EdgeType> {
        self.edges[source].get(&(target as u32)).copied()
    }
}

pub fn reorder_optimal_fixed(mapping: &TopologicalMapping, order: &[usize]) -> Vec<usize> {
    let mut order = order.to_vec();
    let mut positions = order
        .iter()
        .enumerate()
        .map(|(idx, &node)| (node, idx))
        .collect::<FxHashMap<_, _>>();

    let mut swapped_count = 0;

    for (&before, fixed) in mapping.fixed_orders.iter() {
        for &after in fixed {
            let before_pos = positions[&(before as usize)];
            let after_pos = positions[&(after as usize)];

            if before_pos > after_pos {
                order.swap(before_pos, after_pos);
                positions.insert(before as usize, after_pos);
                positions.insert(after as usize, before_pos);
                order.swap(before_pos, after_pos);
                swapped_count += 1;
            }
        }
    }

    if swapped_count > 0 {
        log::info!("Swapped {} fixed elements", swapped_count);
    }

    order
}

pub fn check_flows_correctness(
    full_graph: &OptimMaxFlow,
    _capacities: &[u32],
    _capacities_details: &[FxHashMap<(usize, usize), u32>],
    all_associated_graphs: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> bool {
    for (edge, graph) in all_associated_graphs {
        let mut deltas = vec![0; full_graph.get_nodes_count()];
        for source in 0..graph.len() {
            for (target, weight) in &graph[source] {
                deltas[source] -= *weight as i32;
                deltas[*target as usize] += *weight as i32;
            }
        }
        for (node, delta) in deltas.iter().enumerate() {
            if *delta != 0 {
                if node == edge.0 || node == edge.1 {
                    // Do nothing
                } else {
                    if *delta != 0 {
                        println!("Node: {} should be zero! {}", node, *delta);
                        return false;
                    }
                }
            }
        }
        if deltas[edge.0] != -deltas[edge.1] {
            println!("Edge: {:?} vs {:?}", edge, deltas);
            return false;
        }
    }
    true
}

pub fn check_simple_flows_correctness(
    nodes_count: usize,
    all_associated_graphs: &FxHashMap<(usize, usize), Vec<Vec<(u32, u32)>>>,
) -> bool {
    for (edge, graph) in all_associated_graphs {
        let mut deltas = vec![0; nodes_count];
        for source in 0..graph.len() {
            for (target, weight) in &graph[source] {
                deltas[source] -= *weight as i32;
                deltas[*target as usize] += *weight as i32;
            }
        }
        for (node, delta) in deltas.iter().enumerate() {
            if *delta != 0 {
                if node == edge.0 || node == edge.1 {
                    // Do nothing
                } else {
                    if *delta != 0 {
                        println!("Node: {} should be zero! {}", node, *delta);
                        return false;
                    }
                }
            }
        }
        if deltas[edge.0] != -deltas[edge.1] {
            println!("Edge: {:?} vs {:?}", edge, deltas);
            return false;
        }
    }
    true
}
