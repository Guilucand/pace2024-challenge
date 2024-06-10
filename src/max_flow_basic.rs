use rustc_hash::{FxHashMap, FxHashSet};

use crate::heuristic_v4::utils::EdgeType;

fn dfs_visit(
    node: usize,
    visited: &mut Vec<bool>,
    adj_list: &[FxHashMap<usize, u64>],
    target: SinkType,
    threshold: Option<NodesThreshold>,
) -> Option<Vec<usize>> {
    if match target {
        SinkType::Single(target) => node == target,
        SinkType::Multiple(targets) => targets.contains(&node),
    } {
        return Some(vec![node]);
    }

    if visited[node] {
        return None;
    }

    visited[node] = true;

    for (head, &weight) in adj_list[node].iter() {
        if weight > 0 {
            if let Some(threshold) = threshold {
                if let Some(&order) = threshold.nodes_ordering.get(&node) {
                    if order < threshold.threshold {
                        continue;
                    }
                }
            }

            if !visited[*head] {
                if let Some(mut path) = dfs_visit(*head, visited, adj_list, target, threshold) {
                    path.push(node);
                    return Some(path);
                }
            }
        }
    }

    None
}

#[derive(Clone, Copy)]
pub enum SinkType<'a> {
    Single(usize),
    Multiple(&'a FxHashSet<usize>),
}

pub fn compute_max_flow_directed(
    start_adj_list: &[FxHashMap<u32, u32>],
    source: usize,
    sink: SinkType,
    compute_min_cut: bool,
    limit: Option<u64>,
) -> (i64, Option<Vec<(usize, usize)>>) {
    let start_adj_list: Vec<FxHashMap<usize, u64>> = start_adj_list
        .iter()
        .map(|n| n.iter().map(|(k, v)| (*k as usize, *v as u64)).collect())
        .collect();

    let mut adj_list = vec![FxHashMap::default(); start_adj_list.len()];

    // Remap the nodes putting the joined targets in position 0
    for (node, edges) in start_adj_list.iter().enumerate() {
        for (&target, weight) in edges.iter() {
            let target = target as usize;
            *adj_list[node].entry(target).or_default() += *weight as u64;
            adj_list[target].entry(node).or_insert(0);
        }
    }

    let mut tot_flow = 0;

    loop {
        let Some(path) = dfs_visit(
            source,
            &mut vec![false; start_adj_list.len()],
            &adj_list,
            sink,
            None,
        ) else {
            break;
        };

        // Path is reversed

        let mut max_flow = u64::MAX;
        for edge in path.windows(2) {
            let (node, next) = (edge[1], edge[0]);
            max_flow = max_flow.min(*adj_list[node].get(&next).unwrap());
        }
        tot_flow += max_flow as i64;

        if let Some(limit) = limit {
            if tot_flow >= limit as i64 {
                return (tot_flow, None);
            }
        }

        for edge in path.windows(2) {
            let (node, next) = (edge[1], edge[0]);
            let capacity = adj_list[node].get_mut(&next).unwrap();
            *capacity -= max_flow;
            let rev_capacity = adj_list[next].get_mut(&node).unwrap();
            *rev_capacity += max_flow;
        }
    }

    let min_cut = if compute_min_cut {
        let mut reachable_nodes = FxHashSet::default();
        let mut visited = vec![false; start_adj_list.len()];
        let mut stack = vec![source];
        while let Some(node) = stack.pop() {
            if visited[node] {
                continue;
            }
            visited[node] = true;
            reachable_nodes.insert(node);
            for (head, &weight) in adj_list[node].iter() {
                if weight > 0 {
                    stack.push(*head);
                }
            }
        }

        let mut min_cut = FxHashSet::default();
        for &node in reachable_nodes.iter() {
            for (&edge, &weight) in start_adj_list[node].iter() {
                if weight > 0 && (edge == source || !reachable_nodes.contains(&edge)) {
                    if start_adj_list[node].contains_key(&edge) {
                        min_cut.insert((node, edge));
                    }
                }
            }
        }

        // if let SinkType::Multiple(_, others) = sink {
        //     for &(node, weight) in others {
        //         if !reachable_nodes.contains(&(node as usize)) {
        //             tot_flow = tot_flow - weight as i64;
        //         }
        //     }
        // }

        Some(min_cut.into_iter().collect())
    } else {
        None
    };

    // println!("Max flow for nodes {} -> {}: {}", source, sink, tot_flow);
    (tot_flow, min_cut)
}

pub trait FlowGraph {
    fn get_node_count(&self) -> usize;
    // Target + edge capacity
    fn get_node_adj_list(&self, node: usize) -> impl Iterator<Item = (usize, u64)>;
    fn get_edge_type(&self, source: usize, target: usize) -> Option<EdgeType>;
}

impl FlowGraph for [Vec<(u32, u32)>] {
    fn get_node_count(&self) -> usize {
        self.len()
    }

    fn get_node_adj_list(&self, node: usize) -> impl Iterator<Item = (usize, u64)> {
        self[node].iter().map(|(k, v)| (*k as usize, *v as u64))
    }

    fn get_edge_type(&self, source: usize, target: usize) -> Option<EdgeType> {
        self[source].iter().find_map(|(k, v)| {
            if *k as usize == target {
                Some(EdgeType::Saved { cost: *v })
            } else {
                None
            }
        })
    }
}

#[derive(Clone, Copy)]
pub struct NodesThreshold<'a> {
    pub nodes_ordering: &'a FxHashMap<usize, usize>,
    pub threshold: usize,
}

pub fn compute_max_flow_directed_v2(
    flow_graph: &impl FlowGraph,
    source: usize,
    sink: SinkType,
    compute_min_cut: bool,
    limit: Option<u64>,
    threshold: Option<NodesThreshold>,
) -> (i64, Option<Vec<(usize, usize)>>, Vec<(usize, usize)>) {
    let nodes_count = flow_graph.get_node_count();

    let mut adj_list = vec![FxHashMap::default(); nodes_count];

    // Remap the nodes putting the joined targets in position 0
    for node in 0..nodes_count {
        for (target, capacity) in flow_graph.get_node_adj_list(node) {
            let target = target as usize;
            *adj_list[node].entry(target).or_default() += capacity;
            adj_list[target].entry(node).or_insert(0);
        }
    }

    let mut tot_flow = 0;
    let mut removed_edges = vec![];

    loop {
        let Some(path) = dfs_visit(
            source,
            &mut vec![false; nodes_count],
            &adj_list,
            sink,
            threshold,
        ) else {
            break;
        };

        // Path is reversed

        let mut max_flow = u64::MAX;
        for edge in path.windows(2) {
            let (node, next) = (edge[1], edge[0]);
            max_flow = max_flow.min(*adj_list[node].get(&next).unwrap());
        }
        tot_flow += max_flow as i64;

        if let Some(limit) = limit {
            if tot_flow >= limit as i64 {
                return (tot_flow, None, vec![]);
            }
        }

        for edge in path.windows(2) {
            let (node, next) = (edge[1], edge[0]);
            let capacity = adj_list[node].get_mut(&next).unwrap();
            *capacity -= max_flow;
            let rev_capacity = adj_list[next].get_mut(&node).unwrap();
            *rev_capacity += max_flow;
        }
    }

    let min_cut = if compute_min_cut {
        let mut reachable_nodes = FxHashSet::default();
        let mut visited = vec![false; nodes_count];
        let mut stack = vec![source];
        while let Some(node) = stack.pop() {
            if visited[node] {
                continue;
            }
            visited[node] = true;
            reachable_nodes.insert(node);
            for (head, &weight) in adj_list[node].iter() {
                if weight > 0 {
                    stack.push(*head);
                }
            }
        }

        let mut min_cut = FxHashSet::default();

        for &node in reachable_nodes.iter() {
            for (edge, _) in flow_graph.get_node_adj_list(node) {
                if !reachable_nodes.contains(&edge) {
                    if let Some(edge_type) = flow_graph.get_edge_type(node, edge) {
                        match edge_type {
                            EdgeType::Paid { cost } => {
                                removed_edges.push((node, edge));
                                tot_flow = tot_flow - cost as i64;
                            }
                            EdgeType::Saved { .. } => {
                                min_cut.insert((node, edge));
                            }
                        }
                    }
                }
            }
        }

        Some(min_cut.into_iter().collect())
    } else {
        None
    };

    // println!("Max flow for nodes {} -> {}: {}", source, sink, tot_flow);
    (tot_flow, min_cut, removed_edges)
}
