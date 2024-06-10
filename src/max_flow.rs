use crate::topological::TopologicalMapping;
use rustc_hash::{FxHashMap, FxHashSet};

fn dfs_visit(
    node: usize,
    visited: &mut Vec<bool>,
    adj_list: &Vec<FxHashMap<usize, u64>>,
    target: usize,
) -> Option<Vec<usize>> {
    if visited[node] {
        if node == target {
            return Some(vec![node]);
        } else {
            return None;
        }
    }

    visited[node] = true;

    for (head, &weight) in adj_list[node].iter() {
        if weight > 0 {
            if !visited[*head] || *head == target {
                if let Some(mut path) = dfs_visit(*head, visited, adj_list, target) {
                    path.push(node);
                    return Some(path);
                }
            }
        }
    }

    None
}

pub fn compute_min_cut_circulation(
    mapping: &TopologicalMapping,
    joined_targets: &[&[usize]],
    skip_nodes: &[bool],
) -> (u64, FxHashSet<(usize, usize)>, u64, FxHashSet<usize>) {
    let mut start_adj_list = vec![FxHashMap::default(); mapping.n1];
    let mut adj_list = vec![FxHashMap::default(); mapping.n1];

    let joined_cluster_map = joined_targets
        .iter()
        .enumerate()
        .flat_map(|(idx, &cluster)| {
            cluster
                .iter()
                .enumerate()
                .map(move |(pos, &node)| (node, (idx, pos)))
        })
        .collect::<FxHashMap<_, _>>();

    let mut join_cost = 0;

    // Remap the nodes putting the joined targets in position 0
    for (node, edges) in mapping.adj_list.iter().enumerate() {
        if skip_nodes[node] {
            continue;
        }

        for (&target, weight) in edges.iter() {
            let target = target as usize;
            if skip_nodes[target] {
                continue;
            }

            let node_cluster = joined_cluster_map.get(&node);
            let target_cluster = joined_cluster_map.get(&target);

            // Skip self reference edges and add their cost if they are paid
            if let Some((node_cluster, npos)) = node_cluster {
                if let Some((target_cluster, tpos)) = target_cluster {
                    // Add
                    if node_cluster == target_cluster {
                        if npos < tpos {
                            join_cost += *weight as u64;
                        }
                        continue;
                    }
                }
            }

            *adj_list[node].entry(target).or_default() += *weight as u64;
            adj_list[target].entry(node).or_insert(0);

            *start_adj_list[node].entry(target).or_insert(0) += *weight as u64;
        }
    }

    // Add 2 kinds of new links
    // First: from all joined targets to the first target
    let source = joined_targets[0][0];
    for &target in joined_targets[0].iter().skip(1) {
        *adj_list[target].entry(source).or_insert(0) += u64::MAX / 4;
        adj_list[target].entry(0).or_insert(0);
    }

    // Second add a circular path for all nodes in successive joined targets
    for joined_target in joined_targets.iter().skip(1) {
        for window in joined_target.windows(2) {
            let (node, next) = (window[0], window[1]);
            *adj_list[node].entry(next).or_insert(0) += u64::MAX / 4;
            adj_list[next].entry(node).or_insert(0);
        }
        if joined_target.len() > 1 {
            *adj_list[*joined_target.last().unwrap()]
                .entry(joined_target[0])
                .or_insert(0) += u64::MAX / 4;
            adj_list[joined_target[0]]
                .entry(*joined_target.last().unwrap())
                .or_insert(0);
        }
    }

    // panic!("Join cost: {}", join_cost);
    let mut tot_flow = 0;

    loop {
        let Some(mut path) = dfs_visit(source, &mut vec![false; mapping.n1], &adj_list, source)
        else {
            break;
        };

        path.reverse();

        let mut max_flow = u64::MAX;
        for edge in path.windows(2) {
            let (node, next) = (edge[0], edge[1]);
            max_flow = max_flow.min(*adj_list[node].get(&next).unwrap());
        }
        tot_flow += max_flow;

        for edge in path.windows(2) {
            let (node, next) = (edge[0], edge[1]);
            let capacity = adj_list[node].get_mut(&next).unwrap();
            *capacity -= max_flow;
            if node != source && next != source {
                let rev_capacity = adj_list[next].get_mut(&node).unwrap();
                *rev_capacity += max_flow;
            }
        }
    }

    let mut reachable_nodes = FxHashSet::default();
    let mut visited = vec![false; mapping.n1];
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
                if mapping.adj_list[node].contains_key(&(edge as u32)) {
                    min_cut.insert((node, edge));
                }
            }
        }
    }

    reachable_nodes.remove(&source);

    // println!("Max flow for nodes {:?}: {}", joined_targets, tot_flow);
    (tot_flow, min_cut, join_cost, reachable_nodes)
}
