use std::cmp::min;

use rustc_hash::FxHashMap;

fn scc_recursion(
    node: usize,
    instance: &impl GraphRepresentation,
    visit_order: &mut [Option<usize>],
    lowest_reachable: &mut [usize],
    is_on_stack: &mut [bool],
    stack: &mut Vec<usize>,
    time: &mut usize,
    components: &mut Vec<Vec<usize>>,
) {
    if visit_order[node].is_some() {
        return;
    }
    visit_order[node] = Some(*time);
    lowest_reachable[node] = *time;
    *time += 1;
    is_on_stack[node] = true;
    stack.push(node);

    for neighbor in instance.get_node_adj(node) {
        if visit_order[neighbor].is_none() {
            scc_recursion(
                neighbor,
                instance,
                visit_order,
                lowest_reachable,
                is_on_stack,
                stack,
                time,
                components,
            );
        }

        if is_on_stack[neighbor] {
            lowest_reachable[node] = min(lowest_reachable[node], lowest_reachable[neighbor]);
        }
    }

    if lowest_reachable[node] == visit_order[node].unwrap() {
        let mut component = vec![];
        while let Some(top) = stack.pop() {
            is_on_stack[top] = false;
            component.push(top);
            if top == node {
                break;
            }
        }
        components.push(component);
    }
}

pub fn topo_sort(
    node: usize,
    cc_tree: &[Vec<usize>],
    visited: &mut [bool],
    post_order: &mut Vec<usize>,
) {
    if visited[node] {
        return;
    }
    visited[node] = true;

    for neighbor in cc_tree[node].iter() {
        if !visited[*neighbor] {
            topo_sort(*neighbor, cc_tree, visited, post_order);
        }
    }

    post_order.push(node);
}

pub trait GraphRepresentation {
    fn nodes_count(&self) -> usize;
    fn get_node_adj(&self, node: usize) -> impl Iterator<Item = usize>;
}

pub fn find_ordered_connected_components(
    instance: &impl GraphRepresentation,
    optimal_order: Option<&[usize]>,
) -> Vec<(Vec<usize>, Option<Vec<usize>>)> {
    let nodes_count = instance.nodes_count();

    let mut components = vec![];
    let mut visit_order = vec![None; nodes_count];

    for start in 0..nodes_count {
        if visit_order[start].is_some() {
            continue;
        }
        scc_recursion(
            start,
            instance,
            &mut visit_order,
            &mut vec![0; nodes_count],
            &mut vec![false; nodes_count],
            &mut vec![],
            &mut 0,
            &mut components,
        );
    }

    let mut nodes_to_cc = FxHashMap::default();

    for (idx, component) in components.iter().enumerate() {
        for node in component {
            nodes_to_cc.insert(*node, idx);
        }
    }

    let mut cc_tree = vec![vec![]; components.len()];

    for node in 0..nodes_count {
        for tail in instance.get_node_adj(node) {
            let node_cc = nodes_to_cc[&node];
            let head_cc = nodes_to_cc[&tail];
            if node_cc != head_cc {
                cc_tree[node_cc].push(head_cc);
            }
        }
    }

    for list in cc_tree.iter_mut() {
        list.sort();
        list.dedup();
    }

    let mut visited = vec![false; cc_tree.len()];
    let mut post_order = vec![];

    for node in 0..cc_tree.len() {
        if !visited[node] {
            topo_sort(node, &cc_tree, &mut visited, &mut post_order);
        }
    }
    post_order.reverse();

    // let mapped_order =

    let mut components_in_order = vec![];
    for component in post_order {
        let solution_part = optimal_order.as_ref().map(|o| {
            let elements = components[component]
                .iter()
                .copied()
                .collect::<std::collections::HashSet<_>>();
            o.iter()
                .filter(|&x| elements.contains(x))
                .copied()
                .collect::<Vec<_>>()
        });

        components_in_order.push((components[component].clone(), solution_part));
    }

    components_in_order
}
