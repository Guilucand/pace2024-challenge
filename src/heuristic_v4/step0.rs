use crate::topological::TopologicalMapping;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Costs {
    paid: u64,
    saved: u64,
    rev_paid: u64,
}

fn compute_costs_by_position(mapping: &TopologicalMapping, order: &[usize]) -> Vec<Costs> {
    let mut costs_by_node = vec![
        Costs {
            paid: 0,
            saved: 0,
            rev_paid: 0
        };
        mapping.n1
    ];
    let mut on_left = vec![false; mapping.n1];
    let mut on_right = vec![false; mapping.n1];

    for el in order {
        on_right[*el] = true;
    }

    for el in order {
        for (head, weight) in mapping.adj_list[*el].iter() {
            if on_left[*head as usize] {
                costs_by_node[*el as usize].saved += *weight as u64;
            } else if on_right[*head as usize] {
                costs_by_node[*head as usize].paid += *weight as u64;
                costs_by_node[*el as usize].rev_paid += *weight as u64;
            }
        }
        on_left[*el] = true;
        on_right[*el] = false;
    }

    let mut costs_by_position = vec![
        Costs {
            paid: 0,
            saved: 0,
            rev_paid: 0
        };
        order.len()
    ];
    for (idx, el) in order.iter().enumerate() {
        costs_by_position[idx] = costs_by_node[*el];
    }
    costs_by_position
}

fn swap_elements(mapping: &TopologicalMapping, i: usize, costs: &mut [Costs], order: &mut [usize]) {
    let before = i;
    let after = i + 1;

    let current_cost = mapping.adj_list[order[before]]
        .get(&(order[after] as u32))
        .copied()
        .unwrap_or(0);

    let inverted_cost = mapping.adj_list[order[after]]
        .get(&(order[before] as u32))
        .copied()
        .unwrap_or(0);

    costs.swap(before, after);
    order.swap(before, after);

    costs[before].paid -= current_cost as u64;
    costs[after].rev_paid -= current_cost as u64;
    costs[after].saved += current_cost as u64;

    costs[after].paid += inverted_cost as u64;
    costs[before].rev_paid += inverted_cost as u64;
    costs[before].saved -= inverted_cost as u64;
}

fn fix_element_order(
    mapping: &TopologicalMapping,
    i: usize,
    costs: &mut [Costs],
    order: &[usize],
    positions: &[usize],
) {
    for (head, weight) in mapping.adj_list[order[i]].iter() {
        let head_pos = positions[*head as usize];
        if head_pos > i {
            costs[i].rev_paid -= *weight as u64;
            costs[head_pos].paid -= *weight as u64;
        }
    }

    for (head, weight) in mapping.min_adj_list[order[i]].iter() {
        let head_pos = positions[*head as usize];
        if head_pos > i {
            costs[head_pos].saved -= *weight as u64;
        }
    }

    assert_eq!(costs[i].paid, 0);
    assert_eq!(costs[i].rev_paid, 0);
    assert_eq!(costs[i].saved, 0);
}

fn move_element_backwards(
    mapping: &TopologicalMapping,
    mut origin: usize,
    dest: usize,
    costs: &mut [Costs],
    order: &mut [usize],
    positions: &mut [usize],
) {
    while origin > dest {
        swap_elements(mapping, origin - 1, costs, order);
        positions.swap(order[origin - 1], order[origin]);
        origin -= 1;
    }
}

/// Computes a greedy ordering of the nodes,
/// by putting them earlier if the cost paid on the left is greater than the cost saved on the left
pub fn heuristic_v4_step0(mapping: &TopologicalMapping, mut order: Vec<usize>) -> Vec<usize> {
    let mut changed = true;
    while changed {
        changed = false;

        let mut slice_start = 0;
        let mut costs_by_position = compute_costs_by_position(mapping, &order);
        while slice_start < order.len() {
            let order_slice = &mut order[slice_start..];
            let costs_slice = &mut costs_by_position[slice_start..];
            let mut orig_order_positions = vec![0; mapping.n1];
            for (idx, el) in order_slice.iter().enumerate() {
                orig_order_positions[*el] = idx;
            }

            loop {
                let mut new_order = vec![];
                let mut other = vec![];
                for (cost, el) in costs_slice.iter().zip(order_slice.iter()) {
                    if cost.paid > cost.saved {
                        changed = true;
                        new_order.push(*el);
                    } else {
                        other.push(*el);
                    }
                }
                if new_order.len() == 0 {
                    break;
                }

                new_order.reverse();
                new_order.extend(other);

                for i in 0..order_slice.len() {
                    if order_slice[i] != new_order[i] {
                        let position = orig_order_positions[new_order[i]];
                        assert_eq!(new_order[i], order_slice[position]);
                        move_element_backwards(
                            mapping,
                            position,
                            i,
                            costs_slice,
                            order_slice,
                            &mut orig_order_positions,
                        );
                    }
                }
            }
            fix_element_order(mapping, 0, costs_slice, &order_slice, &orig_order_positions);
            slice_start += 1;
        }
    }
    order
}
