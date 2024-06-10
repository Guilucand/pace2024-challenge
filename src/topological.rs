use std::collections::HashMap;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::{connected::GraphRepresentation, weighted::WeightedInstance};

#[derive(Clone, Debug)]
pub struct TopologicalMapping {
    pub weighted_instance: WeightedInstance,
    pub ocr: String,
    pub n1: usize,
    pub adj_list: Vec<FxHashMap<u32, u32>>,
    pub min_adj_list: Vec<FxHashMap<u32, u32>>,
    pub minimum_cost: u64,
    pub fixed_orders: FxHashMap<u32, Vec<u32>>,
}

fn count_out_of_order_weight(left: &[u32], right: &[u32]) -> (u32, u32) {
    if left.len() == 0 {
        return (0, 0);
    }

    if right.len() == 0 {
        return (0, 0);
    }

    if left.last().unwrap() < right.first().unwrap() {
        return (0, (left.len() * right.len()) as u32);
    }

    if right.last().unwrap() < left.first().unwrap() {
        return ((left.len() * right.len()) as u32, 0);
    }

    let mut left_count = 0;
    let mut right_count = 0;
    for i in 0..right.len() {
        for j in 0..left.len() {
            left_count += (right[i] < left[j]) as u32;
            right_count += (right[i] > left[j]) as u32
        }
    }
    (left_count, right_count)
}

#[derive(Clone, Debug)]
pub struct FixedEdges {
    pub forward: FxHashSet<u32>,
    pub backward: FxHashSet<u32>,
}

impl GraphRepresentation for TopologicalMapping {
    fn nodes_count(&self) -> usize {
        self.n1
    }

    fn get_node_adj(&self, node: usize) -> impl Iterator<Item = usize> {
        self.min_adj_list[node]
            .iter()
            .map(|(k, _)| *k as usize)
            .chain(
                self.fixed_orders
                    .get(&(node as u32))
                    .map(|v| v.iter().map(|&v| v as usize))
                    .into_iter()
                    .flatten(),
            )
    }
}

impl TopologicalMapping {
    pub fn new(weighted_instance: &WeightedInstance) -> Self {
        let mut self_ = TopologicalMapping {
            weighted_instance: weighted_instance.clone(),
            ocr: weighted_instance.ocr.clone(),
            n1: weighted_instance.n1,
            min_adj_list: vec![FxHashMap::default(); weighted_instance.adj_list.len()],
            adj_list: vec![FxHashMap::default(); weighted_instance.adj_list.len()],
            minimum_cost: 0,
            fixed_orders: HashMap::default(),
        };
        self_.compute_edges_v1(weighted_instance);
        self_
    }

    fn compute_edges_v1(&mut self, weighted_instance: &WeightedInstance) {
        let add_all_edges = true;
        let not_taken_infinite = false;

        let mut fixed_edges = vec![
            FixedEdges {
                forward: FxHashSet::default(),
                backward: FxHashSet::default(),
            };
            weighted_instance.n1
        ];

        for i in 0..weighted_instance.n1 {
            for j in (i + 1)..weighted_instance.n1 {
                let first = &weighted_instance.adj_list[i];
                let second = &weighted_instance.adj_list[j];
                let (mut left_cost, mut right_cost) = count_out_of_order_weight(first, second);
                left_cost *= weighted_instance.weights[i] * weighted_instance.weights[j];
                right_cost *= weighted_instance.weights[i] * weighted_instance.weights[j];
                // assert!(left_cost > 0 || right_cost > 0);

                const INFINITY: u32 = 100000000;

                if left_cost > right_cost {
                    if add_all_edges || right_cost != 0 {
                        let weight = if right_cost == 0 && not_taken_infinite {
                            INFINITY
                        } else {
                            left_cost - right_cost
                        };

                        self.adj_list[i].insert(j as u32, weight);
                        self.min_adj_list[j].insert(i as u32, weight);
                        self.minimum_cost += right_cost as u64;
                    }

                    if right_cost == 0 {
                        // removed_edges += 1;
                        self.fixed_orders
                            .entry(j as u32)
                            .or_default()
                            .push(i as u32);
                        fixed_edges[j].forward.insert(i as u32);
                        fixed_edges[i].backward.insert(j as u32);
                    }
                    // total_edges += 1;
                } else if left_cost < right_cost {
                    if add_all_edges || left_cost != 0 {
                        let weight = if left_cost == 0 && not_taken_infinite {
                            INFINITY
                        } else {
                            right_cost - left_cost
                        };

                        self.adj_list[j].insert(i as u32, weight);
                        self.min_adj_list[i].insert(j as u32, weight);
                        self.minimum_cost += left_cost as u64;
                    }

                    if left_cost == 0 {
                        // removed_edges += 1;
                        self.fixed_orders
                            .entry(i as u32)
                            .or_default()
                            .push(j as u32);
                        fixed_edges[i].forward.insert(j as u32);
                        fixed_edges[j].backward.insert(i as u32);
                    }
                    // total_edges += 1;
                }
            }
        }
    }

    pub fn compute_single_costs(&self, order: &[usize]) -> Vec<u64> {
        let mut order_costs = vec![];
        let mut tmp_used_nodes = FxHashSet::default();
        for el in order.iter().copied() {
            let mut node_cost = 0;
            for (&idx, &weight) in self.adj_list[el].iter() {
                if !tmp_used_nodes.contains(&(idx as usize)) {
                    node_cost += weight as u64;
                }
            }
            order_costs.push(node_cost);
            tmp_used_nodes.insert(el);
        }
        order_costs
    }

    pub fn compute_cost(&self, order: &[usize]) -> u64 {
        let mut order_cost = 0;
        let mut tmp_used_nodes = FxHashSet::default();

        // Sanity check
        {
            assert_eq!(order.len(), self.n1);
            let mut sorted = order.to_vec();
            sorted.sort_unstable();
            assert!(sorted.iter().copied().eq(0..self.n1));
        }

        for el in order.iter().copied() {
            for (&idx, &weight) in self.adj_list[el].iter() {
                if !tmp_used_nodes.contains(&(idx as usize)) {
                    order_cost += weight as u64;
                }
            }
            tmp_used_nodes.insert(el);
        }
        order_cost
    }

    pub fn get_paid_edges(&self, order: &[usize]) -> Vec<(usize, usize, u64)> {
        let mut paid_edges = vec![];
        let mut tmp_used_nodes = FxHashSet::default();
        for el in order.iter().copied() {
            for (&idx, &weight) in self.adj_list[el].iter() {
                if !tmp_used_nodes.contains(&(idx as usize)) {
                    paid_edges.push((el, idx as usize, weight as u64));
                }
            }
            tmp_used_nodes.insert(el);
        }
        paid_edges
    }

    pub fn slow_get_total_edges(&self) -> usize {
        let mut count = 0;
        for list in self.adj_list.iter() {
            count += list.len();
        }
        count
    }

    pub fn get_ordered_paid_edges(&self, order: &[usize]) -> Vec<(usize, usize, u64)> {
        let mut edges = self.get_paid_edges(&order);

        let mut nodes_positions = vec![0; order.len()];
        for (idx, el) in order.iter().copied().enumerate() {
            nodes_positions[el] = idx;
        }
        edges.sort_by_key(|e| nodes_positions[e.1] - nodes_positions[e.0]);
        edges
    }
}
