use crate::weighted::WeightedInstance;

pub struct CostsDelta {
    pub deltas: Vec<i32>,
    pub left_costs: Vec<u32>,
    pub right_costs: Vec<u32>,
}

impl CostsDelta {
    pub fn new(weighted_instance: &WeightedInstance) -> Self {
        let mut inv_adjlist = vec![vec![]; weighted_instance.n0];

        for (n1_node, edges) in weighted_instance.adj_list.iter().enumerate() {
            for &n0_node in edges {
                inv_adjlist[n0_node as usize].push(n1_node as u32);
            }
        }

        let mut left_total_costs = 0;
        let mut right_total_costs = 0;

        let mut nodes_left_costs = vec![0; weighted_instance.n1];
        let mut nodes_right_costs = vec![0; weighted_instance.n1];

        for edges in inv_adjlist.iter().rev() {
            for n1_node in edges {
                right_total_costs += weighted_instance.weights[*n1_node as usize];
            }
        }

        for edges in inv_adjlist.iter() {
            for n1_node in edges {
                right_total_costs -= weighted_instance.weights[*n1_node as usize];
            }

            for &n1_node in edges {
                nodes_right_costs[n1_node as usize] += right_total_costs;
                nodes_left_costs[n1_node as usize] += left_total_costs;
            }

            // At the end add the cost of this node to the left costs
            for n1_node in edges {
                left_total_costs += weighted_instance.weights[*n1_node as usize];
            }
        }

        let delta: Vec<_> = nodes_left_costs
            .iter()
            .zip(nodes_right_costs.iter())
            .map(|(l, r)| (*l as i32) - (*r as i32))
            .collect();

        Self {
            deltas: delta,
            left_costs: nodes_left_costs,
            right_costs: nodes_right_costs,
        }
    }
}
