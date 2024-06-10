use rand::{thread_rng, Rng};
use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    connected::GraphRepresentation, lp_solvers::LpSolver, topological::TopologicalMapping,
};

use super::{ModelBuilder, NodeVariable};

pub struct ViolatedConstraintsStats {
    pub added: usize,
    pub total: usize,
}

type Priority = u64;

#[derive(Default)]
struct DfsData {
    visited: Vec<bool>,
    on_stack: Vec<bool>,
    stack: Vec<u32>,
}

pub struct ScipConstraintsManager<Solver: LpSolver> {
    added_constraint_triplets: FxHashSet<(usize, usize, usize)>,
    chokepoint_nodes: FxHashMap<usize, Priority>,
    _phantom: std::marker::PhantomData<Solver>,
}

impl<Solver: LpSolver> ScipConstraintsManager<Solver> {
    pub fn new(
        start_constraints: impl Iterator<Item = (u32, u32, u32)>,
        instance: &TopologicalMapping,
        start_solution: &[usize],
    ) -> Self {
        let mut added_constraint_triplets = FxHashSet::default();

        for (i, j, k) in start_constraints {
            added_constraint_triplets.insert((i as usize, j as usize, k as usize));
        }

        let mut chokepoint_nodes = FxHashMap::default();

        let solution_edges = instance.get_ordered_paid_edges(start_solution);

        let mut incident_edges = vec![0; instance.nodes_count()];

        for &(i, j, _) in &solution_edges {
            incident_edges[i] += 1;
            incident_edges[j] += 1;
        }

        let mut rev_incident_edges = incident_edges.clone();

        for (i, j, _) in solution_edges {
            rev_incident_edges[i] -= 1;
            rev_incident_edges[j] -= 1;

            if chokepoint_nodes.contains_key(&i) || chokepoint_nodes.contains_key(&j) {
                continue;
            }

            let i_incident = incident_edges[i];
            let j_incident = incident_edges[j];

            if rev_incident_edges[i] > rev_incident_edges[j] {
                chokepoint_nodes.insert(i, i_incident as Priority);
            } else if j_incident > i_incident {
                chokepoint_nodes.insert(j, j_incident as Priority);
            }
        }

        ScipConstraintsManager {
            added_constraint_triplets,
            chokepoint_nodes,
            _phantom: std::marker::PhantomData,
        }
    }

    fn find_cycles(graph: &[Vec<u32>], dfs_data: &mut DfsData, node: u32) -> Option<u32> {
        if dfs_data.on_stack[node as usize] {
            // Found a cycle
            return Some(node);
        }

        if dfs_data.visited[node as usize] {
            return None;
        }

        dfs_data.on_stack[node as usize] = true;
        dfs_data.stack.push(node);

        for &next in &graph[node as usize] {
            if let Some(cycle_node) = Self::find_cycles(graph, dfs_data, next) {
                dfs_data.on_stack[node as usize] = false;
                dfs_data.visited[node as usize] = false;
                return Some(cycle_node);
            }
        }

        dfs_data.visited[node as usize] = true;
        dfs_data.on_stack[node as usize] = false;
        dfs_data.stack.pop();
        None
    }

    fn add_cycle_constraint(
        &mut self,
        nodes: &Vec<Vec<NodeVariable<Solver>>>,
        model: &mut impl ModelBuilder<Variable = Solver::Variable>,
        i: usize,
        j: usize,
        k: usize,
    ) -> bool {
        if self.added_constraint_triplets.contains(&(i, j, k)) {
            return false;
        }
        // log::info!("Add constraint for {} {} {}", i, j, k);
        let mut vars = vec![];
        let mut coefs = vec![];
        let mut offset = 0.0;

        assert!(i < j && j < k);

        for (node, coef) in [
            nodes[i][j - i - 1].clone(),
            nodes[j][k - j - 1].clone(),
            nodes[i][k - i - 1].clone(),
        ]
        .into_iter()
        .zip([1.0, 1.0, -1.0].into_iter())
        {
            match node {
                NodeVariable::Fixed(value) => {
                    offset += coef * value;
                }
                NodeVariable::Variable(var) => {
                    vars.push(var);
                    coefs.push(coef);
                }
            }
        }

        self.added_constraint_triplets.insert((i, j, k));

        let _new_constr = model.add_constraint(
            &format!("ltriangle_{}_{}_{}", i, j, k),
            &vars,
            &coefs,
            Some(0.0 - offset),
            Some(1.0 - offset),
        );
        true
    }

    pub fn reset_model(
        &mut self,
        nodes: &Vec<Vec<NodeVariable<Solver>>>,
        new_model: &mut Solver::ModelInit,
    ) {
        let constraints: Vec<_> = self.added_constraint_triplets.drain().collect();

        for (i, j, k) in constraints {
            self.add_cycle_constraint(nodes, new_model, i, j, k);
        }
    }

    pub fn total_added_constraints(&self) -> usize {
        self.added_constraint_triplets.len()
    }

    fn get_paid_edges(
        &self,
        instance: &TopologicalMapping,
        sol_values: &[Vec<f64>],
    ) -> Vec<(usize, usize, u64)> {
        let mut paid_edges = vec![];
        for i in 0..sol_values.len() {
            for j in (i + 1)..sol_values.len() {
                let cost_ij = instance.adj_list[i].get(&(j as u32)).copied().unwrap_or(0);
                let cost_ji = instance.adj_list[j].get(&(i as u32)).copied().unwrap_or(0);
                // assert!(cost_ij == 0 || cost_ji == 0);

                let is_zero = cost_ij == 0 && cost_ji == 0;

                let i_before_j = sol_values[i][j - i - 1] > 0.5;

                // if i_before_j pay cost_ij (true, != 0) = is_paid

                let is_paid = i_before_j ^ (cost_ij == 0);
                if is_paid && !is_zero {
                    if i_before_j {
                        paid_edges.push((i, j, cost_ij as u64));
                    } else {
                        paid_edges.push((j, i, cost_ji as u64));
                    }
                }
            }
        }

        paid_edges
    }

    pub fn add_violated_constraints_v2(
        &mut self,
        nodes: &Vec<Vec<NodeVariable<Solver>>>,
        model: &mut Solver::Model,

        instance: &TopologicalMapping,
        _best_solution: &[usize],
        sol_values: &[Vec<f64>],
    ) -> ViolatedConstraintsStats {
        let mut stats = ViolatedConstraintsStats { added: 0, total: 0 };

        let paid_edges = self.get_paid_edges(instance, sol_values);

        let paid_edges_set: FxHashSet<_> = paid_edges
            .iter()
            .map(|(i, j, _)| (*i as u32, *j as u32))
            .collect();

        let mut graph = vec![vec![]; instance.nodes_count()];
        for (i, list) in instance.min_adj_list.iter().enumerate() {
            for (&j, _) in list {
                if !paid_edges_set.contains(&(j, i as u32)) {
                    graph[i].push(j as u32);
                }
            }
        }

        let mut dfs_data = DfsData::default();
        dfs_data.visited = vec![false; instance.nodes_count()];
        dfs_data.on_stack = vec![false; instance.nodes_count()];
        let mut cycles_found = vec![];
        for node in 0..graph.len() {
            while let Some(start_node) = {
                dfs_data.stack.clear();
                assert!(dfs_data.on_stack.iter().all(|&v| !v));
                Self::find_cycles(&graph, &mut dfs_data, node as u32)
            } {
                let mut cycle = vec![];
                while let Some(node) = dfs_data.stack.pop() {
                    cycle.push(node);
                    if node == start_node {
                        break;
                    }
                }
                cycle.reverse();

                // Remove one edge uniformly at random
                let edge_to_remove = thread_rng().gen_range(0..cycle.len());
                let i = cycle[edge_to_remove];
                let j = cycle[(edge_to_remove + 1) % cycle.len()];
                let start_edges = graph[i as usize].len();
                graph[i as usize].retain(|v| *v != j);
                let end_edges = graph[i as usize].len();
                assert_eq!(start_edges, end_edges + 1);
                cycles_found.push(cycle);
            }
        }

        let mut needed_triplets = vec![];

        let cycles_count = cycles_found.len();

        for mut cycle in cycles_found {
            let chokepoint = *cycle
                .iter()
                .max_by_key(|&&v| {
                    self.chokepoint_nodes
                        .get(&(v as usize))
                        .unwrap_or(&0)
                        .clone()
                })
                .unwrap();

            cycle.push(cycle[0]);

            for window in cycle.windows(2) {
                let mut i = window[0] as usize;
                let mut j = window[1] as usize;
                let mut k = chokepoint as usize;

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

        let all_triplets = needed_triplets.len();

        needed_triplets.sort_unstable();
        needed_triplets.dedup();

        for (i, j, k) in needed_triplets {
            stats.total += 1;

            if self.add_cycle_constraint(nodes, model, i, j, k) {
                stats.added += 1;
            }
        }

        log::info!(
            "Edges count: {} cycles found: {} needed triplets: {}/{} added: {}",
            graph.iter().map(|v| v.len()).sum::<usize>(),
            cycles_count,
            stats.total,
            all_triplets,
            stats.added
        );

        stats
    }
}
