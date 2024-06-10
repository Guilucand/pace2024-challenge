pub mod constraints_manager;

use std::{
    cell::RefCell,
    rc::Rc,
    time::{Duration, Instant},
};

use crate::{
    connected::find_ordered_connected_components,
    heuristic_v4::{
        execute_heuristic_v4, heuristic_v4_flows_to_constraints,
        step0::heuristic_v4_step0,
        utils::{EdgeType, OrderedEdgeSet},
    },
    lp_solvers::*,
    solver_v3::constraints_manager::ScipConstraintsManager,
    time_limit::TimeLimit,
    topological::TopologicalMapping,
    weighted::WeightedInstance,
};
use log::info;
use rustc_hash::FxHashMap;

fn float_equals(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-3
}

fn get_solution_values<Solver: LpSolver>(
    sol: &Solver::Solution,
    nodes: &Vec<Vec<NodeVariable<Solver>>>,
) -> Vec<Vec<f64>> {
    nodes
        .iter()
        .map(|x| {
            x.iter()
                .map(|y| match y {
                    NodeVariable::Fixed(value) => *value,
                    NodeVariable::Variable(v) => sol.get_value(v),
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>()
}

fn build_solution(instance: &TopologicalMapping, sol_values: &[Vec<f64>]) -> Vec<usize> {
    let cc = get_solution_ccs(instance, sol_values);
    cc.2.into_iter()
        .flat_map(|x| {
            let instance_part = WeightedInstance::from_subset(&instance.weighted_instance, &x);
            let topological = TopologicalMapping::new(&instance_part);
            let mut sorted_component = (0..instance_part.n1).collect::<Vec<_>>();
            sorted_component.sort_by_key(|v| topological.adj_list[*v].len());
            let base_ordering = heuristic_v4_step0(&topological, sorted_component);
            // let solution = solve_heuristic_v3(&topological, &base_ordering, None);
            instance_part.expand_solution(&base_ordering)
        })
        .collect()
}

fn get_solution_ccs(
    instance: &TopologicalMapping,
    sol_values: &[Vec<f64>],
) -> (u64, u64, Vec<Vec<usize>>) {
    // let mut nodes_positions = vec![0; instance.n1];

    let mut new_adj_list = vec![FxHashMap::default(); instance.n1];
    let mut new_min_adj_list = vec![FxHashMap::default(); instance.n1];

    let mut lp_cost = 0;

    let mut non_integer_count = 0;
    let mut vars_count = 0;

    let mut fractional_cost = 0.0;

    for i in 0..instance.n1 {
        for j in (i + 1)..instance.n1 {
            let edge_dir_cost = instance.adj_list[i].get(&(j as u32)).copied().unwrap_or(0) as f64
                - instance.adj_list[j].get(&(i as u32)).copied().unwrap_or(0) as f64;

            let i_before_j = if edge_dir_cost < 0.0 {
                sol_values[i][j - i - 1] < 0.1
            } else {
                sol_values[i][j - i - 1] < 0.9
            };

            if !float_equals(sol_values[i][j - i - 1], 0.0)
                && !float_equals(sol_values[i][j - i - 1], 1.0)
            {
                non_integer_count += 1;
            }
            vars_count += 1;

            if edge_dir_cost < 0.0 {
                fractional_cost -= edge_dir_cost;
            }

            fractional_cost += edge_dir_cost * sol_values[i][j - i - 1];

            if i_before_j {
                // nodes_positions[i] += 1;
                if let Some(edge_weight) = instance.adj_list[i].get(&(j as u32)) {
                    new_adj_list[i].insert(j as u32, *edge_weight);
                    new_min_adj_list[j].insert(i as u32, *edge_weight);
                } else if let Some(edge_weight) = instance.adj_list[j].get(&(i as u32)) {
                    lp_cost += *edge_weight as u64;
                }
            } else {
                // nodes_positions[j] += 1;
                if let Some(edge_weight) = instance.adj_list[j].get(&(i as u32)) {
                    new_adj_list[j].insert(i as u32, *edge_weight);
                    new_min_adj_list[i].insert(j as u32, *edge_weight);
                } else if let Some(edge_weight) = instance.adj_list[i].get(&(j as u32)) {
                    lp_cost += *edge_weight as u64;
                }
            }
        }
    }

    log::info!(
        "LP Solution non integers: {}/{} => {:.2}% fractional: {:.2} vs integer: {}",
        non_integer_count,
        vars_count,
        non_integer_count as f64 / vars_count as f64 * 100.0,
        fractional_cost,
        lp_cost
    );

    let new_instance = TopologicalMapping {
        weighted_instance: instance.weighted_instance.clone(),
        ocr: instance.ocr.clone(),
        n1: instance.n1,
        adj_list: new_adj_list,
        min_adj_list: new_min_adj_list,
        minimum_cost: instance.minimum_cost,
        fixed_orders: instance.fixed_orders.clone(),
    };

    let cc = find_ordered_connected_components(&new_instance, None);

    (
        non_integer_count,
        lp_cost,
        cc.into_iter()
            .map(|x| {
                if x.0.len() > 1 {
                    // Perform only basic reordering, not the full heuristic
                    heuristic_v4_step0(&new_instance, x.0)
                } else {
                    x.0
                }
            })
            .collect(),
    )
}

pub struct CostVars<Solver: LpSolver> {
    pub vars: Vec<Solver::Variable>,
    pub coeffs: Vec<f64>,
    pub sol_offset: u64,
}

pub struct SolveInfo {
    pub force_optimal: bool,
    pub best_cost: u64,
    pub best_solution: Vec<usize>,
    pub highest_lower_bound: u64,
}

const FORCE_SOLUTION_ANALYSIS: bool = true;

impl SolveInfo {
    pub fn add_solution_by_edges(
        &mut self,
        instance: &TopologicalMapping,
        sol_values: &[FxHashMap<u32, f64>],
    ) -> bool {
        let ccs: Vec<_> = {
            let mut new_instance = OrderedEdgeSet::new(instance.n1);

            for i in 0..sol_values.len() {
                for (&j, &value) in &sol_values[i] {
                    if value < 0.5 {
                        let cost_ij = instance.min_adj_list[i].get(&(j as u32)).copied().unwrap();
                        new_instance.edges[i].insert(j, EdgeType::Saved { cost: cost_ij });
                    }
                }
            }

            let cc = find_ordered_connected_components(&new_instance, None);

            cc.into_iter()
                .map(|x| {
                    if x.0.len() > 1 {
                        // Perform only basic reordering, not the full heuristic
                        heuristic_v4_step0(&instance, x.0)
                    } else {
                        x.0
                    }
                })
                .collect()
        };

        let largest_cc_len = ccs.iter().max_by_key(|x| x.len()).unwrap().len();

        log::info!("Building solution...");
        let new_solution = ccs
            .iter()
            .flat_map(|x| x.iter().copied())
            .collect::<Vec<_>>();
        let new_cost = instance.compute_cost(&new_solution);
        log::info!("Built solution!");

        if FORCE_SOLUTION_ANALYSIS || self.best_cost > new_cost {
            // let new_solution = improve_solution(&instance, &new_solution);
            let new_cost = instance.compute_cost(&new_solution);

            log::info!("Found better solution with cost {}", new_cost);

            let len1_comps = ccs.iter().filter(|x| x.len() == 1).count();

            log::info!(
                "Connected components sizes: [1] * {} + {:?}",
                len1_comps,
                ccs.iter()
                    .map(|x| x.len())
                    .filter(|x| *x > 1)
                    .collect::<Vec<_>>()
            );

            if self.best_cost > new_cost {
                self.best_cost = new_cost;
                self.best_solution = new_solution.clone();
            }
        }

        if self.best_cost <= self.highest_lower_bound {
            if self.best_cost < self.highest_lower_bound {
                info!(
                    "Bound error in cost computation: best {} is lower than lp lower bound: {}!",
                    self.best_cost, self.highest_lower_bound
                );
            }

            // Optimal solution found, exit
            return true;
        }

        if !self.force_optimal && largest_cc_len > instance.n1 / 4 {
            true
        } else {
            false
        }
    }

    pub fn add_solution(
        &mut self,
        instance: &TopologicalMapping,
        sol_values: &[Vec<f64>],
    ) -> (u64, bool) {
        let (non_integers, lp_cost, ccs) = get_solution_ccs(&instance, &sol_values);
        let largest_cc_len = ccs.iter().max_by_key(|x| x.len()).unwrap().len();

        log::info!("Building solution...");
        let new_solution = build_solution(&instance, &sol_values);
        let new_cost = instance.compute_cost(&new_solution);
        log::info!("Built solution!");

        if FORCE_SOLUTION_ANALYSIS || self.best_cost > new_cost {
            // let new_solution = improve_solution(&instance, &new_solution);
            let new_cost = instance.compute_cost(&new_solution);

            log::info!(
                "Found better solution with cost {} and lp cost: {}",
                new_cost,
                lp_cost
            );

            let len1_comps = ccs.iter().filter(|x| x.len() == 1).count();

            log::info!(
                "Connected components sizes: [1] * {} + {:?}",
                len1_comps,
                ccs.iter()
                    .map(|x| x.len())
                    .filter(|x| *x > 1)
                    .collect::<Vec<_>>()
            );

            if self.best_cost > new_cost {
                self.best_cost = new_cost;
                self.best_solution = new_solution.clone();
            }
        }

        if self.best_cost <= self.highest_lower_bound {
            if self.best_cost < self.highest_lower_bound {
                info!(
                    "Bound error in cost computation: best {} is lower than lp lower bound: {}!",
                    self.best_cost, self.highest_lower_bound
                );
            }

            // Optimal solution found, exit
            return (non_integers, true);
        }

        if !self.force_optimal && largest_cc_len > instance.n1 / 4 {
            (non_integers, true)
        } else {
            (non_integers, false)
        }
    }
}

#[derive(Clone, Debug)]
pub enum NodeVariable<Solver: LpSolver> {
    Fixed(f64),
    Variable(Solver::Variable),
}

impl<Solver: LpSolver> NodeVariable<Solver> {
    pub fn with_obj<T>(&self, func: impl FnOnce(f64) -> T, default: T) -> T {
        match self {
            NodeVariable::Fixed(_) => default,
            NodeVariable::Variable(var) => func(var.get_obj()),
        }
    }

    pub fn get_obj(&self) -> f64 {
        match self {
            NodeVariable::Fixed(_) => f64::INFINITY,
            NodeVariable::Variable(var) => var.get_obj(),
        }
    }
}

pub fn build_model<Solver: LpSolver>(
    instance: &TopologicalMapping,
    is_integer: bool,
    best_solution: Option<&[usize]>,
) -> (
    Solver::ModelInit,
    Vec<Vec<NodeVariable<Solver>>>,
    u64,
    Rc<RefCell<SolveInfo>>,
    CostVars<Solver>,
) {
    let mut model = Solver::create_new_model(
        "pace2024_exact_v3",
        match () {
            #[cfg(not(feature = "verbose"))]
            () => false,
            #[cfg(feature = "verbose")]
            () => is_integer,
        },
        ObjSense::Minimize,
    );

    let mut cost_vars = CostVars {
        vars: vec![],
        coeffs: vec![],
        sol_offset: 0,
    };

    // add decision variables.
    let nodes: Vec<_> = (0..instance.n1)
        .map(|i| {
            ((i + 1)..instance.n1)
                .map(|j| {
                    let cost_ij = instance.adj_list[i].get(&(j as u32)).copied().unwrap_or(0);
                    let cost_ji = instance.adj_list[j].get(&(i as u32)).copied().unwrap_or(0);

                    if instance
                        .fixed_orders
                        .get(&(i as u32))
                        .map(|x| x.contains(&(j as u32)))
                        .unwrap_or_default()
                    {
                        NodeVariable::Fixed(1.0)
                    } else if instance
                        .fixed_orders
                        .get(&(j as u32))
                        .map(|x| x.contains(&(i as u32)))
                        .unwrap_or_default()
                    {
                        NodeVariable::Fixed(0.0)
                    } else {
                        if cost_ij < cost_ji {
                            cost_vars.sol_offset += (cost_ji as u64) - (cost_ij as u64);
                        }

                        cost_vars
                            .coeffs
                            .push((cost_ij as i64 - cost_ji as i64) as f64);

                        let var = model.add_variable(
                            &format!("n{}<{}", i, j),
                            is_integer,
                            cost_ij as f64 - cost_ji as f64,
                            0.0,
                            1.0,
                        );
                        cost_vars.vars.push(var.clone());
                        NodeVariable::Variable(var)
                    }
                })
                .collect::<Vec<_>>()
        })
        .collect();

    let solve_info = Rc::new(RefCell::new(SolveInfo {
        force_optimal: false,
        best_cost: u64::MAX,
        best_solution: best_solution
            .map(|x| x.to_vec())
            .unwrap_or_else(|| (0..instance.n1).collect()),
        highest_lower_bound: 0,
    }));

    (model, nodes, cost_vars.sol_offset, solve_info, cost_vars)
}

pub fn solve_lp_v3<Solver: LpSolver>(
    instance: &TopologicalMapping,
    initial_order: &[usize],
    time_limit: &mut TimeLimit,
) -> Vec<usize> {
    let mut is_binary = false;

    let mut instance = instance.clone();

    // 1/3 of the time for the solve heuristic
    time_limit.start_proportion(0.33);
    let (best_solution, proof_flows) =
        execute_heuristic_v4(&mut instance, initial_order, time_limit);

    let instance = &instance;

    let lp_constraints = heuristic_v4_flows_to_constraints(&proof_flows);

    let (mut model, mut nodes, sol_offset, mut solve_info, mut _cost_vars) =
        build_model::<Solver>(instance, is_binary, Some(&best_solution));

    time_limit.start_proportion(0.5);
    let mut constraints_manager =
        ScipConstraintsManager::new(lp_constraints.into_iter(), instance, &best_solution);

    constraints_manager.reset_model(&nodes, &mut model);
    let mut model = model.build();

    // Set the first best solution
    {
        solve_info.borrow_mut().best_cost = instance.compute_cost(&best_solution);
        solve_info.borrow_mut().best_solution = best_solution;
    }

    let elapsed_time = Instant::now();

    time_limit.start_proportion(1.0);

    let mut solved_model;
    let sol = loop {
        let mut solve_info_mut = solve_info.borrow_mut();

        // // write constructed model to the file.
        if let Some(time_limit) = time_limit.remaining_time_minus(Duration::from_secs(10)) {
            model = model.set_time_limit(time_limit);
        }

        {
            let mut start_solution = model.create_solution();
            let best_sol = &solve_info_mut.best_solution;

            for i in 0..instance.n1 {
                for j in (i + 1)..instance.n1 {
                    let i_node = best_sol[i].min(best_sol[j]);
                    let j_node = best_sol[i].max(best_sol[j]);

                    // if i_node == best_sol[i] => var = 1 else 0

                    if let NodeVariable::Variable(node) = &nodes[i_node][j_node - i_node - 1] {
                        start_solution
                            .set_value(node, if i_node == best_sol[i] { 1.0 } else { 0.0 });
                    }
                }
            }
            if !model.add_solution(start_solution) {
                log::info!("Best solution already in solution set.");
            }
        }

        let elapsed_solve = Instant::now();

        drop(solve_info_mut);

        // optimize the model.
        log::info!("Started solving...");
        solved_model = model.solve();

        solve_info_mut = solve_info.borrow_mut();

        log::info!(
            "Solve time: {:.2?} total: {:.2?}",
            elapsed_solve.elapsed(),
            elapsed_time.elapsed()
        );

        let is_optimal = solved_model.is_optimal();

        if !is_optimal {
            log::info!("Partial solution found");
        }

        let sol = solved_model.get_solutions(1).swap_remove(0);

        let sol_values = get_solution_values(&sol, &nodes);
        let lp_sol_cost = sol_offset
            .checked_add_signed(sol.get_objective_value() as i64)
            .unwrap();

        model = solved_model.reset();

        // Add violated constraints
        let stats = constraints_manager.add_violated_constraints_v2(
            &nodes,
            &mut model,
            instance,
            &solve_info_mut.best_solution,
            &sol_values,
        );
        log::info!(
            "Violated: {}/{} total: [{}(+{})]",
            stats.added,
            stats.total,
            constraints_manager.total_added_constraints(),
            stats.added,
        );

        let mut nodes_positions = vec![0; instance.n1];
        {
            for i in 0..instance.n1 {
                for j in (i + 1)..instance.n1 {
                    if sol_values[i][j - i - 1] > 0.5 {
                        nodes_positions[j] += 1;
                    } else {
                        nodes_positions[i] += 1;
                    }
                }
            }
        }
        let mut subopt_partial_solution: Vec<_> = nodes_positions.into_iter().enumerate().collect();
        subopt_partial_solution.sort_unstable_by_key(|s| s.1 as u64);
        let subopt_solution_nodes = subopt_partial_solution
            .iter()
            .map(|s| s.0)
            .collect::<Vec<_>>();

        let (non_integers, _) = solve_info_mut.add_solution(instance, &sol_values);

        let subopt_sol_cost = instance.compute_cost(&subopt_solution_nodes);

        if subopt_sol_cost < solve_info_mut.best_cost {
            solve_info_mut.best_cost = subopt_sol_cost;
            solve_info_mut.best_solution = subopt_solution_nodes.clone();
        }

        if is_optimal {
            solve_info_mut.highest_lower_bound =
                solve_info_mut.highest_lower_bound.max(lp_sol_cost);
        }

        log::info!(
            "{}: {} Best: {} vs lpopt: {} => {:.2}% (2nd cost: {})",
            if is_optimal { "Optimal" } else { "Lp-partial" },
            lp_sol_cost,
            solve_info_mut.best_cost,
            solve_info_mut.highest_lower_bound,
            (solve_info_mut.best_cost - solve_info_mut.highest_lower_bound) as f64
                / solve_info_mut.highest_lower_bound as f64
                * 100.0,
            subopt_sol_cost
        );

        if solve_info_mut.highest_lower_bound >= solve_info_mut.best_cost {
            break solve_info_mut.best_solution.clone();
        }

        if stats.added == 0 {
            if is_optimal
                && solve_info_mut.highest_lower_bound < solve_info_mut.best_cost
                && !is_binary
                && non_integers > 0
            {
                drop(nodes);
                drop(model);
                // Switch to binary model if the continuous cannot find a valid solution
                let (mut new_model, new_nodes, _, new_info, new_cost_vars) =
                    build_model(instance, true, None);
                nodes = new_nodes;
                constraints_manager.reset_model(&nodes, &mut new_model);
                model = new_model.build();
                is_binary = true;
                log::info!(
                    "************************ SWITCH TO BINARY VARIABLES ************************"
                );

                let mut new_info_mut = new_info.borrow_mut();

                new_info_mut.best_cost = solve_info_mut.best_cost;
                new_info_mut.best_solution = solve_info_mut.best_solution.clone();
                new_info_mut.highest_lower_bound = solve_info_mut.highest_lower_bound;
                drop(solve_info_mut);
                drop(new_info_mut);
                solve_info = new_info;
                _cost_vars = new_cost_vars;
            } else {
                solve_info_mut.force_optimal = true;
            }
        } else {
            solve_info_mut.force_optimal = false;
        }
    };

    log::info!("Solution: {:?} in {:.2?}", sol, elapsed_time.elapsed());

    // for w in solution.windows(2) {
    //     assert!(w[0].1 == w[1].1 - 1);
    // }

    sol
}
