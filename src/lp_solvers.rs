use std::{fmt::Debug, time::Duration};

pub mod scip;

pub trait ModelBuilder {
    type Variable: Clone;

    fn add_variable(
        &mut self,
        name: &str,
        is_integer: bool,
        obj: f64,
        lb: f64,
        ub: f64,
    ) -> Self::Variable;
    fn add_constraint(
        &mut self,
        name: &str,
        vars: &[Self::Variable],
        coeffs: &[f64],
        lb: Option<f64>,
        ub: Option<f64>,
    );

    fn set_verbose(&mut self, verbose: bool);
}

pub trait ModelCreating {
    type Model;
    fn build(self) -> Self::Model;
}

pub trait ModelReady {
    type SolvedModel: ModelSolved;
    type Solution;
    fn solve(self) -> Self::SolvedModel;

    fn set_time_limit(self, duration: Duration) -> Self;

    fn create_solution(&mut self) -> Self::Solution;
    fn add_solution(&mut self, solution: Self::Solution) -> bool;
}

pub trait ModelSolved {
    type Solution: SolutionTrait;
    type Model: ModelBuilder;

    fn is_optimal(&self) -> bool;
    fn get_solutions(&self, max_count: usize) -> Vec<Self::Solution>;
    fn reset(self) -> Self::Model;
}

pub trait SolutionTrait {
    type Variable;
    fn get_value(&self, var: &Self::Variable) -> f64;
    fn get_values(&self, vars: &[Self::Variable]) -> Vec<f64>;
    fn set_value(&mut self, var: &Self::Variable, value: f64);
    fn get_objective_value(&self) -> f64;
}

pub trait VariableTrait {
    fn get_obj(&self) -> f64;
}

#[derive(Clone, Copy, Debug)]
pub enum ObjSense {
    Minimize,
    Maximize,
}

pub trait LpSolver: Clone + Debug {
    type Variable: Clone + Debug + VariableTrait;
    type Solution: SolutionTrait<Variable = Self::Variable>;
    type ModelInit: ModelBuilder<Variable = Self::Variable> + ModelCreating<Model = Self::Model>;
    type Model: ModelBuilder<Variable = Self::Variable>
        + ModelReady<SolvedModel = Self::SolvedModel, Solution = Self::Solution>;
    type SolvedModel: ModelSolved<Model = Self::Model, Solution = Self::Solution>;

    fn create_new_model(name: &str, verbose: bool, sense: ObjSense) -> Self::ModelInit;
}
