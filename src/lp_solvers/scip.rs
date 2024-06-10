use std::rc::Rc;

use russcip::{
    Model, ProblemCreated, ProblemOrSolving, Solution, Solved, VarType, Variable, WithSolutions,
};

use super::{
    LpSolver, ModelBuilder, ModelCreating, ModelReady, ModelSolved, ObjSense, SolutionTrait,
    VariableTrait,
};

#[derive(Clone, Copy, Debug)]
pub struct ScipSolver;

impl SolutionTrait for Solution {
    type Variable = Rc<Variable>;

    fn get_value(&self, var: &Self::Variable) -> f64 {
        self.val(var.clone())
    }

    fn get_values(&self, vars: &[Self::Variable]) -> Vec<f64> {
        vars.iter().map(|v| self.val(v.clone())).collect()
    }

    fn get_objective_value(&self) -> f64 {
        self.obj_val()
    }

    fn set_value(&mut self, var: &Self::Variable, value: f64) {
        self.set_val(var.clone(), value);
    }
}

impl ModelBuilder for Model<ProblemCreated> {
    type Variable = Rc<Variable>;

    fn add_variable(
        &mut self,
        name: &str,
        is_integer: bool,
        obj: f64,
        lb: f64,
        ub: f64,
    ) -> Self::Variable {
        self.add_var(
            lb,
            ub,
            obj,
            name,
            if is_integer {
                VarType::Integer
            } else {
                VarType::Continuous
            },
        )
    }

    fn add_constraint(
        &mut self,
        name: &str,
        vars: &[Self::Variable],
        coeffs: &[f64],
        lb: Option<f64>,
        ub: Option<f64>,
    ) {
        self.add_cons(
            vars.to_vec(),
            coeffs,
            lb.unwrap_or(f64::NEG_INFINITY),
            ub.unwrap_or(f64::INFINITY),
            name,
        );
    }

    fn set_verbose(&mut self, verbose: bool) {
        if verbose {
            self.show_output();
        } else {
            self.hide_output();
        };
    }
}

impl ModelReady for Model<ProblemCreated> {
    type SolvedModel = Model<Solved>;
    type Solution = Solution;

    fn solve(self) -> Self::SolvedModel {
        self.solve()
    }

    fn set_time_limit(self, duration: std::time::Duration) -> Self {
        Model::set_time_limit(self, duration.as_secs_f64() as usize)
    }

    fn create_solution(&mut self) -> Self::Solution {
        self.create_sol()
    }

    fn add_solution(&mut self, solution: Self::Solution) -> bool {
        self.add_sol(solution).is_ok()
    }
}

impl ModelSolved for Model<Solved> {
    type Solution = Solution;
    type Model = Model<ProblemCreated>;

    fn is_optimal(&self) -> bool {
        self.status() == russcip::Status::Optimal
    }

    fn get_solutions(&self, max_count: usize) -> Vec<Self::Solution> {
        self.best_sol().into_iter().take(max_count).collect()
    }

    fn reset(self) -> Self::Model {
        self.free_transform()
    }
}

impl VariableTrait for Rc<Variable> {
    fn get_obj(&self) -> f64 {
        self.obj()
    }
}

impl ModelCreating for Model<ProblemCreated> {
    type Model = Model<ProblemCreated>;
    fn build(self) -> Self::Model {
        self
    }
}

impl LpSolver for ScipSolver {
    type Variable = Rc<Variable>;
    type Solution = Solution;

    type ModelInit = Model<ProblemCreated>;
    type Model = Model<ProblemCreated>;

    type SolvedModel = Model<Solved>;

    fn create_new_model(name: &str, verbose: bool, sense: ObjSense) -> Self::ModelInit {
        let mut model = Model::new().include_default_plugins().create_prob(name);
        if verbose {
            model.show_output()
        } else {
            model.hide_output()
        };

        let model = model.set_obj_sense(match sense {
            ObjSense::Minimize => russcip::ObjSense::Minimize,
            ObjSense::Maximize => russcip::ObjSense::Maximize,
        });

        model
    }
}
