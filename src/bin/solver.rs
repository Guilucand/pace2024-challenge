use log::{debug, LevelFilter};
use pace_challenge_2024_solution::{
    heuristic_v4::{step0::heuristic_v4_step0, step1::heuristic_v4_step1},
    time_limit::TimeLimit,
};
use std::{
    fs::File,
    path::PathBuf,
    time::{Duration, Instant},
};
use structopt::StructOpt;

use pace_challenge_2024_solution::lp_solvers::scip::ScipSolver;

use pace_challenge_2024_solution::{
    connected::find_ordered_connected_components,
    parser::ParsedInstance,
    preprocess_wrapper::{preprocess_and_solve_instance, SolveMode},
    solver_v3::solve_lp_v3,
    topological::TopologicalMapping,
    weighted::WeightedInstance,
};

#[derive(StructOpt, Default)]
#[allow(dead_code)]
struct Args {
    input: PathBuf,
    output: PathBuf,

    #[structopt(short, long)]
    optimal: Option<PathBuf>,

    #[structopt(short, long)]
    heuristic: bool,

    #[structopt(short, long)]
    unlimited: bool,
}

// Mimalloc allocator
#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

struct ConsoleLogger {
    start: Instant,
}

impl log::Log for ConsoleLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::Level::Debug
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            println!(
                "{:.2?} {} - {}",
                self.start.elapsed(),
                record.level(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}

#[cfg(not(feature = "file_input"))]
fn optil_redirect() -> (File, File) {
    use std::{fs::OpenOptions, mem::forget, os::fd::FromRawFd};
    //~~~~ redirect fd 3 as stdin ~~~~
    let null_stdout = OpenOptions::new().write(true).open("/dev/null").unwrap();
    let null_stderr = OpenOptions::new().write(true).open("/dev/null").unwrap();

    if unsafe { libc::dup2(0, 3) } == -1 {
        panic!("cannot redirect stdin!");
    }
    let input_file = unsafe { File::from_raw_fd(3) };

    if unsafe { libc::dup2(1, 4) } == -1 {
        panic!("cannot redirect stdout!");
    }
    let output_file = unsafe { File::from_raw_fd(4) };

    unsafe {
        use std::os::fd::AsRawFd;
        // Redirect stdout and stderr to /dev/null
        libc::dup2(null_stdout.as_raw_fd(), 1);
        libc::dup2(null_stderr.as_raw_fd(), 2);
        forget(null_stdout);
        forget(null_stderr);
    }

    (input_file, output_file)
}

fn main() {
    let console_logger = ConsoleLogger {
        start: Instant::now(),
    };
    let console_logger = Box::leak(Box::new(console_logger));

    let _signals = signal_hook::iterator::Signals::new(&[libc::SIGINT, libc::SIGTERM]);

    // env_logger::init();
    log::set_logger(console_logger).unwrap();
    log::set_max_level(match () {
        #[cfg(feature = "verbose")]
        () => LevelFilter::Debug,
        #[cfg(not(feature = "verbose"))]
        () => LevelFilter::Warn,
    });

    debug!("Debug enabled");

    let args = match () {
        #[cfg(not(feature = "file_input"))]
        () => Args::default(),
        #[cfg(feature = "file_input")]
        () => Args::from_args(),
    };

    let is_heuristic = match () {
        #[cfg(not(feature = "heuristic"))]
        () => args.heuristic,
        #[cfg(feature = "heuristic")]
        () => true,
    };

    let mut time_limit = TimeLimit::new(if args.unlimited {
        None
    } else {
        if is_heuristic {
            Some(Duration::from_secs(292))
        } else {
            None // Some(Duration::from_secs(1795))
        }
    });

    let (input, output) = match () {
        #[cfg(not(feature = "file_input"))]
        () => optil_redirect(),
        #[cfg(feature = "file_input")]
        () => (
            File::open(&args.input).unwrap(),
            File::create(&args.output).unwrap(),
        ),
    };

    let instance = ParsedInstance::new(input);

    log::info!("Prev instance elements: {:?}", instance.n1);

    let weighted_instance = WeightedInstance::from_instance(&instance);
    log::info!("Instance elements: {:?}", weighted_instance.n1);

    let optimal_order = if let Some(optimal) = &args.optimal {
        Some(weighted_instance.uniform_indices(&ParsedInstance::read_order(optimal)))
    } else {
        None
    };

    preprocess_and_solve_instance(
        output,
        if is_heuristic {
            SolveMode::Heuristic {
                max_chunk_size: 1000,
                chunks_stride: 900,
            }
        } else {
            SolveMode::Exact
        },
        &weighted_instance,
        optimal_order.as_deref(),
        |weighted_instance, opt_order, ratio| {
            let mut candidate_order = vec![];

            let mapping = TopologicalMapping::new(&weighted_instance);
            log::info!("Minimum cost: {}", mapping.minimum_cost);

            let components = find_ordered_connected_components(&mapping, opt_order);

            // let tot_length = components

            for (component, optsol) in components {
                let optimal = !is_heuristic;
                if component.len() == 1 {
                    candidate_order.push(component[0]);
                } else if optimal {
                    log::info!("Solving component of size {}", component.len());
                    let instance_part =
                        WeightedInstance::from_subset(weighted_instance, &component);

                    let mut sorted_component = (0..instance_part.n1).collect::<Vec<_>>();
                    sorted_component.sort_by_key(|v| mapping.adj_list[*v].len());

                    let new_order = solve_lp_v3::<ScipSolver>(
                        &TopologicalMapping::new(&instance_part),
                        &sorted_component,
                        &mut time_limit,
                    );

                    let new_order = instance_part.expand_solution(&new_order);
                    for node in new_order {
                        candidate_order.push(node);
                    }
                } else {
                    let instance_part =
                        WeightedInstance::from_subset(weighted_instance, &component);
                    let mapping = TopologicalMapping::new(&instance_part);
                    let _optsol = optsol.map(|optsol| instance_part.uniform_indices(&optsol));

                    let mut sorted_component = (0..instance_part.n1).collect::<Vec<_>>();
                    sorted_component.sort_by_key(|v| mapping.adj_list[*v].len());
                    let order = (0..mapping.n1).collect::<Vec<_>>();
                    let order = heuristic_v4_step0(&mapping, order);

                    let order = if time_limit.can_progress_global() {
                        time_limit.start_proportion(
                            ratio * (component.len() as f64) / mapping.n1 as f64 * 0.9,
                        );
                        if time_limit.remaining_time_minus(Duration::from_secs(120))
                            == Some(Duration::ZERO)
                        {
                            log::info!("Fast run!");
                            // Little time remaining, perform fast analysis
                            heuristic_v4_step1(&mapping, &order, &mut time_limit, true)
                        } else {
                            heuristic_v4_step1(&mapping, &order, &mut time_limit, false)
                        }
                    } else {
                        log::info!("NOT ENOUGH TIME!");
                        order
                    };

                    let new_order = instance_part.expand_solution(&order);

                    candidate_order.extend(new_order);
                }
            }

            // mapping
            //     .adj_list
            //     .iter()
            //     .enumerate()
            //     .map(|(i, x)| (i, x.len() as i32))
            //     .collect::<Vec<_>>();

            candidate_order
        },
    )

    // Local optimization
    // loop {
    //     let mut changed = false;

    //     for start in 1..candidate_order.len() {
    //         for c in start..candidate_order.len() {
    //             let left = candidate_order[c - 1];
    //             let right = candidate_order[c];
    //             if mapping.adj_list[left].contains_key(&(right as u32)) {
    //                 candidate_order.swap(c - 1, c);
    //                 changed = true;
    //                 // log::info!("Swapping {} {}", left, right);
    //             } else {
    //                 break;
    //             }
    //         }
    //     }

    //     if !changed {
    //         break;
    //     }
    // }

    // log::info!("Mappings: {:?}", cards);
    // log::info!("FINAL Mappings: {:?}", final_order);
}
