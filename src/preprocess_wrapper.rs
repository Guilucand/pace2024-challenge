use std::io::Write;
use std::{fs::File, io::BufWriter};

use rustc_hash::FxHashSet;

use crate::deltas::CostsDelta;
use crate::weighted::WeightedInstance;

pub enum SolveMode {
    Exact,
    Heuristic {
        max_chunk_size: usize,
        chunks_stride: usize,
    },
}

pub fn preprocess_and_solve_instance(
    output: File,
    solve_mode: SolveMode,
    weighted_instance: &WeightedInstance,
    optimal_order: Option<&[usize]>,
    mut solver: impl FnMut(&WeightedInstance, Option<&[usize]>, f64) -> Vec<usize>,
) {
    let candidate_order = match solve_mode {
        SolveMode::Exact => solver(weighted_instance, optimal_order, 1.0),
        SolveMode::Heuristic {
            max_chunk_size,
            chunks_stride,
        } => {
            let deltas = CostsDelta::new(&weighted_instance);
            let mut deltas = deltas
                .deltas
                .iter()
                .enumerate()
                .map(|(i, x)| (i, *x))
                .collect::<Vec<_>>();
            deltas.sort_by_key(|e| e.1);

            let mut candidate_order = vec![];
            for (idx, _card) in deltas.iter().copied() {
                candidate_order.push(idx);
            }

            let mut start = 0;
            loop {
                let chunk_start = start;
                let chunk_end = weighted_instance.n1.min(start + max_chunk_size);

                log::info!("Processing {}/{}", chunk_end, weighted_instance.n1);

                let instance_part = WeightedInstance::from_subset(
                    weighted_instance,
                    &candidate_order[chunk_start..chunk_end],
                );

                let elements = candidate_order[chunk_start..chunk_end]
                    .iter()
                    .collect::<FxHashSet<_>>();
                let part_solution = optimal_order.map(|o| {
                    instance_part.uniform_indices(
                        &o.iter()
                            .filter(|&x| elements.contains(x))
                            .copied()
                            .collect::<Vec<_>>(),
                    )
                });

                let new_order = solver(
                    &instance_part,
                    part_solution.as_deref(),
                    (chunk_end - chunk_start) as f64 / weighted_instance.n1 as f64,
                );

                let new_order = instance_part.expand_solution(&new_order);
                let replaced = &mut candidate_order[chunk_start..chunk_end];
                assert_eq!(new_order.len(), replaced.len());

                replaced.copy_from_slice(&new_order);

                if chunk_end == weighted_instance.n1 {
                    break;
                }
                start += chunks_stride.min(max_chunk_size);
            }
            candidate_order
        }
    };

    let mapped_final_order = weighted_instance.expand_solution(&candidate_order);

    let mut output_file = BufWriter::new(output);
    for i in 0..mapped_final_order.len() {
        // eprint!("{} ", true_optimal[i]);
        writeln!(output_file, "{}", mapped_final_order[i]).unwrap();
    }
    log::warn!("Instance solved!");
}
