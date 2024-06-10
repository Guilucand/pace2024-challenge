use crate::parser::ParsedInstance;

#[derive(Clone, Debug)]
pub struct WeightedInstance {
    pub ocr: String,
    pub n0: usize,
    pub n1: usize,
    pub weights: Vec<u32>,
    pub orig_mappings: Vec<Vec<usize>>,
    pub adj_list: Vec<Vec<u32>>,
    pub empty_nodes: Vec<usize>,
}

impl WeightedInstance {
    pub fn from_instance(instance: &ParsedInstance) -> Self {
        let mut empty_nodes = vec![];
        let mut sorted: Vec<_> = instance.adj_list.iter().cloned().enumerate().collect();
        sorted.iter_mut().for_each(|el| {
            el.1.sort();
        });
        sorted.sort_by_cached_key(|x| x.1.clone());

        sorted.retain(|e| {
            if e.1.len() > 0 {
                true
            } else {
                empty_nodes.push((e.0 + instance.n0) + 1);
                false
            }
        });

        let mut new_adj_list = vec![];
        let mut weights = vec![];
        let mut orig_mappings = vec![];

        if sorted.len() > 0 {
            new_adj_list.push(sorted[0].1.iter().copied().collect::<Vec<_>>());
            weights.push(1);
            // Restore 1-based vertices
            orig_mappings.push(vec![(sorted[0].0 + instance.n0) + 1]);
        }

        for el in sorted.windows(2) {
            if el[0].1 != el[1].1 {
                new_adj_list.push(el[1].1.clone());
                weights.push(1);
                orig_mappings.push(vec![(el[1].0 + instance.n0) + 1]);
            } else {
                *weights.last_mut().unwrap() += 1;
                orig_mappings
                    .last_mut()
                    .unwrap()
                    .push((el[1].0 + instance.n0) + 1);
            }
        }

        Self {
            ocr: instance.ocr.clone(),
            n0: instance.n0,
            n1: new_adj_list.len(),
            weights,
            orig_mappings,
            adj_list: new_adj_list,
            empty_nodes,
        }
    }

    pub fn from_subset(weighted_instance: &WeightedInstance, subset: &[usize]) -> Self {
        let mut weights = vec![];
        let mut orig_mappings = vec![];
        let subset_adj_list: Vec<_> = subset
            .iter()
            .map(|s| weighted_instance.adj_list[*s].clone())
            .collect();
        let empty_nodes = vec![];

        for &idx in subset {
            weights.push(weighted_instance.weights[idx]);
            orig_mappings.push(vec![idx]);
        }

        let self_ = WeightedInstance {
            ocr: weighted_instance.ocr.clone(),
            n0: weighted_instance.n0,
            n1: subset.len(),
            weights,
            orig_mappings,
            adj_list: subset_adj_list,
            empty_nodes,
        };

        self_
    }

    pub fn uniform_indices(&self, indices: &[usize]) -> Vec<usize> {
        let reverse_map = self
            .orig_mappings
            .iter()
            .enumerate()
            .map(|(idx, mapping)| mapping.iter().map(move |v| (*v, idx)))
            .flatten()
            .collect::<std::collections::HashMap<_, _>>();

        let mut sorted_indices = indices.to_vec();
        sorted_indices.sort();

        let mut sorted_map_keys = reverse_map.keys().copied().collect::<Vec<_>>();
        sorted_map_keys.sort();

        let mut mapping = indices
            .iter()
            .filter_map(|&x| reverse_map.get(&x).copied())
            .collect::<Vec<_>>();
        mapping.dedup();

        {
            let mut sorted = mapping.clone();
            sorted.sort();
            sorted.dedup();
            assert_eq!(sorted.len(), mapping.len());
            assert_eq!(sorted.len(), self.orig_mappings.len());
        }
        mapping
    }

    pub fn expand_solution(&self, ordering: &[usize]) -> Vec<usize> {
        let mut mapped_final_order = vec![];
        for &idx in ordering {
            mapped_final_order.extend(self.orig_mappings[idx].iter().copied())
        }
        for empty in &self.empty_nodes {
            mapped_final_order.push(*empty as usize);
        }
        mapped_final_order
    }
}
