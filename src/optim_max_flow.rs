use std::collections::BinaryHeap;

use rustc_hash::{FxHashMap, FxHashSet};

use crate::{
    heuristic_v4::utils::EdgeType,
    max_flow_basic::{FlowGraph, SinkType},
};

type Node = u32;
type Weight = u32;
type AdjRef = u32;
type Epoch = u32;

const EPOCH_DIRECTION_FLAG: u32 = 0x80000000;
const EPOCH_MASK: u32 = 0x7FFFFFFF;
#[derive(Copy, Clone, Debug)]
struct EdgeInfo {
    fwd_weight__: Weight,
    bwd_weight__: Weight,
    epoch: Epoch,
}

impl EdgeInfo {
    fn create(weight: Weight, source: Node, target: Node) -> Self {
        let mut fwd_weight = weight;
        let mut bwd_weight = 0;
        let mut epoch = 0;

        // If source < target, fwd_weight should be > 0 and bwd_weight = 0

        if source > target {
            std::mem::swap(&mut fwd_weight, &mut bwd_weight);
            epoch |= EPOCH_DIRECTION_FLAG;
        }

        Self {
            fwd_weight__: fwd_weight,
            bwd_weight__: bwd_weight,
            epoch,
        }
    }

    fn total_weight(&self) -> Weight {
        self.fwd_weight__ + self.bwd_weight__
    }

    fn get_weight(&self, source: Node, dest: Node, epoch: Epoch) -> Weight {
        let is_epoch_valid = epoch == (self.epoch & EPOCH_MASK);

        if is_epoch_valid {
            if source < dest {
                self.fwd_weight__
            } else {
                self.bwd_weight__
            }
        } else {
            // Move all the weight to fwd or bwd
            if (source < dest) ^ ((self.epoch & EPOCH_DIRECTION_FLAG) != 0) {
                self.fwd_weight__ + self.bwd_weight__
            } else {
                0
            }
        }
    }

    fn use_flow(&mut self, source: Node, dest: Node, amount: Weight, epoch: Epoch) {
        let is_epoch_valid = epoch == (self.epoch & EPOCH_MASK);

        if !is_epoch_valid {
            self.reset(epoch);
        }

        if source < dest {
            self.fwd_weight__ -= amount;
            self.bwd_weight__ += amount;
        } else {
            self.bwd_weight__ -= amount;
            self.fwd_weight__ += amount;
        }
    }

    fn permanent_remove_flow_generic(&mut self, amount: Weight) {
        if self.epoch & EPOCH_DIRECTION_FLAG != 0 {
            self.bwd_weight__ = self.total_weight() - amount;
            self.fwd_weight__ = 0;
        } else {
            self.fwd_weight__ = self.total_weight() - amount;
            self.bwd_weight__ = 0;
        }
    }

    fn permanent_remove_flow(&mut self, source: Node, dest: Node, amount: Weight, epoch: Epoch) {
        assert_eq!(epoch, self.epoch & EPOCH_MASK);
        // Remove backward flow
        if source < dest {
            assert!(self.bwd_weight__ >= amount);
            self.bwd_weight__ -= amount;
        } else {
            assert!(self.fwd_weight__ >= amount);
            self.fwd_weight__ -= amount;
        }
    }

    fn add_back_flow(&mut self, source: Node, dest: Node, amount: Weight) {
        // Add back flow
        if source > dest {
            self.bwd_weight__ += amount;
        } else {
            self.fwd_weight__ += amount;
        }
    }

    fn reset(&mut self, epoch: Epoch) {
        self.epoch = epoch | (self.epoch & EPOCH_DIRECTION_FLAG);
        self.fwd_weight__ = self.fwd_weight__ + self.bwd_weight__;
        self.bwd_weight__ = 0;
        if (self.epoch & EPOCH_DIRECTION_FLAG) != 0 {
            std::mem::swap(&mut self.fwd_weight__, &mut self.bwd_weight__);
        }
    }

    fn is_inverted(&self) -> bool {
        self.epoch & EPOCH_DIRECTION_FLAG != 0
    }
}

#[derive(Clone, Debug)]
struct ParentNode {
    epoch: Epoch,
    node: Node,
    edge_idx: AdjRef,
    max_flow: Weight,
}

impl ParentNode {
    fn is_set(&self, epoch: Epoch) -> bool {
        self.epoch == epoch
    }

    fn set_parent(&mut self, node: Node, epoch: Epoch) {
        self.node = node;
        self.epoch = epoch;
    }
}

#[derive(Clone, Debug)]
pub struct MaxFlowCache {
    queue: BinaryHeap<(Weight, Node)>,
    parent: Vec<ParentNode>,
    cache_epoch: Epoch,
    saved_edges: Option<Vec<FxHashMap<Node, AdjRef>>>,
    internal_clear_saved_edges: bool,
    flow_graph: Option<Vec<Vec<(Node, Weight)>>>,
}

#[derive(Clone, Debug)]
pub struct NodeAdj {
    edges: Vec<Option<AdjRef>>,
    adj_list: Vec<(Node, AdjRef)>,
    active_edges: usize,
}

#[derive(Clone, Debug)]
pub struct OptimMaxFlow {
    nodes_adjmat: Vec<NodeAdj>,
    edges: Vec<EdgeInfo>,
    flow_epoch: Epoch,
}

#[derive(Clone, Debug)]
pub enum MinimumRequiredFlowAnalysis {
    Done,
    ImprovementPossible {
        to_pay: Vec<(Node, Node)>,
        to_save: Vec<(Node, Node)>,
    },
}

impl OptimMaxFlow {
    pub fn new<F: FlowGraph + ?Sized>(flow_graph: &F) -> Self {
        let nodes_count = flow_graph.get_node_count();
        let mut edges = vec![];
        let mut nodes_adjmat = (0..nodes_count)
            .map(|_| NodeAdj {
                edges: vec![None; nodes_count],
                adj_list: vec![],
                active_edges: 0,
            })
            .collect::<Vec<_>>();

        for u in 0..nodes_count {
            for (v, weight) in flow_graph.get_node_adj_list(u) {
                if weight == 0 {
                    // Skip zero weight edges (taken ones)
                    continue;
                }
                let edge_idx = edges.len() as AdjRef;
                edges.push(EdgeInfo::create(weight as u32, u as Node, v as Node));
                nodes_adjmat[u].edges[v] = Some(edge_idx);
                nodes_adjmat[u].adj_list.push((v as Node, edge_idx));

                nodes_adjmat[u].active_edges += 1;
                nodes_adjmat[v].edges[u] = Some(edge_idx);
                nodes_adjmat[v].adj_list.push((u as Node, edge_idx));
            }
        }

        Self {
            nodes_adjmat,
            edges,
            flow_epoch: 0,
        }
    }

    pub fn sanity_check(&self) {
        for edge in &self.edges {
            assert!(edge.total_weight() > 0);
        }
    }

    pub fn create_cache(&self) -> MaxFlowCache {
        MaxFlowCache {
            queue: BinaryHeap::with_capacity(self.nodes_adjmat.len()),
            parent: vec![
                ParentNode {
                    epoch: 0,
                    node: 0,
                    edge_idx: 0,
                    max_flow: 0
                };
                self.nodes_adjmat.len()
            ],
            cache_epoch: 0,
            saved_edges: None,
            internal_clear_saved_edges: true,
            flow_graph: None,
        }
    }

    pub fn compute_max_flow_directed(
        &mut self,
        flow_graph: &impl FlowGraph,
        cache: &mut MaxFlowCache,
        source: usize,
        sink: SinkType,
        compute_min_cut: bool,
        limit: Option<u64>,
        allowed_nodes: Option<&[usize]>,
        // threshold: Option<NodesThreshold>,
    ) -> (i64, u64, Option<Vec<(usize, usize)>>, Vec<(usize, usize)>) {
        let SinkType::Single(sink) = sink else {
            panic!("Only single sink supported");
        };

        let nodes_count = flow_graph.get_node_count();

        let max_flow = self.compute_max_flow(
            source as Node,
            sink as Node,
            cache,
            limit,
            false,
            allowed_nodes,
        ) as u64;

        let mut tot_flow = max_flow as i64;

        let mut removed_edges = vec![];

        let min_cut = if compute_min_cut {
            let mut reachable_nodes = FxHashSet::default();
            let mut visited = vec![false; nodes_count];
            let mut stack = vec![source];
            while let Some(node) = stack.pop() {
                if visited[node] {
                    continue;
                }
                visited[node] = true;
                reachable_nodes.insert(node);
                for &(head, edge_idx) in self.nodes_adjmat[node].adj_list.iter() {
                    let weight = self.edges[edge_idx as usize].get_weight(
                        node as Node,
                        head as Node,
                        self.flow_epoch,
                    );

                    if weight > 0 {
                        stack.push(head as usize);
                    }
                }
            }

            let mut min_cut = FxHashSet::default();

            for &node in reachable_nodes.iter() {
                for (edge, _) in flow_graph.get_node_adj_list(node) {
                    if !reachable_nodes.contains(&edge) {
                        if let Some(edge_type) = flow_graph.get_edge_type(node, edge) {
                            match edge_type {
                                EdgeType::Paid { cost } => {
                                    removed_edges.push((node, edge));
                                    tot_flow = tot_flow - cost as i64;
                                }
                                EdgeType::Saved { .. } => {
                                    min_cut.insert((node, edge));
                                }
                            }
                        }
                    }
                }
            }

            Some(min_cut.into_iter().collect())
        } else {
            None
        };

        (tot_flow, max_flow, min_cut, removed_edges)
    }

    #[inline(always)]
    fn retrieve_last_used_flow(&self, source: Node, target: Node, edge_idx: AdjRef) -> Weight {
        // Get the reverse edge, that corresponds to the used capacity
        // let is_inverted = self.edges[edge_idx as usize].epoch & EPOCH_DIRECTION_FLAG != 0;
        self.edges[edge_idx as usize].get_weight(target, source, self.flow_epoch)
    }

    pub fn get_flow_graph<'a>(
        &self,
        cache: &'a mut MaxFlowCache,
        source: Node,
    ) -> &'a [Vec<(Node, Weight)>] {
        let mut flow_graph = vec![vec![]; self.nodes_adjmat.len()];
        let mut visited = vec![false; self.nodes_adjmat.len()];

        let mut stack = vec![source];
        visited[source as usize] = true;

        let used_edges_graph = cache.saved_edges.as_ref().unwrap();

        while let Some(node) = stack.pop() {
            for (&head, &edge_idx) in &used_edges_graph[node as usize] {
                let weight = self.retrieve_last_used_flow(node, head, edge_idx);
                if weight > 0 {
                    let has_node = flow_graph[node as usize].iter().any(|(n, _)| *n == head);

                    assert!(!has_node);

                    flow_graph[node as usize].push((head, weight));

                    if !visited[head as usize] {
                        stack.push(head);
                        visited[head as usize] = true;
                    }
                }
            }
        }

        cache.flow_graph = Some(flow_graph);
        cache.flow_graph.as_ref().unwrap()
    }

    pub fn remove_flow(&mut self, graph: &[Vec<(Node, Weight)>]) {
        for node in 0..self.nodes_adjmat.len() {
            for (head, weight) in &graph[node] {
                let edge_idx = self.nodes_adjmat[node].edges[*head as usize].unwrap();

                self.edges[edge_idx as usize].permanent_remove_flow(
                    node as u32,
                    *head as u32,
                    *weight,
                    self.flow_epoch,
                );
            }
        }
    }

    pub fn remove_generic_flow(&mut self, graph: &[Vec<(Node, Weight)>]) {
        for node in 0..self.nodes_adjmat.len() {
            for (head, weight) in &graph[node] {
                let edge_idx = self.nodes_adjmat[node].edges[*head as usize].unwrap();

                self.edges[edge_idx as usize].permanent_remove_flow_generic(*weight);
            }
        }
    }

    pub fn get_nodes_count(&self) -> usize {
        self.nodes_adjmat.len()
    }

    pub fn get_edges_count(&self) -> usize {
        self.edges.len()
    }

    pub fn get_edge_index(&self, source: Node, target: Node) -> Option<AdjRef> {
        self.nodes_adjmat[source as usize].edges[target as usize]
    }

    pub fn get_edge_capacity(&self, edge_idx: AdjRef) -> Weight {
        self.edges[edge_idx as usize].total_weight()
    }

    pub fn saturating_decrease_capacity(&mut self, edge_idx: AdjRef, amount: u32) {
        let edge = &mut self.edges[edge_idx as usize];
        let total_weight = edge.total_weight();
        let amount = total_weight.saturating_sub(amount);
        if edge.epoch & EPOCH_DIRECTION_FLAG != 0 {
            edge.bwd_weight__ = amount;
            edge.fwd_weight__ = 0;
        } else {
            edge.fwd_weight__ = amount;
            edge.bwd_weight__ = 0;
        }
    }

    pub fn remove_edge_idx(&mut self, edge_idx: AdjRef) {
        let edge = &mut self.edges[edge_idx as usize];

        // Clear the capacity
        edge.bwd_weight__ = 0;
        edge.fwd_weight__ = 0;
    }

    pub fn remove_edge(&mut self, source: Node, target: Node) {
        let edge_idx = self.nodes_adjmat[source as usize].edges[target as usize].unwrap();

        let edge = &mut self.edges[edge_idx as usize];

        // Clear the capacity
        edge.bwd_weight__ = 0;
        edge.fwd_weight__ = 0;
    }

    pub fn add_back_flow(&mut self, graph: &[Vec<(Node, Weight)>]) {
        for node in 0..self.nodes_adjmat.len() {
            for (head, weight) in &graph[node] {
                let edge_idx = self.nodes_adjmat[node].edges[*head as usize].unwrap();

                self.edges[edge_idx as usize].add_back_flow(node as u32, *head as u32, *weight);
            }
        }
    }

    pub fn add_back_reverse_flow(
        &mut self,
        cache: &mut MaxFlowCache,
        graph: &[Vec<(Node, Weight)>],
        excluded_edge: AdjRef,
    ) {
        self.flow_epoch += 2;

        cache.saved_edges = Some(vec![FxHashMap::default(); self.nodes_adjmat.len()]);
        cache.internal_clear_saved_edges = false;

        let mut found = false;

        for node in 0..self.nodes_adjmat.len() {
            for (head, weight) in &graph[node] {
                let edge_idx = self.nodes_adjmat[node].edges[*head as usize].unwrap();

                self.edges[edge_idx as usize].reset(self.flow_epoch);
                if excluded_edge == edge_idx {
                    found = true;
                }

                self.edges[edge_idx as usize].add_back_flow(*head, node as u32, *weight);
                cache.saved_edges.as_mut().unwrap()[node].insert(*head, edge_idx);
            }
        }

        assert!(found);

        // Do not invalidate the changed edges in the next flow computation!
        self.flow_epoch -= 1;
    }

    pub fn find_single_path(
        &mut self,
        source: Node,
        target: Node,
        cache: &mut MaxFlowCache,
        allowed_nodes: Option<&[usize]>,
    ) -> Option<Vec<(Node, Node)>> {
        self.flow_epoch += 1;
        self.find_path(cache, source, target, allowed_nodes);
        if !cache.parent[target as usize].is_set(cache.cache_epoch) {
            return None;
        }

        let mut current = target;
        let mut path = vec![];
        loop {
            let parent = &cache.parent[current as usize];

            if current == source {
                break;
            }
            path.push((parent.node, current));
            current = parent.node;
        }
        path.reverse();
        Some(path)
    }

    pub fn debug_inc_epoch(&mut self) {
        self.flow_epoch += 1;
    }

    /*
       FLOW RELATED FUNCTIONS
    */
    pub fn compute_max_flow(
        &mut self,
        source: Node,
        target: Node,
        cache: &mut MaxFlowCache,
        limit: Option<u64>,
        save_edges: bool,
        allowed_nodes: Option<&[usize]>,
    ) -> Weight {
        self.flow_epoch += 1;

        if save_edges {
            if cache.internal_clear_saved_edges {
                cache.saved_edges = Some(vec![FxHashMap::default(); self.nodes_adjmat.len()]);
            }
            cache.internal_clear_saved_edges = true;
        }

        let mut max_flow = 0;

        if limit == Some(0) {
            return 0;
        }

        loop {
            self.find_path(cache, source, target, allowed_nodes);
            if !cache.parent[target as usize].is_set(cache.cache_epoch) {
                break;
            }

            let parent_ref = &cache.parent[target as usize];
            max_flow += parent_ref.max_flow;

            let mut current = target;

            let flow_limit = limit.unwrap_or(u64::MAX);

            let usable_flow = if (max_flow as u64) > flow_limit {
                let extra_flow = max_flow - (flow_limit as u32);
                parent_ref.max_flow - extra_flow
            } else {
                parent_ref.max_flow
            };

            // let mut length = 0;
            loop {
                if current == source {
                    break;
                }

                let parent = &cache.parent[current as usize];
                // length += 1;

                self.edges[parent.edge_idx as usize].use_flow(
                    parent.node,
                    current,
                    usable_flow,
                    self.flow_epoch,
                );

                if save_edges {
                    let (ss, st) = if (parent.node < current)
                        ^ self.edges[parent.edge_idx as usize].is_inverted()
                    {
                        (parent.node, current)
                    } else {
                        (current, parent.node)
                    };

                    cache.saved_edges.as_mut().unwrap()[ss as usize].insert(st, parent.edge_idx);
                }

                current = parent.node;
            }

            if max_flow as u64 >= flow_limit {
                break;
            }
        }
        max_flow.min(limit.unwrap_or(u64::MAX) as u32)
    }
    /*
       BFS RELATED FUNCTIONS
    */

    #[inline(always)]
    fn explore_edge(
        &self,
        cache: &mut MaxFlowCache,
        edge_index: AdjRef,
        max_flow: Weight,
        element: Node,
        edge_target: Node,
    ) -> bool {
        if max_flow == 0 {
            return false;
        }

        if cache.parent[edge_target as usize].is_set(cache.cache_epoch) {
            return false;
        }

        cache.parent[edge_target as usize].set_parent(element, cache.cache_epoch);
        cache.parent[edge_target as usize].max_flow = max_flow;
        cache.parent[edge_target as usize].edge_idx = edge_index;

        cache.queue.push((
            self.nodes_adjmat[edge_target as usize].active_edges as Weight,
            edge_target,
        ));
        true
    }

    fn find_path(
        &self,
        cache: &mut MaxFlowCache,
        source: Node,
        target: Node,
        allowed_nodes: Option<&[usize]>,
    ) {
        cache.cache_epoch += 1;
        cache.queue.clear();
        cache.queue.push((0, source));
        cache.parent[source as usize].max_flow = u32::MAX;

        // let mut visited_nodes = 0;
        // let mut visited_edges = 0u32;

        let mut non_visited_nodes = allowed_nodes
            .map(|a| a.to_vec())
            .unwrap_or_else(|| (0..self.nodes_adjmat.len()).collect::<Vec<_>>());

        'outer: while let Some((_, element)) = cache.queue.pop() {
            let current_flow = cache.parent[element as usize].max_flow;

            let mut index = 0;
            let mut rewritten = 0;

            while index < non_visited_nodes.len() {
                let edge_target = non_visited_nodes[index];

                if let Some(edge_idx) = self.nodes_adjmat[element as usize].edges[edge_target] {
                    // visited_edges += 1;

                    let edge = &self.edges[edge_idx as usize];
                    let weight = edge.get_weight(element, edge_target as Node, self.flow_epoch);
                    let is_visited = self.explore_edge(
                        cache,
                        edge_idx,
                        weight.min(current_flow),
                        element,
                        edge_target as Node,
                    );

                    if is_visited && edge_target as Node == target {
                        break 'outer;
                    }

                    if is_visited {
                        // visited_nodes += 1;
                        index += 1;
                        continue;
                    }
                }

                non_visited_nodes[rewritten] = edge_target;
                rewritten += 1;
                index += 1;
            }
            non_visited_nodes.truncate(rewritten);
        }
    }
}
