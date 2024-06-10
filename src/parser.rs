use std::{
    fs::File,
    io::{BufRead, BufReader, Write},
    path::Path,
};

#[derive(Clone, Debug)]
pub struct ParsedInstance {
    pub ocr: String,
    pub cutwidth: Option<usize>,
    pub n0: usize,
    pub n1: usize,
    pub m: usize,
    pub adj_list: Vec<Vec<u32>>,
}

impl ParsedInstance {
    pub fn new(file: File) -> Self {
        let mut input = BufReader::new(file);
        let mut instance;
        loop {
            let mut line = String::new();
            input.read_line(&mut line).unwrap();

            let mut parts = line.split_whitespace();

            let first = parts.next().unwrap();

            match first {
                "c" => continue,
                "p" => {
                    let ocr = parts.next().unwrap();
                    let n0 = parts.next().unwrap().parse().unwrap();
                    let n1 = parts.next().unwrap().parse().unwrap();
                    let m: usize = parts.next().unwrap().parse().unwrap();
                    let cutwidth = parts.next().map(|x| x.parse().unwrap());
                    instance = ParsedInstance {
                        ocr: ocr.to_string(),
                        n0,
                        n1,
                        m,
                        cutwidth,
                        adj_list: vec![vec![]; n1],
                    };
                    break;
                }
                _ => {}
            }
        }

        if instance.cutwidth.is_some() {
            for _ in 0..(instance.n0 + instance.n1) {
                input.read_line(&mut String::new()).unwrap();
            }
        }

        for line in input.lines().take(instance.m) {
            let line = line.unwrap();
            let mut parts = line.split_whitespace();
            let u: u32 = parts.next().unwrap().parse().unwrap();
            let v: usize = parts.next().unwrap().parse().unwrap();
            // Make everything 0 based
            instance.adj_list[v - 1 - instance.n0].push(u - 1);
        }

        instance
    }

    pub fn uniform_indices(&self, indices: &[usize]) -> Vec<usize> {
        indices.iter().map(|&x| x - 1 - self.n0).collect::<Vec<_>>()
    }

    pub fn read_order(file: impl AsRef<Path>) -> Vec<usize> {
        let input = BufReader::new(File::open(file).unwrap());
        let mut order = vec![];
        for line in input.lines() {
            let line = line.unwrap();
            let idx: usize = line.trim().parse().unwrap();
            order.push(idx);
        }
        order
    }

    pub fn write_order(file: impl AsRef<Path>, order: &[usize]) {
        let mut output = File::create(file).unwrap();
        for idx in order {
            writeln!(output, "{}", idx).unwrap();
        }
    }
}
