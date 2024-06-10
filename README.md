# Solver for the pace2024 challenge
This solver uses flow heuristics to find a good solution,
then for the exact and parametric tracks finds the optimum using an ILP model (with SCIP as backend).

# Build
The build process is self contained, no internet connection is required
(except for the docker musl image).
### Requirements
- Stable Rust >= 1.78

To build the solver, three options are available:
- Generate an executable for the exact and parameterized tracks using the local environment
```
cargo build --profile bundle
```
the binary will be in ./target/bundle/solver
- Generate an executable for the heuristic track using the local environment
```
cargo build --profile bundle --features "heuristic"
```
the binary will be in ./target/bundle/solver
- Generate a binary targeting x86_64-musl that should run in any linux x64 machine (requires docker)
```
bash build-musl.sh
```
it generates 2 executables in the root of the repository:
- pace2024solver for the exact and parameterized tracks
- pace2024solver_heuristic for the heuristic track

# Usage
The default behavior of the program is to read an instance from stdin and write
an instance to stdout, as required for the optil.io platform.

To override this behavior, the feature "file_input" can be specified, that enables a command line to specify input and output files.
The combination of features "verbose,file_input" can output detailed information about the solver processing
