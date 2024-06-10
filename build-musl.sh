docker run --rm -it -v "$(pwd)":/home/rust/src messense/rust-musl-cross:x86_64-musl cargo build --offline --profile bundle --bin solver
cp target/x86_64-unknown-linux-musl/bundle/solver pace2024solver
rm -rf target/x86_64-unknown-linux-musl/bundle/solver
docker run --rm -it -v "$(pwd)":/home/rust/src messense/rust-musl-cross:x86_64-musl cargo build --offline --profile bundle --features "heuristic" --bin solver
cp target/x86_64-unknown-linux-musl/bundle/solver pace2024solver_heuristic
