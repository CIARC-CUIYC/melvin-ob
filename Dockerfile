# === Stage 1: Build the melvin-ob Binary ===
FROM rust:latest AS builder
WORKDIR /usr/src/app

# leverage Docker layer caching for dependencies
COPY Cargo.* ./
# Copy the source code
COPY src/ ./src/

RUN apt-get update && \
    apt-get install -y gcc-x86-64-linux-gnu libc6-dev-amd64-cross && \
    rustup target add x86_64-unknown-linux-gnu

ENV RUSTFLAGS="-C target-feature=+crt-static"

# Build the release version of melvin-ob
RUN cargo build --release \
        --target=x86_64-unknown-linux-gnu \
        --config "target.x86_64-unknown-linux-gnu.linker=\"x86_64-linux-gnu-gcc\""

# === Stage 2: Create the Runtime Image ===
FROM ubuntu:latest AS runner
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages: tmux for session management
RUN apt-get update && apt-get install -y \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Custom tmux.conf
COPY tmux.conf /root/.tmux.conf

WORKDIR /app

# Copy the compiled binary from the builder stage.
COPY --from=builder /usr/src/app/target/x86_64-unknown-linux-gnu/release/melvin-ob /app/melvin-bin

CMD tmux new-session -d -s melvin_debug bash \
    -c 'export TRACK_MELVIN_POS=1; export DRS_BASE_URL="http://palantiri_container:5000"; export EXPORT_ORBIT=1; exec bash' && \
    tail -f /dev/null