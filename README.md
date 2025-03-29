![CI](https://github.com/CIARC-CUIYC/melvin-ob/workflows/build-deploy.yaml/badge.svg)
# 🛰 MELVIN-OB
Welcome to the onboard software for **Team 03 — "Cache us if you can"** competing in the **2024/2025 ESA Computer in a Room Challenge**. 
This repository contains the embedded code running on the simulated MELVIN onboard computer, responsible 
for command execution, event detection, task scheduling and DRS communication during the mission.

## 📦 Project Overview
MELVIN-OB is designed to:
* Communicate with a RESTful API server (DRS).
* Manage the challenges objectives and system state.
* Support runtime configuration and semi-live patching.

In this README you can find information regarding the building and deployment of this repository 
as well as some insights into the runtime configuration for our implementation. 
An alternative to building and running this codebase locally is our containerized integration 
solution bundling this binary with our self-written Software-in-a-loop Testing framework 'Palantiri'. 
You can find further instructions on the use of this compound Docker container
[here](https://github.com/CIARC-CUIYC/cirdan).

## 🔨 Building the Project
### ✅ Dependencies
Ensure the following are installed on your deployment system:
* Rust toolchain
    + Recommended: rustup (for managing versions)
    + Minimum Rust version: 1.85.0 (or your actual version)
    + cargo (comes with rustup)
* *(Optional)* Docker for containerized builds using our dockerized testing environment [Cirdan](https://github.com/CIARC-CUIYC/cirdan).

### 📥 Installation
Install Rust via:
#### Linux/macOS
```bash
  curl https://sh.rustup.rs -sSf | sh
```
#### Windows
Download and execute the `.exe`-installer from 
[the official Rust webpage](https://forge.rust-lang.org/infra/other-installation-methods.html) and follow the GUI instructions.
### 🖥️ Compilation/Build
```bash
# Clone the repository
git clone https://github.com/CIARC-CUIYC/melvin-ob.git
cd melvin-ob

# Build in release mode
cargo build --release

# Optional: run directly
cargo run
# Optional: execute testcases (we implented some, but they weren't the focus here)
cargo test -- --nocapture
```
The compiled binary will be located at `target/release/melvin-ob`.

## ⚙️ Runtime Configuration
There are multiple configurable options that are settable via environment variables.
### ❗ Mandatory
| Variable                          | Description                                           |
|-----------------------------------|-------------------------------------------------------|
| `DRS_BASE_URL=http://server:port` | Base URL of the DRS backend for sending HTTP requests |

### ❓ Optional
| Variable              | Description                                                           |
|-----------------------|-----------------------------------------------------------------------|
| `RUST_BACKTRACE=1`    | Enables full Rust backtraces on panic for debugging.                  |
| `SKIP_RESET=1`        | Skips the initial reset command to the DRS backend.                   |
| `EXPORT_ORBIT=1`      | Periodically export the orbit configuration to `orbit.bin`.           |
| `TRY_IMPORT_ORBIT=1`  | Initially attempts to load a previous orbit state from `./orbit.bin`. |
| `LOG_MELVIN_EVENTS=1` | Enables logging of all `/announcements` messages.                     |
| `SKIP_OBJ=1,3,15`     | Comma-separated list of objective IDs to skip during execution.       |

## 🐳 Containerized SIL Deployment
To run MELVIN-OB inside the Cirdan container environment (with the SIL framework):
```bash
git clone https://github.com/CIARC-CUIYC/cirdan.git
cd cirdan
docker compose up --build
```
This setup includes:
* melvin-ob in a terminal-accessible tmux session
* palantiri SIL backend (exposes a REST API)
See detailed instructions [here](https://github.com/CIARC-CUIYC/cirdan).