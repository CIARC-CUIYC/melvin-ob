#!/bin/bash

# Kill existing tmux session if it exists
sshpass -p password ssh -p 50000 root@localhost 'tmux kill-session -t melvin_evaluation || true'

# Copy the Rust binary and tmux conf to the remote machine
sshpass -p password scp -P 50000 ./target/x86_64-unknown-linux-gnu/release/melvin-ob root@localhost:/home/melvin-ob
sshpass -p password scp -P 50000 ./tmux.conf root@localhost:/home/tmux.conf

# Start a new tmux session with environment variables set
sshpass -p password ssh -p 50000 root@localhost \
"cd /home && tmux -f ./tmux.conf new-session -d -s melvin_evaluation bash -c 'export SKIP_RESET=1; export DRS_BASE_URL=\"http://10.100.10.3:33000\"; export EXPORT_ORBIT=1; export TRY_IMPORT_ORBIT=1; exec bash -l'"