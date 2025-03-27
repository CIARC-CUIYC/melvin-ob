#!/bin/bash

# Copy the dump folder and the zo_img folder from the remote machine
sshpass -p password scp -P 50000 -r root@localhost:/home/dumps ./evaluation/dumps
sshpass -p password scp -P 50000 -r root@localhost:/home/zo_img ./evaluation/zo_img

if [[ -n "$PULL_FULL" ]]; then
    echo "Pulling full snapshot"
    sshpass -p password scp -P 50000 root@localhost:/home/snapshot_full.png ./evaluation/snapshot_full.png
else
    echo "Not pulling full snapshot"
fi

