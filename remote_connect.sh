#!/bin/bash

host_dir=.
HOST=$(<"$host_dir/farm-ng.txt")
echo "Connecting to remote farm-ng at $HOST"
ssh -X -t "$HOST"  "cd robot_side && exec bash"

