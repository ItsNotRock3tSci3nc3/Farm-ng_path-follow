#!/bin/bash

host_dir="."
HOST=$(<"$host_dir/farm-ng.txt")
HOST2=$(<"$host_dir/farm-ng_2.txt")

SELECT_HOST=0

TIMEOUT_SECONDS=5 # Optional: Set a timeout for the connection attempt

# Attempt to connect and immediately exit, redirecting output to /dev/null
ssh -o BatchMode=yes -o ConnectTimeout=${TIMEOUT_SECONDS} $HOST exit &>/dev/null

# Check the exit status of the last command
if [ $? -eq 0 ]; then
    echo "SSH connection to ${HOST} successful."
    SELECT_HOST=1
else
    echo "SSH connection to ${HOST} failed. Attempting to connect to ${HOST2}"
    ssh -o BatchMode=yes -o ConnectTimeout=${TIMEOUT_SECONDS} $HOST exit &>/dev/null
    if [ $? -eq 0 ]; then
        echo "SSH connection to ${HOST2} successful."
        SELECT_HOST=2
    else
        echo "SSH connection to both hosts failed. Exiting."
        exit 1
    fi

fi

if SELECT_HOST=1; then
    ssh -X -t "$HOST" "cd robot_side && exec bash && echo 'Debugging connection to $HOST complete.'"
elif SELECT_HOST=2; then
    ssh -X -t "$HOST2" "cd robot_side && exec bash && echo 'Debugging connection to $HOST2 complete.'"
fi

