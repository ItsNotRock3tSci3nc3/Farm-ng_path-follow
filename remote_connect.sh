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

    ssh -o BatchMode=yes -o ConnectTimeout=${TIMEOUT_SECONDS} $HOST2 exit &>/dev/null
    if [ $? -eq 0 ]; then
        echo "SSH connection to ${HOST2} successful."
        SELECT_HOST=2
    else
        echo "SSH connection to both hosts failed."
    fi

fi

if [[ "SELECT_HOST" -eq 1 ]]; then
    echo "connecting to ${HOST}"
    ssh -X -t "$HOST" "cd robot_side && exec bash"
elif [[ "SELECT_HOST" -eq 2 ]]; then
    echo "connecting to ${HOST2}"
    ssh -X -t "$HOST2" "cd robot_side && exec bash"
fi

