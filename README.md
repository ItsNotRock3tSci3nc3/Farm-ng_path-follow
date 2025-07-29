# Farm-ng Amiga Path Following

This project is a path following project for the Farm-ng Amiga. Using an external computer, we are able to send data to the Farm-ng to navigate terrain accordingly.

## How to Start (Without Lookahead)

### Robot Side:
1. SSH into robot  
2. `cd robot_side`  
3. `bash start.sh` → composes `robot_side` container  
4. `. run.sh` → WebSocket starts and is running, starts robot controller program  

### Local Side:
1. `docker compose up local_side`  
2. `bash start.sh`  
3. `python main_controller.py`  

## How to Start (With Lookahead)

### Robot Side:
1. SSH into robot  
2. `cd robot_side`  
3. `bash start.sh` → composes `robot_side` container  
4. `. run_lookahead.sh` → WebSocket starts and is running, starts robot controller program  

### Local Side:
1. `docker compose up local_side`  
2. `bash start.sh`  
3. `python main_controller_lookahead.py`  
