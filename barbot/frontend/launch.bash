#!/bin/bash
roslaunch rosbridge_server rosbridge_websocket.launch &
python -m SimpleHTTPServer 8080 . &
rostopic pub /drink_order barbot/UserAction "jobs: