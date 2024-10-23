#!/bin/bash

xhost +local:root

# Start the Docker container first
gnome-terminal -- bash -c "sudo docker start -ai 4c1a19786d95; exec bash" &

# Open second terminal and attach to the running container
gnome-terminal -- bash -c "sudo docker exec -it reverent_robinson /bin/bash; exec bash" &

# Open third terminal and attach to the running container
gnome-terminal -- bash -c "sudo docker exec -it reverent_robinson /bin/bash; exec bash" &

# Open fourth terminal and attach to the running container
gnome-terminal -- bash -c "sudo docker exec -it reverent_robinson /bin/bash; exec bash" &

# Open fourth terminal and attach to the running container
gnome-terminal -- bash -c "sudo docker exec -it reverent_robinson /bin/bash; exec bash" &

gnome-terminal -- bash -c "sudo docker exec -it reverent_robinson /bin/bash; exec bash" &

gnome-terminal -- bash -c "sudo docker exec -it reverent_robinson /bin/bash; exec bash" &

