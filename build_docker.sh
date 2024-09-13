#!/usr/bin/env bash

# script to fetch the correct ip of the host system and to build the docker ENV
ROS_IP=$(hostname -I | awk '{print $1}')
docker build -t edu-elfin --build-arg="IP=$ROS_IP" .