#!/usr/bin/env bash

docker run -it \
  --name ep_car \
  --privileged \
  --net=host \
  -v /home/hkit4/config.ini:/root/config.ini:ro \
  -v /dev:/dev \
  -w /root \
  ros:humble \
  /bin/bash
  