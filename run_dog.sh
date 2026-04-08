#!/bin/bash

set -e

docker stop robot_dog 2>/dev/null
docker rm robot_dog 2>/dev/null

sleep 5

docker logs robot_dog --tail 20

