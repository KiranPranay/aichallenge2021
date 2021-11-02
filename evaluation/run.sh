#!/bin/sh
set -u
docker run \
  --rm -it \
  --privileged --gpus all \
  -e LG_VEHICLE_ID=$LG_VEHICLE_ID \
  -v $PWD/output:/output \
  --net host \
  ade_foxy_eval \
  bash /main.bash
