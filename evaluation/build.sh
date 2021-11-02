#!/bin/sh
mkdir -p bringup/autoware/adehome/aichallenge_ws/src
mkdir -p bringup/scenario
cp -r ../autoware/adehome/aichallenge_ws/src/* bringup/autoware/adehome/aichallenge_ws/src/
cp -r ../scenario/* bringup/scenario/

docker build -t ade_foxy_eval .
