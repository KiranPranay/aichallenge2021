#!/bin/bash

cd "$(dirname "$0")/autoware/docker"
docker build -t ade_foxy_lgsvl .
mkdir -p ../adehome
cd ../adehome
touch .adehome
cp ../aderc .aderc

ade start --update