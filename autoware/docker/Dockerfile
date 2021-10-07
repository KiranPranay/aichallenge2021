FROM registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/ade-foxy:1.0.0

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update; apt install ca-certificates && apt-get install -y ros-foxy-lgsvl-bridge
