# syntax=docker/dockerfile:1
FROM registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/ade-foxy:1.0.0

ARG NVIDIA_VISIBLE_DEVICES=all
ARG NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display
ARG DEBIAN_FRONTEND=noninteractive

# copy autoware.auto files
COPY --from=registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/binary-foxy:1.0.0 /opt/AutowareAuto /opt/AutowareAuto

# add bridge and pythonapi for lg
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
 && apt-get update; apt install ca-certificates \
 && apt-get update \
 && apt-get install -y ros-foxy-lgsvl-bridge

WORKDIR /
RUN git clone https://github.com/lgsvl/PythonAPI \
 && cd PythonAPI \
 && git checkout 2021.2 \
 && pip3 install -e .

# copy bringup files
COPY bringup/autoware/adehome/aichallenge_ws/src /opt/aichallenge_ws/src
COPY bringup/scenario /scenario

# Copy user files by docker `COPY` command
COPY aichallenge_submit.tar.gz /aichallenge_submit.tar.gz
RUN cd / \
 && tar zxf aichallenge_submit.tar.gz -C /opt/aichallenge_ws/src/aichallenge_submit \
 && rm aichallenge_submit.tar.gz

# build
RUN apt-get update
RUN bash -c 'source /opt/AutowareAuto/setup.bash; \
 cd /opt/aichallenge_ws; \
 rosdep update; \
 rosdep install -y -r -i --from-paths src --ignore-src --rosdistro foxy; \
 colcon build'

COPY main.bash /main.bash
ENTRYPOINT []
CMD ["bash", "/main.bash"]
