# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:humble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Layer 1: System packages — only rebuilds when this list changes
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        nano \
        vim \
        python3-pip \
        python3-dev \
        libeigen3-dev \
        tmux \
        ros-humble-rviz2 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /sim_ws

# Layer 2: f1tenth_gym Python package and its deps
# Only rebuilds when f1tenth_gym source or pyproject.toml changes, not when ROS code changes
RUN mkdir -p /sim_ws/src/f1tenth_gym_ros
COPY f1tenth_gym/ /sim_ws/src/f1tenth_gym_ros/f1tenth_gym/
RUN pip install -U pip && \
    pip install -e /sim_ws/src/f1tenth_gym_ros/f1tenth_gym

# Layer 3: ROS dependencies via rosdep
# Only rebuilds when package.xml changes, not when source code changes
COPY package.xml /sim_ws/src/f1tenth_gym_ros/package.xml
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update && \
    rosdep install -i --from-paths /sim_ws/src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# Layer 4: colcon build — only the files colcon actually needs, no README/Dockerfile/tests/etc.
# Rebuilds only when ROS package source changes
COPY setup.py setup.cfg /sim_ws/src/f1tenth_gym_ros/
COPY resource/ /sim_ws/src/f1tenth_gym_ros/resource/
COPY f1tenth_gym_ros/ /sim_ws/src/f1tenth_gym_ros/f1tenth_gym_ros/
COPY launch/ /sim_ws/src/f1tenth_gym_ros/launch/
COPY config/ /sim_ws/src/f1tenth_gym_ros/config/
COPY maps/ /sim_ws/src/f1tenth_gym_ros/maps/
COPY urdf/ /sim_ws/src/f1tenth_gym_ros/urdf/
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /sim_ws/install/local_setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
