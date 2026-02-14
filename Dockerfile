FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update --fix-missing && apt-get install -y git \
    vim \
    python3-pip \
    libeigen3-dev \
    ros-humble-rviz2 \
    ros-humble-rqt-graph
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && pip3 install -e .

RUN mkdir -p workspace/src/f1tenth_simulator
COPY . /workspace/src/f1tenth_simulator
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

WORKDIR '/workspace'
ENTRYPOINT ["/bin/bash"]