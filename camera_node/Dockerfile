FROM ros:humble-ros-base

ENV USER=root
ENV HOME="/${USER}"

RUN apt update \
    && apt install -y software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt update \
    && apt install -y python3 python3-pip
RUN echo "source /opt/ros/humble/setup.bash" >> ${HOME}/.bashrc \
    && echo "env | grep ROS" >> ${HOME}/.bashrc

# src ディレクトリを作成
WORKDIR ${HOME}/workspace/src 
WORKDIR ${HOME}/workspace
RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro humble -y \
    && colcon build
