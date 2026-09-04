# ROS 2 Jazzy (Ubuntu 24.04) development container for this class.
#
# Only needed on machines that are not Ubuntu 24.04 - the class environment
# itself is a native Ubuntu 24.04 + ROS 2 Jazzy install.
FROM osrf/ros:jazzy-desktop

ARG USERNAME=irm
ARG UID=1000
ARG GID=1000

# joint_state_publisher_gui is not part of the ros-jazzy-desktop variant.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-rviz2 \
        python3-numpy \
        mesa-utils \
        libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 images ship a default "ubuntu" user on uid 1000; drop it so the
# container user can take the host uid and keep the mounted repo writable.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu 2>/dev/null || true; fi \
    && groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME} \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "cd /workspace/0904" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace/0904

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
