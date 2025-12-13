FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ============================================
# DEPENDENCIAS DEL SISTEMA (apt)
# ============================================
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-opencv \
    python3-skimage \
    python3-networkx \
    sudo \
    tree \
    vim \
    nano \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# DEPENDENCIAS PYTHON (pip)
# ============================================
RUN python3 -m pip install --no-cache-dir \
    coppeliasim-zmqremoteapi-client

# ============================================
# CREAR USUARIO NO-ROOT ALINEADO CON EL HOST
# ============================================
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=rosuser

RUN set -e; \
    if getent group $USER_GID > /dev/null 2>&1; then \
        EXISTING_GROUP=$(getent group $USER_GID | cut -d: -f1); \
        echo "Grupo $EXISTING_GROUP con GID $USER_GID ya existe, lo usamos"; \
    else \
        groupadd --gid $USER_GID $USERNAME; \
        EXISTING_GROUP=$USERNAME; \
    fi; \
    if getent passwd $USER_UID > /dev/null 2>&1; then \
        EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        echo "Usuario $EXISTING_USER con UID $USER_UID ya existe, lo ajustamos"; \
        usermod -l $USERNAME -d /home/$USERNAME -m $EXISTING_USER || true; \
        usermod -g $USER_GID $USERNAME || true; \
    else \
        useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME; \
    fi; \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME; \
    chmod 0440 /etc/sudoers.d/$USERNAME

# ============================================
# WORKSPACE ROS2
# ============================================
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws && chown -R $USER_UID:$USER_GID /ros2_ws

# ============================================
# CAMBIAR A USUARIO NORMAL
# ============================================
USER $USERNAME

# ============================================
# CONFIGURAR ENTORNO DE SHELL
# ============================================
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> /home/$USERNAME/.bashrc && \
    echo 'cd /ros2_ws' >> /home/$USERNAME/.bashrc && \
    echo 'alias fix-perms="sudo chown -R $(id -un):$(id -gn) /ros2_ws"' >> /home/$USERNAME/.bashrc && \
    echo 'alias build="colcon build --symlink-install"' >> /home/$USERNAME/.bashrc && \
    echo 'alias source-ws="source install/setup.bash"' >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
