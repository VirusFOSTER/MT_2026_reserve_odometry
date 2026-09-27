FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Moscow

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 libqt5svg5 ros-humble-rmw-cyclonedds-cpp -y libyaml-cpp-dev libpcap-dev smbclient python3-pip nlohmann-json3-dev libgeographic-dev

# Создаём пользователя с UID/GID хоста, чтобы избежать проблем с правами
RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Рабочая директория внутри контейнера — отдельно от монтируемого src
# Сюда будут складываться build/, install/, log/
RUN mkdir -p /ros2_ws/src /ros2_ws_build \
    && chown -R $USER_UID:$USER_GID /ros2_ws /ros2_ws_build

USER $USERNAME
WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

CMD ["bash"]
