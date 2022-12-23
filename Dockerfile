FROM dustynv/ros:noetic-ros-base-l4t-r35.1.0
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 2>/dev/null && \
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
sudo apt-get update && \
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras -y --no-install-recommends 
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh
COPY apm_config.yaml /opt/ros/noetic/share/mavros/launch/apm_config.yaml
# Envs
ENV FCUURL=/dev/ttyACM0

# Finally the command
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT /entrypoint.sh ${FCUURL}
