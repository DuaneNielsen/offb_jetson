FROM dustynv/ros:noetic-pytorch-l4t-r35.1.0

RUN rm -rf /usr/lib/python3.8/dist-packages/cv2 && rm /usr/local/lib/python3.8/dist-packages/cv2 && pip3 install opencv-python==4.5.5.64

# install mavros
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 2>/dev/null && \
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
sudo apt-get update && \
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras -y --no-install-recommends 
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
RUN chmod +x install_geographiclib_datasets.sh
RUN ./install_geographiclib_datasets.sh
COPY apm_config.yaml /opt/ros/noetic/share/mavros/launch/apm_config.yaml

# install ZED
ARG L4T_MAJOR_VERSION
ARG L4T_MINOR_VERSION
ARG L4T_PATCH_VERSION
ARG ZED_SDK_MAJOR
ARG ZED_SDK_MINOR

#This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update -y || true
RUN sudo apt install zstd -y
RUN apt-get install --no-install-recommends lsb-release wget less udev sudo apt-transport-https -y && \
    echo "# R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}" > /etc/nv_tegra_release ; \
    wget -q --no-check-certificate -O ZED_SDK_Linux.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR_VERSION}.${L4T_MINOR_VERSION}/jetsons && \
    chmod +x ZED_SDK_Linux.run ; ./ZED_SDK_Linux.run silent runtime_only && \
    rm -rf /usr/local/zed/resources/* \
    rm -rf ZED_SDK_Linux.run && \
    rm -rf /var/lib/apt/lists/*

# ZED Python API
RUN apt-get update -y || true
RUN apt-get install --no-install-recommends python3 python3-pip python3-dev python3-setuptools build-essential -y && \ 
    wget download.stereolabs.com/zedsdk/pyzed -O /usr/local/zed/get_python_api.py && \
    python3 /usr/local/zed/get_python_api.py && \
    python3 -m pip install cython wheel && \
    python3 -m pip install numpy *.whl && \
    apt-get remove --purge build-essential python3-dev -y && apt-get autoremove -y && \
    rm *.whl ; rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# clone repos
WORKDIR /root
ADD https://api.github.com/repos/duanenielsen/YOLOv7_Tensorrt/git/refs/heads/master YOLOv7_Tensorrt_git_version.json
RUN git clone https://github.com/DuaneNielsen/YOLOv7_Tensorrt.git --branch master
ADD https://api.github.com/repos/duanenielsen/yolov7/git/refs/heads/main yolov7_git_version.json
RUN git clone https://github.com/DuaneNielsen/yolov7.git
RUN cd YOLOv7_Tensorrt && \
pip3 install -e .

# copy the engine file for xavier
COPY yolov7.engine /root/yolov7.engine

ADD https://api.github.com/repos/duanenielsen/conefinder/git/refs/heads/main conefinder_git_version.json
RUN git clone https://github.com/DuaneNielsen/conefinder.git && cd conefinder && pip3 install -e .

# offboard library
ENV FCUURL=/dev/ttyACM0
ADD https://api.github.com/repos/duanenielsen/offboard/git/refs/heads/magellan offboard_git_version.json
RUN mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src && git clone --branch magellan http://github.com/duanenielsen/offboard
RUN . /opt/ros/noetic/setup.sh && cd ~/catkin_ws && catkin_make install 

# alias to make life easier
COPY .bash_aliases /root/.bash_aliases
COPY cone.png /root/cone.png

# Finally the command
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT /entrypoint.sh ${FCUURL}
