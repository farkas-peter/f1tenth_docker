ARG HOST_UID=1000
ARG HOST_GID=1000
ARG BASE_IMG=nvidia/cuda:12.2.0-base-ubuntu22.04

FROM $BASE_IMG

ARG HOST_USER=f1tenth
ARG HOST_UID
ARG HOST_GID

SHELL ["/bin/bash", "-c"]

## Update and add dependencies
RUN apt-get update 									 && \
	DEBIAN_FRONTEND=noninteractive apt install -y curl	\ 
		dbus-x11 										\
		gnome-terminal 									\
		htop 											\
		iproute2 										\
		libcanberra-gtk-module libcanberra-gtk3-module 	\
		mc 												\
		nano 											\
		net-tools 										\
		openssh-server 									\
		python3-pip 									\
		snap 											\
		software-properties-common 						\
		sudo 											\
		telnet 											\
		terminator 										\
		vim 											\
		wget 										 && \
	apt-get clean -qq 								 && \
	rm -rf /var/lib/apt/lists/* 					 && \
	rm -rf /tmp/* 

# Set ssh config
RUN cat /etc/ssh/sshd_config | sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN cat /etc/ssh/sshd_config | sed -i 's/#Port 22/Port 24/' /etc/ssh/sshd_config

# Set user
RUN useradd -m $HOST_USER 												&& \
	echo "$HOST_USER:$HOST_USER" | chpasswd  							&& \
	usermod --shell /bin/bash $HOST_USER 								&& \
	groupadd -f -r gpio 												&& \
	usermod -aG gpio,render,sudo,video,dialout $HOST_USER 						&& \
	echo "$HOST_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$HOST_USER && \
	usermod  --uid $HOST_UID $HOST_USER 								&& \
	groupmod --gid $HOST_GID $HOST_USER

## Install ros humble
ENV ROS_DISTRO=humble
RUN apt update 															&& \ 
	add-apt-repository universe  										&& \ 
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
	apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-$ROS_DISTRO-ros-base \
		ros-dev-tools 													&& \
	rosdep init 														&& \
	rosdep update --rosdistro $ROS_DISTRO 								&& \
	apt-get clean -qq 													&& \
	rm -rf /var/lib/apt/lists/* 										&& \
	rm -rf /tmp/* 

## Adding repositories/PPAs

# Adding additional packages
RUN apt update && apt install -y 											\
		libasio-dev 														\
		pcl-tools 															\
		python3-tk 															\
		ros-$ROS_DISTRO-asio-cmake-module 										\
		ros-$ROS_DISTRO-ackermann-msgs 										\
		ros-$ROS_DISTRO-camera-calibration 									\
		ros-$ROS_DISTRO-can-msgs 											\
		ros-$ROS_DISTRO-cv-bridge 											\
		ros-$ROS_DISTRO-foxglove-bridge 									\
		ros-$ROS_DISTRO-gps-msgs 											\
		ros-$ROS_DISTRO-joy 												\
		ros-$ROS_DISTRO-message-filters 									\
		ros-$ROS_DISTRO-pcl-conversions 									\
		ros-$ROS_DISTRO-pcl-ros 									\
		ros-$ROS_DISTRO-plotjuggler-ros 									\
		ros-$ROS_DISTRO-python-qt-binding 									\
		ros-$ROS_DISTRO-robot-localization 									\
		ros-$ROS_DISTRO-rosbag2-storage-mcap 								\
		ros-$ROS_DISTRO-rqt-gui-py 											\
		ros-$ROS_DISTRO-rqt-tf-tree 										\
		ros-$ROS_DISTRO-rtcm-msgs 											\
		ros-$ROS_DISTRO-rviz2 												\
		ros-$ROS_DISTRO-tf2-eigen 												\
		ros-$ROS_DISTRO-udp-msgs 											\
		ros-$ROS_DISTRO-usb-cam 											\
		ros-$ROS_DISTRO-xacro 											 && \
	apt-get clean -qq 													 && \
	rm -rf /var/lib/apt/lists/* 										 && \
	rm -rf /tmp/* 

## Create workspace for ROS2(workspace)
RUN mkdir -p /workspace/src

## Adding egy f1-tenth stack
RUN cd /workspace/src && git clone https://github.com/f1tenth/f1tenth_system.git
RUN cd /workspace/src/f1tenth_system && git checkout humble-devel && git submodule update --init --force --remote


## ROS2 workspace build
RUN cd /workspace && apt update							&& \
	source /opt/ros/$ROS_DISTRO/setup.bash 		&& \
	DEBIAN_FRONTEND=noninteractive rosdep update && \
	DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src -i -y && \
	colcon build --symlink-install

## Settings in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc 			&& \
	echo "source /workspace/install/setup.bash" >> /etc/bash.bashrc 			&& \
	echo "export ROS_LOG_DIR=/workspace/LOG/ROS_OUTPUT" >> /etc/bash.bashrc 	&& \
	echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /etc/bash.bashrc 				&& \
	echo "set -a && source /workspace/.env && set +a" >> /etc/bash.bashrc 		&& \
	echo "cd /workspace" >> /etc/bash.bashrc

## Set environment variables for terminator
RUN echo "export NO_AT_BRIDGE=1" >> /etc/bash.bashrc

## Set ownership
RUN chown -R $HOST_UID:$HOST_GID /workspace

## Copy terminator config
COPY ./utility/terminal_config /etc/xdg/terminator/config
	
## Copy entry.sh into the docker
COPY ./entry.sh /
