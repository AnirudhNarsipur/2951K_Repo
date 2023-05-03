FROM ros:kinetic

RUN apt-get update

RUN chmod +x /bin/sh

RUN apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y emacs
RUN apt-get install -y sudo
RUN apt-get install -y python-pip
RUN apt-get install -y net-tools
RUN apt-get install -y iproute2
RUN apt-get install -y iputils-ping
RUN apt-get install -y openssh-client openssh-server
RUN apt-get install -y ros-kinetic-desktop-full

#baxter sdk dependencies
RUN apt-get install -y git-core
RUN apt-get install -y python-argparse
RUN apt-get install -y python-wstool
RUN apt-get install -y python-vcstools
RUN apt-get install -y python-rosdep
RUN apt-get install -y ros-kinetic-control-msgs
RUN apt-get install -y ros-kinetic-joystick-drivers

RUN apt-get install -y gdb
RUN apt-get install -y mlocate
RUN apt-get install -y screen
RUN apt-get install -y emacs
RUN apt-get install -y git
RUN apt-get install -y netcat nmap wget iputils-ping openssh-client vim less
RUN apt-get install -y python-numpy
RUN apt-get install -y python-smbus
RUN apt-get install -y python-scipy
RUN apt-get install -y locate


# ein stuff
#RUN apt-get install qt5-default python-wstool ros-kinetic-object-recognition-msgs libgsl0-dev ros-kinetic-serial ros-kinetic-object-recognition-msgs ros-kinetic-pcl-ros libgsl0-dev qt5-default screen


#copied and paste from pidrone dockerfile
ARG hostuser
ARG hostgroup
ARG hostuid
ARG hostgid
ARG hostname

RUN echo Host user is $hostuser:$hostuser
RUN groupadd --gid $hostgid $hostgroup || true
RUN adduser --disabled-password --gecos '' --gid $hostgid --uid $hostuid $hostuser 
RUN adduser $hostuser sudo
RUN adduser $hostuser video

# Ensure sudo group users are not asked for a p3assword when using sudo command
# by ammending sudoers file
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
/etc/sudoers

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

USER $hostuser
WORKDIR /home/$hostuser
ENV HOME=/home/$hostuser
RUN mkdir $HOME/repo
RUN mkdir -p $HOME/catkin_ws/src
RUN rosdep update
#baxter sdk install

RUN cd ~/catkin_ws/src && wstool init .
RUN cd ~/catkin_ws/src && wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
RUN cd ~/catkin_ws/src && wstool update
#RUN cd ~/catkin_ws/src && git clone https://github.com/h2r/ein
#RUN cd ~/catkin_ws/src/ein && git checkout 

RUN cd ~/catkin_ws && source /opt/ros/kinetic/setup.bash && catkin_make
RUN cp ~/catkin_ws/src/baxter/baxter.sh ~/catkin_ws

RUN sudo apt-get install -y libgl1-mesa-glx mesa-utils

# nvidia-container-runtime
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

CMD ["bash"]


