FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/Rome"

RUN apt update
RUN apt install -y nano gedit net-tools sudo iptables git python3-pip x11-apps
RUN python3 -m pip install -U pip

RUN useradd -m -p $(openssl passwd -1 mivia) -s /bin/bash mivia
RUN usermod -aG sudo mivia

#Configure Timezone
RUN apt-get update && \
    apt-get install -yq tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

USER mivia
WORKDIR /home/mivia/
RUN git clone https://github.com/gdesimone97/turtlebot4.git
WORKDIR turtlebot4/scripts
USER root
RUN bash install_locales.bash
RUN bash ros2_humble_install.bash
RUN bash ros2-tools.bash
RUN bash turtlebot4_tools.bash
RUN apt install -y ntp

RUN apt update
SHELL [ "/bin/bash" , "-c"]
WORKDIR /home/mivia/turtlebot4/diem_turtlebot_ws/src
RUN git clone -b humble https://github.com/iRobotEducation/create3_sim.git
WORKDIR ..
ENV IGNITION_VERSION=fortress
RUN source /opt/ros/humble/setup.bash && rosdep init && rosdep update
RUN source /opt/ros/humble/setup.bash && rosdep install --ignore-src --rosdistro humble --from-path src -yi
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

ADD ntp.conf /etc/ntp.conf

USER mivia

WORKDIR /home/mivia
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> .bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> .bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> .bashrc

ENV ROS_LOCALHOST_ONLY=1

CMD ["/bin/bash"]