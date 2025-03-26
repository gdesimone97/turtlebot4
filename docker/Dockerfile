FROM ubuntu:22.04 AS ros_base
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
SHELL [ "/bin/bash" , "-c"]
RUN source /opt/ros/humble/setup.bash && rosdep init && rosdep update

RUN  apt install -y xserver-xorg
RUN dpkg-reconfigure xserver-xorg

RUN chown -R mivia /home/mivia/turtlebot4/

FROM scratch as turtlebot
COPY --from=ros_base / /
SHELL [ "/bin/bash" , "-c"]

# WORKDIR /opt/ros/humble
# RUN find . -name irobot_create_ignition_toolbox -type d -exec rm -rv {} +
# RUN find . -name irobot_create_nodes -type d -exec rm -rv {} +
# RUN find . -name irobot_create_ignition_bringup -type d -exec rm -rv {} +
# RUN find . -name irobot_create_common_bringup -type d -exec rm -rv {} +
# RUN find . -name irobot_create_toolbox -type d -exec rm -rv {} +
# RUN find . -name irobot_create_description -type d -exec rm -rv {} +

RUN apt update
WORKDIR /home/mivia/turtlebot4/diem_turtlebot_ws/src
RUN git config --global --add safe.directory /home/mivia/turtlebot4
RUN git clone -b humble https://github.com/gdesimone97/create3_sim.git
WORKDIR ..
ENV IGNITION_VERSION=fortress
RUN source /opt/ros/humble/setup.bash && rosdep install --ignore-src --rosdistro humble --from-path src -yi
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --allow-overriding irobot_create_common_bringup irobot_create_description irobot_create_ignition_bringup irobot_create_ignition_toolbox irobot_create_nodes irobot_create_toolbox
ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" /tmp/skipcache
WORKDIR /home/mivia/turtlebot4/diem_turtlebot_ws/src/create3_sim
RUN git pull
WORKDIR /home/mivia/turtlebot4/
RUN git pull
WORKDIR /home/mivia/turtlebot4/diem_turtlebot_ws/
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --allow-overriding irobot_create_common_bringup irobot_create_description irobot_create_ignition_bringup irobot_create_ignition_toolbox irobot_create_nodes irobot_create_toolbox

USER mivia
WORKDIR /home/mivia
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> .bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> .bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> .bashrc

ENV ROS_LOCALHOST_ONLY=1
ENV ROS_DOMAIN_ID=0
#ENV LIBGL_ALWAYS_SOFTWARE=true #Uncomment to disable GPU

CMD ["/bin/bash"]