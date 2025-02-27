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

RUN apt install -y xserver-xorg
RUN dpkg-reconfigure xserver-xorg

USER mivia
WORKDIR /home/mivia
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> .bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> .bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> .bashrc

EXPOSE 7400 7401
EXPOSE 7410-7415

CMD ["/bin/bash"]