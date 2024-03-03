FROM ubuntu:22.04

RUN apt update
RUN apt install xfce4 xfce4-goodies -y
RUN apt install xrdp -y
RUN apt install -y nano gedit net-tools
RUN apt install -y sudo iptables git

RUN useradd -m -p $(openssl passwd -1 mivia) -s /bin/bash mivia
RUN usermod -aG sudo mivia
RUN adduser xrdp ssl-cert 

USER mivia
WORKDIR /home/mivia/
ADD run.sh .
RUN echo "xfce4-session" | tee .xsession

RUN git clone https://github.com/gdesimone97/turtlebot4.git
WORKDIR turtlebot4/scripts

USER root
RUN bash ros2_humble_install.bash
RUN bash ros2-tools.bash
RUN bash turtlebot4_tools.bash

RUN update-alternatives --set x-terminal-emulator /usr/bin/xfce4-terminal.wrapper

USER mivia
CMD ["/bin/bash"]