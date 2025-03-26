mkdir -p ~/shared_folder
xhost +
docker run --name mivia_turtlebot4_cnt \
    -v ~/shared_folder:/shared_folder \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v /dev:/dev \
    --privileged \
    -d \
    -e DISPLAY=$DISPLAY \
    -it mivia_turtlebot4 /bin/bash