sudo apt update
sudo apt install -y net-tools
if [ ! -d "/tmp/turtlebot4_setup_orig/" ]; then
	git clone -b humble https://github.com/turtlebot/turtlebot4_setup.git /tmp/turtlebot4_setup_orig/
	echo "Return: $?" 
fi
