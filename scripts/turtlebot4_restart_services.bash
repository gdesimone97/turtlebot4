sshpass -p turtlebot4 ssh -o StrictHostKeyChecking=no ubuntu@$1 << EOF
sudo systemctl restart turtlebot4.service
EOF
