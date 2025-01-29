sshpass -p turtlebot4 ssh -o StrictHostKeyChecking=no ubuntu@$1 << EOF
turtlebot4-service-restart
EOF
