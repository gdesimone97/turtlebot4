#/bin/bash

sudo rm -f /var/run/xrdp/xrdp.pid /var/run/xrdp/xrdp-sesman.pid
sudo xrdp-sesman
sudo xrdp
sleep infinity