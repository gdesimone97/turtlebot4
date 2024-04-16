sudo ntpdate -u ntp.unisa.it
sudo ntpdate -u ntp.ubuntu.com
sleep 5
sudo systemctl start ntp
