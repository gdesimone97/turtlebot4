ntpdate -u ntp.unisa.it
ntpdate -u ntp.ubuntu.com
sleep 5
sudo systemctl start ntp
