sudo systemctl stop powerUp
sudo systemctl disable powerUp
sudo rm /etc/systemd/system/powerUp.service
sudo systemctl daemon-reload
systemctl status powerUp