echo \
"[Unit]
Description=Power UP inverter for cec-e50 via CAN bus
[Service]
Type=simple
ExecStart=$(rospack find lidar_perception)/../../devel/lib/lidar_perception/powerUp -i
[Install]
WantedBy=default.target" > powerUp.service

sudo cp powerUp.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable powerUp.service
rm ./powerUp.service
systemctl status powerUp.service