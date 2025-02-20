## Give user serial port permission

[](https://askubuntu.com/questions/58119/changing-permissions-on-serial-port)

1. `sudo usermod -a -G dialout $USER`
2. Reboot computer

## Enable rover hotspot

[](https://askubuntu.com/questions/500370/setting-up-wireless-hotspot-to-be-on-at-boot)

1. Settings → Wi-Fi → Right Corner → Turn On Wi-Fi Hotspot
2. Enter SSID and PW
3. `sudo vim /etc/NetworkManager/system-connections/Hotspot.nmconnection`
4. set `autoconnect=false` to `true`

## Fix CH340 issues

[](https://askubuntu.com/questions/1403705/dev-ttyusb0-not-present-in-ubuntu-22-04)

1. `sudo apt remove brltty`
2. Unplug and replug USB device

## Add startup service

1. `chmod +x ~/ros2_ws/script/rover_startup.sh`
2. `sudo cp ~/ros2_ws/script/mr2_rover.service /etc/systemd/system/`
3. `sudo systemctl daemon-reload`
4. `sudo systemctl enable mr2_rover.service`
5. `sudo systemctl start mr2_rover.service`

## Enable wake on lan

[](https://devdebin.tistory.com/343)

1. `sudo apt-get install net-tools ethtool wakeonlan`
2. Find out network interface name using `ifconfig`
3. `sudo vim /etc/systemd/system/wol.service`
```
[Unit]
Description=Configure Wake-up on LAN

[Service]
Type=oneshot
ExecStart=/sbin/ethtool -s <INTERFACE NAME> wol g

[Install]
WantedBy=basic.target
```
4. `sudo systemctl daemon-reload`
5. `sudo systemctl enable wol.service`
6. `sudo systemctl start wol.service`

## Setting realtime kernel
1. `sudo pro enable realtime-kernel`
2. reboot
3. `sudo addgroup realtime`
4. `sudo usermod -a -G realtime $(whoami)`
5. `sudo nano /etc/security/limits.conf`
```
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
```
6. logout and in again
