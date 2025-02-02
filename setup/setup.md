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
