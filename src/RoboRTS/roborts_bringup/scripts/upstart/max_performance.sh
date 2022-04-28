#/usr/bash
sleep 10
echo "Max performance" 
echo lima | sudo -S sudo  "/usr/bin/jetson_clocks"

# disable wifi power saving

echo "Set power_save off "
echo lima | sudo -S sudo iw dev wlan0 set power_save off


echo "open fans"
sudo sh -c "echo '255' > /sys/devices/pwm-fan/target_pwm"


