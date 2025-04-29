# unblocl
rfkill unblock all
# flush
sudo ip addr flush dev wlo1
sudo ifconfig wlo1 down
sudo service network-manager stop
sudo iwconfig wlo1 mode ad-hoc
sudo iwconfig wlo1 channel 1
sudo iwconfig wlo1 essid 'djiadhoc'
sudo ip addr add 10.0.0.x/24 broadcast 10.0.0.255 dev wlo1
sudo ifconfig wlo1 up
ifconfig
iwconfig
