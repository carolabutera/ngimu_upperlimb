# ngimu_upperlimb

How to set IMUs IP addresses (router TpLink TL_WR902AC):
* go to http://tplinkwifi.net 
* access with username "admin" and password "admin"
* in LAN-> NETWORK "LAN Type" has to be set to "static IP" 
* in DHCP-> Address Reservation-> Add New and associate each MAC address of the IMUs to a different IP address between "192.168.0.100","192.168.0.101","192.168.0.102"

How to get the MAC address of each IMU
* connect the router to the computer
* connect to NGIMU Network with password "xiotechnologies"
* open NGIMU GUI (the software is available for Windows only and can be downloaded from https://x-io.co.uk/ngimu/
* connect the IMU 
* go to Settings-> Wi-Fi-> MAC Address 