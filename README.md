# ngimu upperlimb application 

This code evaluates shoulder and elbow ISB angles using 3 NGIMUs: one placed on the torso, one the upper arm and one in the forearm.
The ISB angles for the shoulder are the 'plane of elevation', 'upper arm elevation' and 'humeral rotation', while for the elbow are 
'flexion-extension' and 'pronosupination'. 
A calibration is performed before starting the measurements. 
ISB data and linear acceleration data are saved in a .csv file

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

How to make the IMUs send messages: 
* open NGIMU GUI  software 
* connect an IMU
* go to Settings-> Send Rates-> set the desired frequency for Sensor, Rotation Matrix and Linear Acceleration 

Before running the code, make sure to be connected to "NGIMU Network" (password: "xiotechnologies").


