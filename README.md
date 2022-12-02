# ngimu upperlimb application 

Evaluation of shoulder and elbow ISB angles using 3 NGIMUs: one placed on the torso, one the upper arm and one in the forearm.
The ISB angles for the shoulder are the 'plane of elevation', 'upper arm elevation' and 'humeral rotation', while for the elbow are 'flexion-extension' and 'pronosupination'.  
ISB data and linear acceleration data are saved in a .csv file

*  "ngimu_demo_imu.py" performs an initial calibration of the IMUs. The reference frame in which the ISB angle are calculated is
 the one with z-axis upward, y-axis perpendicular to the torso and x-axis pointing from torso to right arm. 
The calibration consists in 2 phases: 10 seconds in which the subject has to stand with arm along sides and another 10 seconds in which the subject has to lift arms of 90° placing them horizontally. 

* [work in progress] "ngimu_demo_isb.py"  performs a calibration and evaluate ISB angle with respect to the ISB convention reference frame (y_axis upward, x-axis perpendicular to torso and z-axis pointing to the right arm)

* [work in progress] "ngimu_offline.py" performs calibration and evaluates ISB angles on data acquired with the humanoid robot TIAGo. This script is used to validate the proposed calibration method. "synchro_tiago_imu.csv" is a file that contains the sychronized output of the IMUs and TIAGo encoders (joint values).

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

NB: Before running the code, make sure to be connected to "NGIMU Network" (password: "xiotechnologies").

To solve  error "ImportError: You must be root to use this library on linux" run the script with command

```
sudo -E ./ngimu_demo_imu.py
```


