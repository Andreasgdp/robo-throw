# GripperHandling - Handling of the gripper controller

## Guide to connect to the gripper
- Plug network cable into the computer
  - If problems, try turning wi-fi off
- Configure a new ethernet connection
  - Go to Settings -> Network and press "+" next to Wired to add a new ehternet connection.
  - Give the new connection a name, fx. myConnection
  - Select the IPv4 Settings tab and select Method: Manual. Under Addresses manually type in the Address and Netmask: The Address must be called 192.168.100.X where X is a number 0-255 of your choice (not the same as the gripper). The Netmask must be set to 255.255.255.0 while the Gateway is left blank.
  - Press Add to save the settings
- Disconnect from you current default ethernet connection and connect using the newly configured net-work
- Confirm that the connection is established by pinging the other devices on the network with 
  ```
  ping 192.168.100.X
  ```
  
## Gripper Control Panel
- The grippers control panel can be accesed by establishing a connection and entering the grippers IP in google.
- For the gripper to work properly, go to settings and turn off "Use text based interface".
![image](https://user-images.githubusercontent.com/45763482/135037809-a19bb1fe-807f-46bd-b086-82fcbaa4fcec.png)


## IP Adresses of the hardware this semester (3. semester)
- Cell #1
  - UR: 192.168.100.49
  - WSG: 192.168.100.11
- Cell #2
  - UR: 192.168.100.53
  - WSG: 192.168.100.10
 - Cell #3
   - UR: 192.168.100.53
   - WSG: 192.168.100.10
- Cell #4
  - UR: 192.168.100.49
  - WSG: 192.168.100.10
