# Instructions on how to install the verius libraries uesd in this project.

## UR-RTDE
Follow the following steps

- Run the following commands:
```
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
```

- You will also need this dependency

```
sudo apt-get install libboost-all-dev
```

## Robotics Library
Install the following in your ubuntu system

```
sudo apt-add-repository ppa:roblib/ppa
sudo apt-get update
sudo apt-get install librl librl-demos librl-examples librl-dev 
```

## URSim
Carefully follow the following steps to install URSim.

- Download URSim from the official UR website: 
https://www.universal-robots.com/download/?query=ursim

  - We downloaded CB-SERIES - LINUX - URSIM-3.14.3: 
https://www.universal-robots.com/download/software-cb-series/simulator-linux/offline-simulator-cb3-linux-ursim-3143/

- Install Java 8
  ```
  sudo apt-get install openjdk-8-jdk
  ```
  - Confirm Java version
    ```
    java -version
    ```
    The output should look something like this:
    ```
    openjdk version "1.8.0_292"
    OpenJDK Runtime Environment (build 1.8.0_292-8u292-b10-0ubuntu1~20.04-b10)
    OpenJDK 64-Bit Server VM (build 25.292-b10, mixed mode)
    ```
  - If there are multiple versions, set the version to Java 8:
    ```
    sudo update-alternatives --config java
    ```
  <!-- JAVA_HOME might need to be set to the file path of Java 8-->
     
- Install libcurl4
  ```
  sudo apt-get update
  sudo apt-get install libcurl4
  sudo ln -s /usr/lib/x86_64-linux-gnu/libcurl.so.4.5.0 /usr/lib/x86_64-linux-gnu/libcurl.so.4.7.0
  sudo apt-get install php7.2-curl
  ```

- Replace the following line in the install.sh file
  ```
  commonDependencies='libcurl3 libjava3d-* ttf-dejavu* fonts-ipafont fonts-baekmuk fonts-nanum fonts-arphic-uming fonts-arphic-ukai'
  ```
  with
  ```
  commonDependencies='libcurl4 openjdk-8-jre libjava3d-* ttf-dejavu* fonts-ipafont fonts-baekmuk fonts-nanum fonts-arphic-uming fonts-arphic-ukai'
  ```
- Make the following files executable
  ```
  sudo chmod +x filename.sh
  ``` 
    - start-ursim.sh
    - starturcontrol.sh
    - stopurcontrol.sh
    - URControl
    
- Run the installer
  ```
  ./install.sh
  ```
- Start URSim
  ```
  ./start-ursim.sh
  ```
    - If the controller is offline, close URSim and run the following command:
      ```
      sudo ./start-ursim.sh
      ```
