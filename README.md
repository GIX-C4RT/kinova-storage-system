# kinova-storage-system

## Instructions
### Set Up
1. Physically set up the robot and the storage space. TODO: add diagram here

2. Connect the robot to your network by following [this guide](https://www.kinovarobotics.com/sites/default/files/UG-014_KINOVA_Gen3_Ultra_lightweight_robot_User_guide_EN_R06_0.pdf#%5B%7B%22num%22%3A270%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C66.692%2C183.727%2Cnull%5D).

3. Use the [Kinova web app](http://192.168.1.10/configurations) to set the color camera resolution to 640x480.

4. Download the following file: [link](https://artifactory.kinovaapps.com/ui/api/v1/download?repoKey=generic-public&path=kortex%2FAPI%2F2.3.0%2Fkortex_api-2.3.0.post34-py3-none-any.whl)

5. In an Ubuntu Bash terminal, run the following commands:
```
sudo apt install python3, python3-pip # install python3 and pip
python3 -m pip install --upgrade # upgrade pip
cd # go to home directory
mkdir robotics # make new robotics directory
cd robotics # go to robotics directory
# clone this repository
git clone https://github.com/GIX-C4RT/kinova-storage-system.git
cd kinova-storage-system # go to kinova-storage-system directory
python3 -m venv .venv # create new virtual environment
source .venv/bin/activate # activate the virutual environment
# install the kinova kortex python api
python3 -m pip install ~/Downloads/kortex_api-2.3.0.post34-py3-none-any.whl
# install OpenCV with ArUco support
python3 -m pip install opencv-contrib-python
```

### Run
In an Ubuntu Bash terminal, run the following commands:
```
cd ~/robotics/kinova-storage-system
source .venv/bin/activate
python3 storage_device.py
```
