# kinova-storage-system
Storgage system utilizing Kinova Gen3 and Gen3 lite robotic arms.

## Instructions
### Set Up
1. Physically set up the robot and the storage space. TODO: add diagram here

2. Connect the robot to your network.

3. Use the Kinova web app to set the color camera resolution to 640x480.

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
python3 -m pip install opencv-python-contrib
```

### Run
In an Ubuntu Bash terminal, run the following commands:
```
cd ~/robotics/kinova-storage-system
source .venv/bin/activate
TODO
```