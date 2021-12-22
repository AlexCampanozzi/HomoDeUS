echo '+--------------------------------------+'
echo '|   HomoDeUS - Initialization Script   |'
echo '+--------------------------------------+'
echo ''

cd ~

echo "* Checking if VSCode is installed.."

if ! dpkg -l "code"; then
    echo "* Installing VSCode..."
    sudo apt update
    sudo apt install software-properties-common apt-transport-https wget

    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update
    sudo apt install code
else
    echo "* VSCode is OK..."
fi


if test -d ./catkin_ws; then
    echo "WARNING: The catkin workspace already exists!"
    read -p "Are you sure you want to delete the current catkin workspace? [y/n]: " clear_repo

    if [ $clear_repo == "y" ]; then
        echo '* Deleting the old catkin workspace...'
        rm -rf ./catkin_ws
    else
        exit 1
    fi

fi

echo '* Creating a new catkin workspace at the root...'
mkdir catkin_ws
cd ./catkin_ws
mkdir src
cd ./src

echo '* Cloning the HomoDeUS repo...'

cd ~/.ssh
if ! test -f ./id_rsa.pub; then
    echo "WARNING: Looks like you don't have a valid key to clone the repo."
    echo "* Checking if XClip is installed..."
    if ! dpkg -l "xclip"; then
        echo "* Installing XClip..."
        sudo apt install xclip
    else
        echo "XClip is OK..."
    fi

    echo "* Follow the instructions to generate a SSH key..."
    ssh-keygen
    xclip -selection clipboard < ~/.ssh/id_rsa.pub

    echo "* Copied the key to your clipboard. Add it on GitHub."
    read -p "Press enter when it's done..."
fi

read -p 'Git username: ' username
read -sp 'Git password: ' password
echo ''
read -p 'Branch to checkout (master recommended): ' branch_name
git config --global user.name $username
git config --global user.password $password

cd ~/catkin_ws/src
#git clone git@github.com:AlexCampanozzi/HomoDeUS.git
#git checkout -b $branch_name
git clone --branch $branch_name git@github.com:AlexCampanozzi/HomoDeUS.git

read -p 'Do you also wish to clone PAL Robotics repos [y/n]: ' clone_pal

if [ $clone_pal == "y" ]; then
    echo '* Cloning PAL Robotics repos...'
    git clone https://github.com/pal-robotics/pal_msgs.git
    git clone https://github.com/pal-robotics/aruco_ros.git
    git clone https://github.com/pal-robotics/tiago_tutorials.git
fi

echo "* Installing HBBA..."

cd ~/catkin_ws/src

sudo apt-get install bison
sudo apt-get install flex
sudo apt-get install libv8-dev
sudo apt-get install libyaml-cpp-dev

git clone https://github.com/introlab/HBBA.git

cd ./HBBA/iw_translator/or_tools
./generate_dpkg.sh
sudo dpkg -i or-tools_ubuntu-18.04_v7.1.6720.deb
cd ../..
git submodule init
git submodule update

echo "* Installing darknet_ros..."

cd ~/catkin_ws/src
git clone --recursive git@github.com:leggedrobotics/darknet_ros.git

echo "* Installing voice dependencies..."

# Voice dependencies
pip2 install pygame
pip2 install gtts
pip2 install pytictoc
pip2 install pydub
sudo apt-get install -y libttspico-utils
sudo apt install ffmpeg

pip2 install SpeechRecognition
sudo apt-get install libasound-dev
sudo apt-get install portaudio19-dev
sudo apt-get install libportaudio2
sudo apt-get install libportaudiocpp0
pip2 install PyAudio
sudo apt-get install python-sphinxbase
sudo apt-get install python-pocketsphinx

echo "* Copying gazebo files..."

sudo cp ~/catkin_ws/src/HomoDeUS/homodeus_common/worlds/homodeus_office.world /opt/pal/ferrum/share/pal_gazebo_worlds/worlds

echo '* Sourcing setup files...'

cd ~/catkin_ws
source /opt/pal/ferrum/setup.bash
source ./devel/setup.bash

echo '* Setup done! Please run catkin_make.'

# counter=1
# catkin_make
# build_exit_status=$?

# while [ ! $build_exit_status -eq 0 ] -a [ $counter -lt 5 ]
# do
# 	echo "* Build failed, trying to resource ($counter/5)..."
# 	source ./devel/setup.bash
# 	let counter = $counter + 1
# 	catkin_make
# 	build_exit_status=$?
# done

# if [ $build_exit_status -eq 0 ]; then
# 	echo "* Environment ready..."
# else
# 	echo " Couldn't build the environment!!!"
# fi
