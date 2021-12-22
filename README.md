Welcome to the repository for the HomoDeUS project created by seven students in robotics and software engineering at the Université de Sherbrooke.
Here are the steps to build the project from scratch. You can also run the script 

## Create an ubuntu environment with PAL Robotics packages
You should have access to a .iso file containing installation file for the PAL Robotics environment  
Select Install Development TIAGo 82 during booting to create a permament environment.  
Select Run to have a temporary environment without it taking place in you hard drive

## Credentials
Username: pal  
Password: pal  

Admin username: root  
Admin password: palroot

## Network
If you are in a VM it is possible that you have no internet connection.  
To fix the problem, edit /etc/netplan/01-netcfg.yaml to add this:

	network:
		version: 2
		renderer: NetworkManager
		ethernets:
		    enp0s3:
		        dhcp4: true

verify that the edits worked with

	sudo netplan apply

then restart the server

	sudo ln -sf /run/systemd/resolve/resolv.conf /etc/resolv.conf

Depending on your network configuration, it may be ens33 instead of enp0s3 that you have to put in the file
You can verify it with
	ip a
	
## Configure a catkin workspace
	mkdir -p ~/catkin_ws/src
	cd catkin_ws/src

## Setup your SSH keys
Before being able to use git you have to generate SSH keys for your new environment:  
Procedure here: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

## Clone HomoDeUS
then clone the repo in src

	git clone git@github.com:AlexCampanozzi/HomoDeUS.git

## Install Pip
Since the project is on ROS Melodic, it is in Python 2.7 by default. Pip deprecated support for Python 2.7 so it can't be installed directly. You will have to install it manually from the source.

	cd ~
	sudo apt-get install curl
	curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
	python get-pip.py 

This will install the last compatible version of pip.

## Install dependencies

	cd ~/catkin_ws/src/HomoDeUS
	./homodeus_setup.sh

## Build the project
	
	cd ~/catkin_ws
	catkin_make


