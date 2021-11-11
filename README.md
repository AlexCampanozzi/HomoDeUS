Welcome to the repository for the HomoDeUS project created by seven students in robotics and software engineering at the Université de Sherbrooke.
Here are the steps to build the project from scratch.

##Create an ubuntu environment with PAL Robotics packages
You should have access to a .iso file containing installation file for the PAL Robotics environment
Select Install Development TIAGo 82 during booting to create a permament environment.
Select Run to have a temporary environment without it taking place in you hard drive

###Credentials
Username:pal
Password:pal

Admin username:root
Admin password:palroot

##Network
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

##Configure a catkin workspace
	mkdir -p ~/catkin_ws/src
	cd catkin_ws/src

###Setup your SSH keys
Before being able to use git you have to generate SSH keys for your new environment:
Procedure here: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent

##Clone HomoDeUS
then clone the repo in src
	git clone git@github.com:AlexCampanozzi/HomoDeUS.git

###Clone HBBA
To make the project work you will also need HBBA. Complete documentation is at https://github.com/introlab/HBBA
	git clone git@github.com:introlab/HBBA.git
	cd iw_translator/or_tools/
	./generate_dpkg.sh
	sudo dpkg -i or-tools_ubuntu-18.04_v7.1.6720.deb


	

