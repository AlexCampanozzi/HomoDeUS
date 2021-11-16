#!/bin/sh

sudo apt-get update
# HBBA Dependencies
sudo apt-get install -y bison
sudo apt-get install -y flex
sudo apt-get install -y libv8-dev
sudo apt-get install -y libyaml-cpp-dev

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

