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
pico2wave -l fr-FR -w test.wav "Test de parole"
sudo apt install ffmpeg
