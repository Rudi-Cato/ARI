#!/bin/bash

echo "Listing installed Python packages..."
pip list > installed_python_packages.txt
echo "Python packages saved to installed_python_packages.txt"

echo "Listing installed system packages related to audio and python..."
dpkg --get-selections | grep -E 'pyaudio|alsa|python3|libasound|pulseaudio' > installed_audio_python_sys_packages.txt
echo "System packages saved to installed_audio_python_sys_packages.txt"

echo "Done."
