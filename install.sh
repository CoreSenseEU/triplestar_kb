#!/bin/sh
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y python3-pip 

pip install --break-system-packages pydantic pyoxigraph reasonable oxrdflib shapely copier returns
