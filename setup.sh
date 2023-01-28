#!/bin/bash

# install apt packages
sudo apt-get update
sudo apt-get upgrade -y

# install miniconda
wget -O miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-py310_22.11.1-1-Linux-x86_64.sh
chmod +x miniconda.sh
./miniconda.sh -u -b
rm miniconda.sh

# init conda base workspace
eval "$($HOME/gitpod/miniconda3/bin/conda shell.bash hook)"
conda init

# create the custom environment file
conda env create -f environment.yml --force
conda activate behappy
