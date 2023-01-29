#!/bin/bash

# install apt packages
sudo apt-get update
#sudo apt-get upgrade -y

# install miniconda
wget -O miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
chmod +x miniconda.sh
bash miniconda.sh -b
rm miniconda.sh

# init conda base workspace
eval "$($HOME/miniconda3/bin/conda shell.bash hook)"
. ~/.bashrc
conda init
conda install -y conda

# create the custom environment file
conda env update --file environment.yml --prune
echo "conda activate behappy" >> ~/.bashrc
. ~/.bashrc
