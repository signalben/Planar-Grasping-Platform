#!/bin/bash 
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $SCRIPT_DIR
cd ..
cd ..
cd ..
cd ..
cd grasp_generation
cd ggcnn-master

source ~/anaconda3/etc/profile.d/conda.sh
conda activate gr
python3 ./run.py
exit 0

#References
#https://stackoverflow.com/questions/59895/how-can-i-get-the-source-directory-of-a-bash-script-from-within-the-script-itsel
#https://askubuntu.com/questions/849470/how-do-i-activate-a-conda-environment-in-my-bashrc
