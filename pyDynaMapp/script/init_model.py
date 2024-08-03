# craete a new robot model by initlizing the specific robot folders and 
# write some configs to config.yml files
#
import os 
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--v',type=bool)
parser.add_argument('--robot_name',type=str)
args = parser.parse_args()

base_dir = os. getcwd()
pkg_dir = os.path.join(base_dir,'pyDynaMapp')
sys.path.append(pkg_dir)

