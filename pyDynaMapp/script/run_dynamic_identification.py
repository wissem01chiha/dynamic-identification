import os 
import sys
import time 
import argparse
import logging 
import nlopt 

from dynamics.robot import Robot
from dynamics.regressor import Regressor
logger = logging.getLogger(__name__)
st = time.time()
