from importlib import import_module
prmrobot = import_module('3_prmrobot')
from math import pi
RADpDEG = pi/180

def check_intermediate_angle(source, target, numbers = 6):
    for i in range(numbers + 1):
        print(prmrobot.intermediate_angle(source*RADpDEG, target*RADpDEG, i/numbers, allow_wrap= True)/RADpDEG, end = ', ')
    print()

check_intermediate_angle(-90, 90)
