import sys
import os

sys.path.append(os.getcwd() + "/src")

from controller import Controller


def get_sandbox_controller():
    return Controller("localhost:50000")