import os
import sys
import time
import subprocess
from pathlib import Path
from hpp.corbaserver.manipulation import Client, loadServerPlugin
from omniORB.CORBA import SystemException


class CorbaServer:
    def __init__(self, start=True) -> None:
        super().__init__()
        self.process = None
        if start:
            self.start()

    def start(self):
        
        ld_path = ""
        ros_pkg_path =  f"ROS_PACKAGE_PATH={Path(__file__).parent}/"
        corba_command = f"{ld_path} {ros_pkg_path} hppcorbaserver"

        self.process = subprocess.Popen(corba_command, shell=True)
        print(corba_command)

        assert self.wait_for_corba(), "ERROR: time out, could not start corba server!"

        Client().problem.resetProblem()
        return self

    def reset_problem(self):
        Client().problem.resetProblem()

    @staticmethod
    def wait_for_corba(time_out=100):
        """
        will wait for corba to properly boot up

        :param time_out: how many 0.05s should we wait for corba before time out
        :return: True if success, false if time out
        """

        def block_print():
            sys.stdout = open(os.devnull, "w")

        def enable_print():
            sys.stdout = sys.__stdout__

        block_print()
        for i in range(time_out):
            try:
                loadServerPlugin("corbaserver", "manipulation-corba.so")
                loadServerPlugin("corbaserver", "bin_picking.so")
                enable_print()
                return True
            except SystemException:
                time.sleep(0.05)
            except Exception as e:
                print(e)

        enable_print()
        return False

    def kill(self):
        corba_command = "pkill hppcorbaserver"
        p = subprocess.Popen(corba_command, shell=True)
        p.wait()
        self.process.wait()

    def __del__(self):
        self.kill()

    def restart(self):
        self.kill()
        time.sleep(1)
        self.start()

if __name__ == "__main__":
    corba = CorbaServer()
    