import os
import numpy as np
from roboticstoolbox.robot.ERobot import ERobot

class TwoLink(ERobot):
    '''
    Initialize a 2link robot
    '''
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read("two_link.urdf", tld=os.getcwd())
        super().__init__(
            links,
            name=name,
            manufacturer="ARClab",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )
        
        self.qr = np.array([0, 0.0])
        self.qz = np.zeros(2)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)