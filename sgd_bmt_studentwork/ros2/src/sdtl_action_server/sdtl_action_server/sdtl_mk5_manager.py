### HAW Hamburg
### BMT6-Studienarbeit Shared Dog Project
### Prof. Dr. Henner Gärtner
### created by Helmer Barcos
### helmer@barcos.co - https://barcos.co
### Sommer Semester 2023


# from typing import List
# from sdtl_interfaces.msg import SDTLPoint
import numbers
import threading

from rclpy.impl.rcutils_logger import RcutilsLogger
from .v2x_mqtt import V2XMqtt

def is_number(variable):
    return isinstance(variable, numbers.Number)

class SDTLMK5Manager:
    """The main manager for connecting to the mk5 and creating mqtt connections"""
    
    v2xmqtt: V2XMqtt
    logger: RcutilsLogger
    v2xmqttMainThread : threading.Thread
    
    currentLatitude : 0.0
    currentLongitude: 0.0

    def __init__(self, logger: RcutilsLogger):        
        self.currentLocation = None
        self.v2xmqtt = V2XMqtt(logger=logger)
        self.logger = logger
        self.v2xmqtt.onPositionUpdate = self.onPositionUpdate
    
    def connect(self):
        # start the v2x client in a separated thread
        # while not self.v2xmqtt.connected:
        self.v2xmqttMainThread = threading.Thread(target=self.v2xmqtt.run)
        self.v2xmqttMainThread.start()
        
    def isRunning(self) -> bool:
        return self.v2xmqtt.connected
    
    def hasCurrentLocation(self) -> bool:
        areNumbers = is_number(self.currentLatitude) and is_number(self.currentLongitude)
        haveValues = abs(self.currentLatitude) > 0.0 and abs(self.currentLongitude) > 0.0
        return haveValues and areNumbers
    
    def onPositionUpdate(self, latitude: float, longitude: float):
        self.currentLatitude = latitude
        self.currentLongitude = longitude
    
        