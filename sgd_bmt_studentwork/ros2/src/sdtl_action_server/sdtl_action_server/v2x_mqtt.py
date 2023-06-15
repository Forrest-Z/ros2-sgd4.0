#!/usr/bin/env python3.9

### HAW Hamburg
### BMT6-Studienarbeit Shared Dog Project
### Prof. Dr. Henner Gärtner
### created by Fabian Petersen
### Co-Author: Helmer Barcos
### Sommer Semester 2023

import os
import json
import datetime
from enum import Enum
from typing import Any, Dict, List

import paho.mqtt.client as mqtt

from rclpy.impl.rcutils_logger import RcutilsLogger



class PedestrianLight(Enum):
    UNKNOWN = 0
    RED = 1
    GREEN = 2
    DARK = 3
    

class V2XTopics(Enum):
    MAP = "v2x-uca/output/json/map"
    SPAT = "v2x-uca/output/json/spat"
    GNSS = "v2x/rx/obu_gnss"
    

class DataframColums(Enum):
    INTERSECTION_ID = "intersectionId"
    MOVEMENT_STATES = "movementStates"
    LANES = "lanes"
    

classprefix = "[V2XMqtt] "

MQTT_HOST = "MQTT_HOST"
MQTT_PORT = "MQTT_PORT"

class V2XMqtt:
    '''
    This class is used to connect to the Conda Wireless device via MQTT.
    It processes the incoming data and returns the pedestrain lanes with all relevent data.
    '''
    
    mqttHost = "localhost"
    mqttPort = 1883
    mqttKeepAlive = 60
            
    logger: RcutilsLogger
    
    onPositionUpdate: None

    def __init__(self, logger: RcutilsLogger):       
        
        # create the client
        prefix = classprefix + "[__init__] "
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.connected = False
        
        self.result = {}
        self.logger = logger
        self.mainDataset: Dict[str, Dict[str, Any]] = {}
        self.configureEnv()
        
        try:
            self.logger.info(prefix + "conecting to mqtt {host}:{port}".format(host=self.mqttHost, port=self.mqttPort))
            self.client.connect(self.mqttHost, self.mqttPort, self.mqttKeepAlive)
            self.connected = True
            self.logger.info(prefix + "conected successfully")
            
        except Exception as error:
            self.logger.error(prefix + str(error))
            self.logger.error(prefix + "No connection to MQTT broker. Please check the right IP adress and the connection to the broker.")
            exit(1)
    
    
    def run(self):
        self.client.loop_forever()
        
               
    def configureEnv(self):
        prefix = classprefix + "[configureEnv] "
        self.logger.debug(prefix + "starting...")

        env_host = os.getenv(MQTT_HOST)
        env_port = os.getenv(MQTT_PORT)
        
        if env_host is not None:
            self.logger.info(prefix + "setting {some_var}={some_value}".format(some_var=MQTT_HOST, some_value=env_host))
            self.mqttHost = str(env_host)
            
        if env_port is not None:
            self.logger.info(prefix  + "setting {some_var}={some_value}".format(some_var=MQTT_PORT, some_value=env_host))
            self.mqttPort = int(env_port)

        self.logger.debug(prefix + "finished")


    def on_connect(self, client, userdata, flags, rc):
        prefix = classprefix + "[on_connect] "
        self.logger.info(prefix + "Connected with result code " + str(rc))
        client.subscribe(V2XTopics.MAP.value)
        client.subscribe(V2XTopics.SPAT.value)
        client.subscribe(V2XTopics.GNSS.value)

    def on_message(self, client, userdata, msg : mqtt.MQTTMessage):
        
        if msg.topic == V2XTopics.MAP.value:
            self.proccess_map(V2XTopics.MAP.value, msg.payload)
                
        elif msg.topic == V2XTopics.SPAT.value:
            self.proccess_spat(V2XTopics.SPAT.value, msg.payload)
            
        elif msg.topic == V2XTopics.GNSS.value:
            self.proccess_gnss(msg.payload)
            
        else:        
            message = "processed message for topic {var1}".format(var1=msg.topic)
            self.logger.warn(classprefix + "[on_message] " + message)
        
    
    def proccess_map(self, topic: str, payload: Any):
        self.proccess_message(topic, payload, DataframColums.LANES.value)
        print("")
     
    
    def proccess_spat(self, topic: str, payload: Any):
        self.proccess_message(topic, payload, DataframColums.MOVEMENT_STATES.value)
        print("")

        
    def proccess_gnss(self, payload : Any):
        prefix = classprefix + "[proccess_gnss] "
        self.logger.debug(prefix + "starting...")

        try:
            converted = json.loads(payload) 
            gnss = converted["gnss"]
            latitude = gnss["latitude"]
            longitude = gnss["longitude"]
            
            if self.onPositionUpdate is not None:
                self.onPositionUpdate(latitude, longitude)

            self.logger.debug(prefix + "basic values: latitude={lat} longitude={lon} speed_mps={speed} numSatUsed={numSatUsed}"
                                .format(lat=latitude, lon=longitude, speed=gnss["speed_mps"], numSatUsed=gnss["satinfo"]["numSatUsed"]))
            
            self.logger.debug(prefix + "finished")
        except Exception as error:
            self.logger.error(prefix + str(error))
        
            
    
    def proccess_message(self, topic: str, payload: Any, targetColumn: str, callback = None):
        prefix = classprefix + "[proccess_message] for topic {var1} ".format(var1=topic) 
        self.logger.debug(prefix + "starting...")
        
        try:
            converted = json.loads(payload)
            extractedData, intersectionId = self.extractData(converted, targetColumn)
            if intersectionId is not None and extractedData is not None:        
                # add the intersection id and the payload to the dataframe if the intersection id is not already in the dataframe
                if intersectionId in self.mainDataset:
                    data = extractedData
                    if targetColumn == DataframColums.LANES.value:
                        filteredData = self.filterLaneTypes('pedestrian', extractedData)
                        data = filteredData
                    
                    self.updateDataframe(intersectionId, targetColumn, data)
                else:
                    
                    data = {}
                    if targetColumn == DataframColums.LANES.value:
                        filteredData = self.filterLaneTypes('pedestrian', extractedData)
                        data[targetColumn] = filteredData
                    
                    if targetColumn == DataframColums.MOVEMENT_STATES.value:
                        data[targetColumn] = extractedData

                    self.appendToData(intersectionId, data)
                                               
                if callback is not None:
                    callback(intersectionId)
                
                self.logger.debug(prefix + "finished")
                
        except Exception as error:
            self.logger.error(prefix + str(error))
        

    
    # get always 1328
    def getLightState(self, intersectionId: int, signalGroupId) -> PedestrianLight:
        prefix = classprefix + "[getLightState] "
        result = PedestrianLight.UNKNOWN.value
        self.logger.debug(prefix + "starting...")

        try:
            movement  = self.findMovementStates(intersectionId, signalGroupId)
            
            if movement != None and len(movement) > 0:
                req = movement[0]["events"][0]["eventState"]
            
                if req == "red":
                    result = PedestrianLight.RED.value 
                if req == "green":
                    result = PedestrianLight.GREEN.value 
                if req == "dark":
                    result = PedestrianLight.DARK.value 
                
            self.logger.debug(prefix + "finished")
        except Exception as error:
            self.logger.error(prefix + str(error))
            
        return result
    

    def updateDataframe(self, intersectionId: int, targetColumn: str, data: Any):
        '''updateDataframe updates a targetColumnName in a pandas DataFrame based on a condition.'''
        prefix = classprefix + "[updateDataframe] intersectionId={var1} targetCol={var2} ".format(var1=intersectionId, var2=targetColumn)
        self.logger.debug(prefix + "starting...")
        try:
            self.mainDataset[intersectionId][targetColumn] = data
            self.mainDataset[intersectionId]["updated_at"] = datetime.datetime.now()
            self.printCurrentStatus(intersectionId, prefix)
            self.logger.debug(prefix + "finished")
        except Exception as error:
            self.logger.error(prefix + str(error))

    
    def appendToData(self, intersectionId:str, data):
        '''appendToData append the new intersection id and information'''
        prefix = classprefix + "[appendToData] intersectionId={var1} ".format(var1=intersectionId)
        self.logger.debug(prefix + "starting...")
        try:
            self.mainDataset[intersectionId] = data
            self.mainDataset[intersectionId]["updated_at"] = datetime.datetime.now()
            
            # print(json.dumps(self.mainDataset[intersectionId], indent=4))
            self.printCurrentStatus(intersectionId, prefix)
            self.logger.debug(prefix + "current status: intersectionsCount={var1}".format(var1=len(self.mainDataset)))

            self.logger.debug(prefix + "finished")
        except Exception as error:
            self.logger.error(prefix + str(error))
            
    
    def extractData(self, payload, targetColumn : str):
        '''extractData extracts from spat or map '''
        prefix = classprefix + "[extractData] "
        self.logger.debug(prefix + "starting...")
        try:
            self.logger.debug(prefix + "finished")
            return payload[targetColumn], payload[DataframColums.INTERSECTION_ID.value] 
        except Exception as error:
            self.logger.error(prefix + str(error))
        
        return None, None
    
    def filterLaneTypes(self, laneType: str, data: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        filtered_lanes = [lane for lane in data if lane.get('laneType') == laneType]
        return filtered_lanes
    
    def printCurrentStatus(self, intersectionId: str, prefix):
        
        main = self.mainDataset[intersectionId]        
        var2 = 0 if DataframColums.LANES.value not in main else len(main[DataframColums.LANES.value])
        var3 = 0 if DataframColums.MOVEMENT_STATES.value not in main else len(main[DataframColums.MOVEMENT_STATES.value])
        
        self.logger.debug(prefix + "current status: intersectionId={var1} | lanes={var2} | movementsStates={var3}"
                            .format(var1=intersectionId, var2=var2, var3=var3))


     # retrive the last 3 intersections
    def findMovementStates(self, bestIntersectionId : int, bestSignalGroup : int):
        prefix = classprefix + "[findMovementStates] "
        result = None
        data = self.mainDataset[bestIntersectionId]
        movementStates = data[DataframColums.MOVEMENT_STATES.value]    
            
        for movementState in movementStates:
            if "signalGroupId" in movementState and movementState["signalGroupId"] == bestSignalGroup:
                result = movementState
                break
            
        if result == None:
            self.logger.error(prefix + " no movementState was found for interactionId={a} and signalGroupId={b}".format(a=bestIntersectionId, b=bestSignalGroup))
            
        return result


# # function for live testing on an defined intersection (the first pedestrian lane LightState is printed)
# def live_test(self, intersectionID=1328):
#     # proof if the Key intersectionID exists in result
#     if intersectionID in self.result and intersectionID != None:
#         if 'movementState' in self.result[intersectionID][0]:
#             if self.result[intersectionID][0]['movementState']:
#                 print("The actual light state is:", self.result[intersectionID][0]['movementState'][0]["eventState"])


# if __name__ == "__main__":
#     v2xmqtt = V2XMqtt()
#     t0 = threading.Thread(target=v2xmqtt.run)
#     t0.start()
#     # request data from the dataframe with 1Hz
#     while True:
#         import time
#         time.sleep(1)
#         RESULT = v2xmqtt.request()
#         print("The actual light state is:", RESULT.get(1328, [{}])[0].get('movementState', [{}])[0].get('eventState'))