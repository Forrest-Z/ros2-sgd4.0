
### HAW Hamburg
### BMT6-Studienarbeit Shared Dog Project
### Prof. Dr. Henner Gärtner
### created by Helmer Barcos
### helmer@barcos.co - https://barcos.co
### Sommer Semester 2023

import time
import math
import json
from typing import List, Callable
from rclpy.impl.rcutils_logger import RcutilsLogger

from sdtl_interfaces.action import SDTLPedestrianTrafficLight
from sdtl_interfaces.msg import SDTLPoint
from .sdtl_mk5_manager import SDTLMK5Manager
from .v2x_mqtt import DataframColums

classprefix = "[SDTLRouteManager] "
minDistance = 300 # 10 meters

class SDTLRouteManager:
    """The main route manager"""
    
    logger: RcutilsLogger
    publish_feedback: Callable[[SDTLPedestrianTrafficLight.Feedback], None]
    
    def __init__(self, logger : RcutilsLogger):
        self.current_route = []
        self.mk5_manager = SDTLMK5Manager(logger=logger)
        self.logger = logger
        self.mk5_manager.connect()
    
    def setCurrentRoute(self, current_route : List[SDTLPoint]):
        self.current_route = current_route
        
    def registerHandlers(self, publish_feedback):
        self.publish_feedback = publish_feedback
        
    
    # TODO: improve
    # this could be improved for accurate results
    # information about the baering could be helpful     
    def proccessRoute(self):
        
        if self.publish_feedback == None:
            raise(ValueError("You need to register a publish_feedback"))
        
        # shouldLoop = True
        # while shouldLoop:
        for i in range(30):
            self.evalPreconditions()
            sortedData = self.getLastItersections()
            bestIntersectionId, bestSignalGroup = self.findNearIntersection(sortedData)
            
            if bestIntersectionId != None and bestSignalGroup != None:
                result = self.mk5_manager.v2xmqtt.findMovementStates(bestIntersectionId, bestSignalGroup)
                print(json.dumps(result, indent=4))

            time.sleep(2)
            
            
        
            
    
    # retrive the last 3 intersections
    def getLastItersections(self):
        sorted_data = sorted(self.mk5_manager.v2xmqtt.mainDataset.items(), key=lambda x: x[1]["updated_at"], reverse=True)
        return [{"intersectionId": intersection_id, **data} for intersection_id, data in sorted_data[:3]]
    
    
    def findNearIntersection(self, sorted_data):
        prefix = classprefix + "[findNearIntersection] "
        
        bestIntersectionId = None
        bestSignalGroup = None
        matchCounts = 0
        
        try:
    
            for entry in sorted_data:
                intersection_id = entry["intersectionId"]

                if "lanes" not in entry:
                    self.logger.debug(prefix + "intersectionId={intersectionId} | lanes not found".format(intersectionId=intersection_id))
                    continue
                
                localMatchCounts = 0
                lanes = entry["lanes"]
                minFoundDistance = minDistance
                
                for lane in lanes:
                    
                    if "nodes" not in lane:
                        self.logger.debug(prefix + "intersectionId={intersectionId} | nodes not in lane laneId={laneId}"
                                          .format(intersectionId=intersection_id, laneId=lane["laneId"]))
                        continue
                
                    if "coordinates" not in lane["nodes"]:
                        self.logger.debug(prefix + "intersectionId={intersectionId} | coordinates not in lane.nodes laneId={laneId}"
                                          .format(intersectionId=intersection_id,laneId=lane["laneId"]))
                        continue
                    
                    if "connections" not in lane:
                        self.logger.debug(prefix + "intersectionId={intersectionId} | connections not in lane laneId={laneId}"
                                          .format(intersectionId=intersection_id,laneId=lane["laneId"]))
                        continue
   
   
                    if  len(lane["connections"]) <= 0:
                        self.logger.debug(prefix + "intersectionId={intersectionId} | non connections found laneId={laneId} "
                                          .format(intersectionId=intersection_id, laneId=lane["laneId"]))
                        continue
                    
                    coordinates = lane["nodes"]["coordinates"]
                    for coordinate in coordinates:
                        latitude = coordinate[1]
                        longitude = coordinate[0]
                        
                        distance = self.calculate_distance(latitude, longitude, self.mk5_manager.currentLatitude, self.mk5_manager.currentLongitude)
                        self.logger.debug(prefix + "intersectionId={intersectionId} | distance={distance}m | laneId={laneId}"
                                          .format(intersectionId=intersection_id,distance=round(distance, 2), laneId=lane["laneId"]))

                        if distance <= minFoundDistance:
                            minFoundDistance = distance
                            # TODO it could be more than 1 connections
                            bestSignalGroup = lane["connections"][0]["signalGroupId"]
                            self.logger.debug(prefix + "new bestSignalGroup {group} found".format(group=bestSignalGroup))

                        
                        if distance <= minDistance:
                            localMatchCounts = localMatchCounts + 1
                        
                if localMatchCounts > matchCounts:
                    matchCounts = localMatchCounts
                    bestIntersectionId = intersection_id
                    
        except Exception as error:
            self.logger.error(prefix + str(error))

        self.logger.info(prefix + "best matches bestIntersectionId={a} bestSignalGroup={b}".format(a=bestIntersectionId, b=bestSignalGroup))

        return bestIntersectionId, bestSignalGroup
        

    
    # returns the distance in meters
    def get_distances(self, current_route_point, next_route_point):
        latitude = self.mk5_manager.currentLatitude
        longitude = self.mk5_manager.currentLongitude,
        distance_to_current = self.calculate_distance(
            latitude,
            longitude,
            current_route_point.latitude,
            current_route_point.longitude,                
        )
        
        distance_to_next = self.calculate_distance(
            latitude,
            longitude,
            next_route_point.latitude,
            next_route_point.longitude,                
        )
        
        return distance_to_current, distance_to_next
    
        
    def evalPreconditions(self):
        if not self.mk5_manager.isRunning():
            raise Exception("MK5_CONNECTION_ERROR: mk5 not running or still not connected")
        
        if not self.mk5_manager.hasCurrentLocation():
            raise Exception("MK5_GPS_SIGNAL_ERROR: mk5 is not returning a current location")
        
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
         # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371 * c  # Radius of the Earth in kilometers
        
        return distance * 1000 

            
        
        

        
    
# for i in range(len(self.current_route)-1):
#     current_route_point : SDTLPoint = self.current_route[i]
#     next_route_point = self.current_route[i+1]
    
#     shouldLoop = True
#     is_next_to_current_point = False
#     is_next_to_next_point = False
    
#     was_already_next_to_current_point = False
    
#     while shouldLoop:
#         distance_to_current, distance_to_next = self.get_distances(current_route_point, next_route_point)
#         is_next_to_current_point = distance_to_current <= minDistance
        
            
#         if was_already_next_to_current_point and not is_next_to_current_point:
#             shouldLoop = False
        
#         if distance_to_current <=

# for i in range(30):
#     self.evalPreconditions()
    
#     feedback = SDTLPedestrianTrafficLight.Feedback()
#     feedback.green = False
#     feedback.red = True
#     feedback.next_change_at = "no idea"    
#     self.publish_feedback(feedback)
#     self.logger.debug(classprefix + "feedback published")
#     time.sleep(2) # sleep for 2 seconds
        
# 53.5559309,9.9772394
# 53.5559558,9.9771795
# 53.5559807,9.9771196
# 53.5560056,9.9770596
# 53.5560305,9.9769997
# 53.5560554,9.9769398
# 53.5560778,9.9769635
# 53.5561001,9.9769872
# 53.556126,9.9770146
# 53.5561519,9.977042
# 53.5561826,9.9770747
# 53.5562133,9.9771073
# 53.5562336,9.9771345
# 53.5562573,9.9771066
# 53.556281,9.9770786
# 53.5563022,9.9770227