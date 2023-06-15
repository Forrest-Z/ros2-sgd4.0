### HAW Hamburg
### BMT6-Studienarbeit Shared Dog Project
### Prof. Dr. Henner Gärtner
### created by Helmer Barcos
### helmer@barcos.co - https://barcos.co
### Sommer Semester 2023

from typing import List

import os
import uuid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.logging import LoggingSeverity


from sdtl_interfaces.msg import SDTLPoint
from sdtl_interfaces.action import SDTLPedestrianTrafficLight
from .sdtl_route_manager import SDTLRouteManager

minPoints = 1
classprefix = "[SDTLActionServer] "

class SDTLActionServer(Node):

    def __init__(self):
        super().__init__('sdtl_action_server')
        self.route_manager = SDTLRouteManager(self.get_logger())
        self._action_server = ActionServer(
            self,
            SDTLPedestrianTrafficLight, # type of the action
            'pedestrian_traffic_light', # action name
            self.execute_callback)
    
        self.get_logger().info(classprefix +'Started')

        

    def execute_callback(self, goal_handle : ServerGoalHandle):

        result = SDTLPedestrianTrafficLight.Result()
        uuid_obj = uuid.UUID(bytes=bytes(goal_handle.goal_id.uuid))
        uuid_string = str(uuid_obj)

        prefix =  str(classprefix + "goal " + uuid_string + " ")
        
        ## 1. Start proccessing the goal with this callback
        self.get_logger().info(prefix +'Executing...')

        ## 2. Test if payload sent by the client if ok
        route : List[SDTLPoint] = []
        if goal_handle.request.route:
            route = goal_handle.request.route
        
        ## 2.1. Abort the execution if some requirements are not meet               
        if len(route) <= minPoints:
            message = "aborting since no route or not enough points were provided"
            self.get_logger().error(prefix + message)
            
            result.succeeded = False
            result.message = message
            goal_handle.abort()
            
            return result
        
        self.get_logger().info(prefix + "route provided with " + str(len(route)) + " points")
        self.route_manager.setCurrentRoute(route)
        self.route_manager.registerHandlers(goal_handle.publish_feedback)

        ## 3. Proccess the task if everything ok and send feedbacks. this may block the execution of this action server.
        self.get_logger().info(prefix + "proccesing...")
        try:
            self.route_manager.proccessRoute()
        except Exception as error:
            result.succeeded = False
            result.message = str(error)
            self.get_logger().error(prefix + result.message)
            goal_handle.abort()
            
            return result

        # 4. Terminate the task successfully
        self.get_logger().info(prefix + "terminating..." )
        goal_handle.succeed()
        result.succeeded = True
        result.message = str("Goal with id " + uuid_string + " terminated successfully")
        self.get_logger().info(prefix +  "successfully terminated")
        return result



def main(args=None):
    rclpy.init(args=args)
    action_server = SDTLActionServer()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    


if __name__ == '__main__':
    main()