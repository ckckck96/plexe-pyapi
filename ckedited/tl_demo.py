import os
import sys
from math import floor
​
import random
from utils import add_platooning_vehicle, start_sumo, running, communicate
​
if 'SUMO_HOME' in os.environ:
   tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
   sys.path.append(tools)
else:
   sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from plexe import Plexe, ACC, CACC, RPM, GEAR, RADAR_DISTANCE
​
# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# platoon size
PLATOON_SIZE = 8
# number of platoon
PLATOON_NUM = 1
# cruising speed
SPEED = 33 # m/s
# distance between multiple platoons
PLATOON_DISTANCE = SPEED * 1.5 + 2 # PLATOON_DISTANCE = 51.5
# vehicle who starts to brake
BRAKING_VEHICLE = "v.0.0"
# the original leader ID
ORI_LEADER_ID ="v.0.0"
# traffic light ID
TL_ID = "tl_0"
# range of traffic light broadcast
RANGE = 100 
# traffic light position
TL_POS = 558
​
​
​
def add_vehicles(plexe, n, n_platoons, real_engine=False):
 """
 Adds a set of platoons of n vehicles each to the simulation
 :param plexe: API instance
 :param n: number of vehicles of the platoon
 :param n_platoons: number of platoons
 :param real_engine: set to true to use the realistic engine model,
 false to use a first order lag model
 :return: returns the topology of the platoon, i.e., a dictionary which
 indicates, for each vehicle, who is its leader and who is its front
 vehicle. The topology can the be used by the data exchange logic to
 automatically fetch data from leading and front vehicle to feed the CACC
 """
 # add a platoon of n vehicles
     topology = {}
     p_length = n * LENGTH + (n - 1) * DISTANCE # p_length = (8*4)+(8-1)*5 = 67
     for p in range(n_platoons): # p = 0
        for i in range(n): # n = 7
           vid = "v.%d.%d" % (p, i)

           # Differences with the brakedemo.py is the value = 200 in the 3rd param (position) in add_platooning vehicle function
           # (n-p+1) * (p_length + PLATOON_DISTANCE) = 8 * (67 + 51.5) not equal 200
           # Value 200 is probably something related to the network file of the car position on the road
           add_platooning_vehicle(plexe, vid, 200 + (n-i+1) * (DISTANCE+LENGTH), 0, SPEED, DISTANCE, real_engine)
           plexe.set_fixed_lane(vid, 0, False)
           traci.vehicle.setSpeedMode(vid, 0)
           
           plexe.use_controller_acceleration(vid, False)
           # if we want to try without cacc, we can just set for the else part to acc. 
           if i == 0: # leader is ACC
               plexe.set_active_controller(vid, ACC)
           else:
               plexe.set_active_controller(vid, CACC)
           if i > 0: # follower is CACC
               topology[vid] = {"front": "v.%d.%d" % (p, i - 1),  "leader": "v.%d.0" % p}
           else:
           topology[vid] = {}
     return topology
​
​
def main(demo_mode, real_engine, setter=None):
     # used to randomly color the vehicles
     random.seed(1)
     start_sumo("cfg/intersection/intersection.sumo.cfg", False)
     plexe = Plexe()
     traci.addStepListener(plexe)
     step = 0
     topology = dict()
     min_dist = 1e6

     # newly added split feature compared with brakedemo.py
     split = False
​
     while running(demo_mode, step, 3000):
​         
         # We can add reset simulation here just like the brakedemo.py

         traci.simulationStep()
​
         if step == 0:
             # create vehicles and track the braking vehicle
             topology = add_vehicles(plexe, PLATOON_SIZE, PLATOON_NUM, real_engine) # PLATOON_SIZE = 8, PLATOON_NUM = 1
             tracked_veh = "v.0.%d" %(PLATOON_SIZE-1)
             traci.gui.trackVehicle("View #0", tracked_veh)
             traci.gui.setZoom("View #0", 2000)
​             # at this part we can set another View #1 to tracked the first platoon group pass the traffic light
             traci.gui.trackVehicle("View #1", ORI_LEADER_ID)
             traci.gui.setZoom("View #1", 2000)

         '''This part is newly added'''
         # when the leader is 100m away from the traffic light, it will receive the current phase of the traffic light
         # Accordingly, it computes which vehicles could pass safely.
         leader_data = plexe.get_vehicle_data(ORI_LEADER_ID) # it will return the vid
         # the structure of vehicle data is defined in vehicle_data.py file in plexe folder
         # self.acceleration,  self.speed, self.pos_x, self.pos_y 
         if leader_data.pos_x >= TL_POS - RANGE and not split: # split is false 
             current_phase = traci.trafficlight.getPhase(TL_ID)
             if current_phase == 0:
                 absolute_time = traci.trafficlight.getNextSwitch(TL_ID)
                 time_left = absolute_time - traci.simulation.getTime()
                 new_leader = floor((leader_data.speed * time_left - RANGE)/(LENGTH + DISTANCE))
                 new_leader_id = "v.0.%d" % new_leader
                 # change topology: add new leader and decelerate.
                 for i in range(new_leader+1,PLATOON_SIZE):
                 topology["v.0.%d" %i]["leader"] = new_leader_id
                 topology[new_leader_id] = {}
                 new_leader_data = plexe.get_vehicle_data(new_leader_id)
                 decel = new_leader_data.speed**2 / (2* (RANGE + new_leader * (LENGTH + DISTANCE)))
                 plexe.set_fixed_acceleration(new_leader_id, True, -1 * decel)
              split = True
​         '''This part is newly added *end'''

         # set color for leader
         for i in range(PLATOON_SIZE):
             vid = "v.0.%d" % i
             if topology[vid] == {}:
                 traci.vehicle.setColor(vid, (250,0,0, 255))
​
         if step % 10 == 1:
             # simulate vehicle communication every 100 ms (normal)
             communicate(plexe, topology)

         # We don't need this part
         if real_engine and setter is not None:
             # if we are running with the dashboard, update its values
             tracked_id = traci.gui.getTrackedVehicle("View #0")
             if tracked_id != "":
                 ed = plexe.get_engine_data(tracked_id)
                 vd = plexe.get_vehicle_data(tracked_id)
                 setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)
​
         '''This is the part to handle when the platooning has been splitted'''
         if split == True:
               new_leader_data = plexe.get_vehicle_data(new_leader_id)
               current_phase = traci.trafficlight.getPhase(TL_ID)
               if TL_POS - new_leader_data.pos_x < 10 and current_phase == 0: 
                   plexe.set_fixed_acceleration(new_leader_id, True, 3)
​         '''This is the part to handle when the platooning has been splitted *end'''

         # record the minumum distance (normal)
         if step > 1:
               radar = plexe.get_radar_data("v.0.1")
               if radar[RADAR_DISTANCE] < min_dist:
                     min_dist = radar[RADAR_DISTANCE]
​
         step += 1
         
         # this is not necessary, can be comment out (I think)
         if step > 3000:
             break
​
     traci.close()
​
​
if __name__ == "__main__":
 main(True, True)
