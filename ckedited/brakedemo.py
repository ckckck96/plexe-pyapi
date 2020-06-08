import os
import sys
import random
​
# 调用 utils module，里面包含了 platooning 的中层实现函数
from utils import add_platooning_vehicle, start_sumo, running, communicate
​
# 确保路径设置正确，python 能够搜索到 traci module
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
​
# 调用 plexe module, 包含了 platooning 的底层实现函数
from plexe import Plexe, ACC, CACC, RPM, GEAR, RADAR_DISTANCE
​
# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# cruising speed
# 120 km/h = 33.3 m/s
SPEED = 120/3.6  
# distance between multiple platoons
# 不同 Platoon 之间的距离，在该 demo 中只有一个 platoon
PLATOON_DISTANCE = SPEED * 1.5 + 2
# vehicle who starts to brake
# 最初开始刹车的是头车，编号为 "v.0.0"，即 0 号 platoon 中的 0 号车辆
# 由于只有一个 platoon，所以所有车辆编号都是 "v.0.* "的形式
BRAKING_VEHICLE = "v.0.0"
​
# 构造 platoon，返回 platoon 中车辆之间的通信连接关系
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
 # topology 中以 dictionary 的形式存储车辆之间的通信拓扑结构，包括每辆车的前车和头车编号
    topology = {}
    p_length = n * LENGTH + (n - 1) * DISTANCE      # p_length is the whole length of the platooning formation
    for p in range(n_platoons):                     # n_platoons = number of platoons = 0 (only 1 platoon in this case)
        for i in range(n):
           vid = "v.%d.%d" % (p, i)                 # p is always zero, i will change according to the number of vehicles in the platoon. 
           # vid = vehicle id

           # 调用 utils module 中的函数，将车辆编排成 platoon 的形式
           add_platooning_vehicle(plexe, vid, (n-p+1) * (p_length + PLATOON_DISTANCE) + (n-i+1) *       
                                           (DISTANCE+LENGTH), 0, SPEED, DISTANCE,  real_engine)

           # 将车辆保持在 lane 0 ，并忽略安全距离限制
           plexe.set_fixed_lane(vid, 0, False)

           # 设置车辆速度模式。车辆的速度有 5 个影响因素
           # https://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#speed_mode_.280xb3.29
           # 1\. safe speed  
           # 2\. max accel
           # 3\. max speed
           # 4\. right of way at intersections
           # 5\. brake hard to avoid passing red light
           # 如果全都不考虑，则设置为 [0, 0, 0, 0, 0] = 0, 此时，车速完全由 traci 控制
           # 如果全都考虑，则设置为 [1, 1, 1, 1, 1] = 31 (dec)
           # 如果只考虑 safe speed，则设置为 [0, 0, 0, 0, 1] = 1
           traci.vehicle.setSpeedMode(vid, 0)

           # 在 platoon 中头车采用 adaptive cruise control 的控制方式
           # 后边的跟车采用 cooperative adative cruise control 的控制方式
           plexe.use_controller_acceleration(vid, False)
           if i == 0:
               plexe.set_active_controller(vid, ACC)
           else:
               plexe.set_active_controller(vid, CACC)

           if i > 0:
               topology[vid] = {"front": "v.%d.%d" % (p, i - 1), "leader": "v.%d.0" % p}    
               # this is to update the latest key and values of the topology dic  
           else:
               topology[vid] = {}
               # this is to clear the dic to a completely blank dic if i = 0
    return topology
​​
def main(demo_mode, real_engine, setter=None):    
     # used to randomly color the vehicles
     # 具体着色是在 utils module 的 add_platooning_vehicle 函数中实现的
     random.seed(1)

     # 运行 SUMO 的配置文件，后边的参数 False / True 表示 SUMO server 是否已经在运行了。
     # 若为 False，则打开 SUMO 并加载配置文件
     # 若为 True，则重新加载配置文件
     # freeway.sumo.cfg 中仿真步长为 0.01s
     start_sumo("cfg/freeway.sumo.cfg", False)

     # 以下设置可以使得每次 traci.simulationStep() 之后都调用一次 plexe 
     plexe = Plexe()
     traci.addStepListener(plexe)

     step = 0
     topology = dict()
     min_dist = 1e6
​
     # 主循环
     while running(demo_mode, step, 1500):
​
         # when reaching 15 seconds, reset the simulation when in demo_mode
         if demo_mode and step == 1500:
             print("Min dist: %f" % min_dist)
             start_sumo("cfg/freeway.sumo.cfg", True)
             step = 0
             random.seed(1)
​
         traci.simulationStep()
​
         # 仿真初始化阶段，构造含有 8 辆车的 platoon
         # 设置 GUI 中画面在整个仿真过程中始终聚焦在 v.0.0， 即头车
         # 镜头缩放参数 20000, 这个可以根据具体场景设置，使得镜头既不会拉的太近，也不会拉的太远。
         if step == 0:
             # create vehicles and track the braking vehicle
             topology = add_vehicles(plexe, 8, 1, real_engine)      # 8 vehicles in 1 platoon
             traci.gui.trackVehicle("View #0", BRAKING_VEHICLE)
             traci.gui.setZoom("View #0", 20000)

         # 每隔 10 步车辆之间通信一次，获得其他车辆的位置、速度、加速度等信息
         if step % 10 == 1:
             # simulate vehicle communication every 100 ms
             communicate(plexe, topology)

         # 是否使用 plexe 中改进的更加逼真的引擎模型
         # we probably don't need this
         if real_engine and setter is not None:
             # if we are running with the dashboard, update its values
             tracked_id = traci.gui.getTrackedVehicle("View #0")
             if tracked_id != "":
                 ed = plexe.get_engine_data(tracked_id)
                 vd = plexe.get_vehicle_data(tracked_id)
                 setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

         # 在第 500 步时头车刹车，由于是 车联网 状态，所以其他车辆也会及时得到头车信息，几乎同步刹车，即使跟车距离很小，也不会追尾。这就是 车辆网 的优势。
         if step == 500:
             plexe.set_fixed_acceleration(BRAKING_VEHICLE, True, -6)

         # 记录在整个仿真过程中车辆间隔的最小距离，有需要的话可以随后进行分析
         if step > 1:
             radar = plexe.get_radar_data("v.0.1")
             if radar[RADAR_DISTANCE] < min_dist:
                 min_dist = radar[RADAR_DISTANCE]
​
         step += 1
​
     traci.close()
​
​
if __name__ == "__main__":
 main(True, True)