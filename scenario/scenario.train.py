#!/usr/bin/env python3

import lgsvl
import copy
import time
import csv
import argparse
from lgsvl.geometry import Vector

def add_ego_car(vehicle_name):
    spawns = sim.get_spawn()
    state = lgsvl.AgentState()
    state.transform = spawns[1]
    state.transform.position = Vector(-15, 0, -6)

    agent = sim.add_agent(vehicle_name, lgsvl.AgentType.EGO, state)

    agent.connect_bridge(args.bridge, 9090)
    print('Waiting for connection...')
    while not agent.bridge_connected:
        time.sleep(1)
    print('Bridge connected:', agent.bridge_connected)

    return agent

def get_ground_point(x, z):
    raycast_pos = Vector(x, 100, z)
    # project the point to ground surface
    layer_mask = 0
    layer_mask |= 1 << 0  # 0 is the layer for the road (default)
    hit = sim.raycast(raycast_pos, lgsvl.Vector(0, -1, 0), layer_mask)
    return copy.deepcopy(hit.point)

def get_angle(spawn_id, offset):
    rotation = spawns[spawn_id].rotation
    angle = copy.deepcopy(rotation)
    angle.y += offset
    return angle

def parse_csv(file_path):
    points = []
    with open(file_path, newline='') as file: 
        csv_reader = csv.reader(file, delimiter=',')
        for row in csv_reader:
            if len(row) < 4:
                continue
            if row[0] == 'x':
                continue
            point = {
                'x': float(row[0]),
                'z': float(row[1]),
                'yaw': float(row[2]),
                'speed': float(row[3])
            }
            points.append(point)
    return points

def on_collision(agent1, agent2, contact):
    cp = contact - agent1.state.transform.position
    f = lgsvl.utils.transform_to_forward(agent1.state.transform)
    has_hit_to_front = cp.x * f.x + cp.z * f.z > 0; 
    if has_hit_to_front:
        agent1.follow_closest_lane(True, 0)

def add_racing_car(csv_path):
    waypoints = []
    with open(csv_path, newline='') as file: 
        csv_reader = csv.reader(file, delimiter=',')
        for row in csv_reader:
            if len(row) < 7:
                continue
            if row[0] == 'x':
                continue
            waypoints.append( lgsvl.DriveWaypoint(
            lgsvl.Vector(row[0], row[1], row[2]),
            row[6], Vector(row[3], row[4], row[5])) )


    state = lgsvl.AgentState()
    state.transform.position = waypoints[0].position
    state.transform.rotation = waypoints[0].angle
    npc = sim.add_agent('DallaralL15', lgsvl.AgentType.NPC, state)

    npc.follow(waypoints, loop=False)
    npc.on_collision(on_collision)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host',help='simulator execute pc\'s ip address / --host 192.168.0.1',default='127.0.0.1')
    parser.add_argument('--bridge',help='bridge execute pc\'s ip address / --bridge 192.168.0.1',default='127.0.0.1')
    parser.add_argument('--vehicle_id',help='vehicle id\'s ip address / --bridge 192.168.0.1',default='35a48eff-4a4c-46f3-9742-f79b5c8f848f')
    args = parser.parse_args()

    sim = lgsvl.Simulator(args.host, 8181)

    scene_name = 'IndianapolisMotorSpeedway'
    vehicle_id = args.vehicle_id
    if sim.current_scene == scene_name:
        sim.reset()
    else:
        sim.load(scene_name)

    scenario_id = 'train'

    add_ego_car(vehicle_id)
    add_racing_car('data/npc1.csv')
    add_racing_car('data/npc2.csv')
    add_racing_car('data/npc3.csv')
    add_racing_car('data/npc4.csv')
    add_racing_car('data/npc5.csv')

    sim.run(time_limit = 300.0)
    sim.remote.command('simulator/push_message', {'message': 'publish_timersensor'})
