import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla
import random
from queue import Queue, Empty
import numpy as np
import copy
from display_manager import *  # 用pygame实现传感器数据实时可视化的类
import logging


set_synchronous = True # 同步模式
vehicles_id_list = []


# 收到激光数据的返回函数：保存点云
def sensor_callback(sensor_data, frame_num):
    points = np.frombuffer(sensor_data.raw_data, dtype=np.dtype('f4'))
    points = copy.deepcopy(points)
    lidar = np.reshape(points, (int(points.shape[0] / 4), 4))
    np.save("save_path/lidar/"+str(frame_num)+".npy", lidar)
    


# 主仿真函数
def run_simulation(client):
    vehicle_list = []
    try:
        world = client.get_world()
        original_settings = world.get_settings()
        weather = carla.WeatherParameters(cloudiness=10.0,
                                        precipitation=10.0,
                                        fog_density=10.0)
        world.set_weather(weather)
        
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # 拿到这个世界所有物体的蓝图
        blueprint_library = world.get_blueprint_library()
        
        # 获取可用的车辆生成点位置，手动加上坐标和rqy偏置，模拟路侧激光雷达安装位置
        spawn_points = world.get_map().get_spawn_points()[0]
        lidar_transform = carla.Transform(spawn_points.location+ carla.Location(x=4, y=-3, z=4.5), carla.Rotation(pitch=-15, yaw=-45))

        # 设置我们的观察角度：与激光雷达视角一致
        spectator = world.get_spectator()
        spectator.set_transform(lidar_transform)
        
        # 用DisplayManager类添加传感器，并实时可视化传感器数据
        display_manager = DisplayManager(grid_size=[1,3], window_size=[1800, 1000])
        SensorManager(world, display_manager, 'LiDAR', lidar_transform, 
                       {'channels' : '64', 'range' : '100',  'points_per_second': '250000', 'rotation_frequency': '20'}, display_pos=[0, 0])
        SensorManager(world, display_manager, 'LiDAR', lidar_transform, 
                    {'channels' : '128', 'range' : '100',  'points_per_second': '500000', 'rotation_frequency': '20'}, display_pos=[0, 1])
        SensorManager(world, display_manager, 'SemanticLiDAR', lidar_transform, 
                       {'channels' : '128', 'range' : '100',  'points_per_second': '500000', 'rotation_frequency': '20'}, display_pos=[0, 2])

        # -------------------生成交通npc--------------------------
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        blueprints_vehicle = blueprint_library.filter("vehicle.*")
        blueprints = sorted(blueprints_vehicle, key=lambda bp: bp.id)
        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)
        
        # --------------
        # Spawn vehicles
        # --------------
        
        batch = []
        number_of_vehicles = 100
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, tm.get_port())))
        for response in client.apply_batch_sync(batch, set_synchronous):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_id_list.append(response.actor_id)

        # ---------------------主仿真循环---------------------------
        while True:
            world.tick()
            
             # 呈现接收到的数据
            display_manager.render()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    call_exit = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        call_exit = True
                        break

            
    finally:
        # sensor_lidar.destroy()
        
        # 删除本次仿真生成的所有actor
        if display_manager:
            display_manager.destroy()

        world.apply_settings(original_settings)
        print("All destroyed.")
        


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(100.0)
        run_simulation(client)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
if __name__ == "__main__":
    
    main()