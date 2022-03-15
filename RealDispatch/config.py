import numpy as np

class global_var:
    # 班次时间(min)
    T = 480
    # 矿卡载重(吨)payload = 220
    payload = 220
    # 矿卡数量
    truck_num = 10
    # 电铲数量
    n = 3
    # 卸点数量
    m = 4
    # 矿卡平均行驶速度
    empty_speed = 25
    heavy_speed = 22

    # 各路线距离
    dis = [[4.01,   4.45,     5.32],
            [5.45,  3.65,     4.75],
            [4.481, 4.481,    5.481],
            [6.481, 3.481,    7.481]]

    # 各路线空载行驶时间（h）
    com_time = np.array(dis) / empty_speed
    go_time = np.array(dis) / heavy_speed

    # 电铲装载速度&卸点卸载速度（吨/时）
    load_capacity = np.array([1600, 2000, 2000]);
    unload_capacity = np.array([2375, 2375, 2375, 2375]);

    # 电铲装载时间&卸点卸载时间（吨/时）
    loading_time = np.round(60 * (payload) / load_capacity, 3)
    unloading_time = np.round(60 * (payload) / unload_capacity, 3)

    # 装载及卸载时间维度扩展
    loading_time_dims = np.expand_dims(loading_time, 0).repeat(m, axis=0)
    unloading_time_dims = np.expand_dims(unloading_time, 1).repeat(n, axis=1)

    # 矿卡空载行驶能耗
    empty_power = 85;
    # 矿卡重载行驶能耗
    heavy_power = 150;
    # 矿卡空转能耗
    idle_power = 40;
    # 电铲闲置能耗
    shovel_idle_power = [6.6, 9.5, 9.5];
    shovel_work_power = [117, 130, 130];

    # shovel_work_power = np.divide(shovel_work_power, 60)

    # 速度油耗关系(速度:km/h, 节油系数:%)
    fuel_speed_empty = [[22.5, 0.2], [23, 0.18], [23.5, 0.17], [24, 0.15], [24.5, 0.11], [25, 0.0]]
    fuel_speed_heavy = [[19.5, 0.2], [20, 0.18], [20.5, 0.17], [21, 0.15], [21.5, 0.11], [22, 0.0]]

    # 各挖机/卸点目标产量
    dump_target_mass = np.array([15000, 15000, 15000, 15000])
    shovel_target_mass = np.array([15000, 15000, 15000])

# 对于每个全局变量，都需要定义get_value和set_value接口
def set_para(name):
    global_var.name = name
def get_para(name):
    if name == 'T&p':
        return global_var.T, global_var.payload
    elif name == 'mnt':
        return global_var.m, global_var.n, global_var.truck_num
    elif name == 'time':
        return global_var.com_time, global_var.go_time, global_var.loading_time, global_var.unloading_time
    elif name == 'energy':
        return global_var.empty_power, global_var.heavy_power, global_var.idle_power, \
               global_var.shovel_idle_power, global_var.shovel_work_power
    elif name == 'dis':
        return global_var.dis
    elif name == 'fuel_speed':
        return global_var.fuel_speed_empty, global_var.fuel_speed_heavy
    elif name == 'road':
        return global_var.road_ava
    elif name == 'capacity':
        return global_var.load_capacity, global_var.unload_capacity
    elif name == 'target':
        return global_var.shovel_target_mass, global_var.dump_target_mass
    elif name == 'speed':
        return global_var.empty_speed, global_var.heavy_speed
