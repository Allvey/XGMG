#!E:\Pycharm Projects\Waytous
# -*- coding: utf-8 -*-
# @Time : 2021/5/6 11:26
# @Author : Opfer
# @Site :
# @File : sim_py.py
# @Software: PyCharm


import simpy
import datetime
import config
from traffic_flow import traffic_flow_plan
import numpy as np

##############################
#  仿真参数配置，整个仿真的基本单位是分钟
##############################
T, payload = config.get_para("T&p")
dumps, shovels, truck_num = config.get_para("mnt")
com_time, go_time, loading_time, unloading_time = config.get_para("time")
dis = config.get_para("dis")
fuel_speed_empty, fuel_speed_heavy = config.get_para("fuel_speed")
(
    empty_power,
    heavy_power,
    idle_power,
    shovel_idle_power,
    shovel_work_power,
) = config.get_para("energy")
shovel_target_mass, dump_target_mass = config.get_para("target")
empty_speed, heavy_speed = config.get_para("speed")

# 求解初始车流
# traffic_flow_to_unload_area, traffic_flow_to_load_area = traffic_flow_plan(np.array([0, 0, 0, 0]))

global go_to_unload_point_vehical_num  # 用于保存每条卸载道路上经过的车辆个数
global go_to_excavator_vehical_num  # 用于保存每条装载道路上的经过的车辆总个数
global sim_start_time  # 用于保存仿真程序运行的真实起始时间

global walking_go_to_unload_point_vehical_num  # 卸载道路上正在行走的车辆个数
global walking_go_to_excavator_vehical_num  # 装载道路上正在行走的车辆个数

global loading_in_excavator_vehical_num  # 挖机装车数
global unloading_in_unload_point_vehical_num  # 卸点卸车数

global wating_in_excavator_vehical_num  # 挖机排队等待装载矿卡数
global wating_in_unload_point_vehical_num  # 卸点排队等待卸载矿卡数

global dump_available_time  # 卸点最近一次可用时间
global shovel_available_time  # 挖机最近一次可用时间
global truck_available_time  # 矿卡最近一次可用时间

global real_shovel_mass  # 挖机实际装载量
global real_dump_mass  # 卸点实际卸载量

global truck_stage  # 标记矿卡位于哪个阶段（0：空运阶段，1：重运阶段）

global cost  # 记录燃油消耗

global recording  # 数据统计{时间(min), 产量(tonnes), 油耗(liters)}

global traffic_flow_to_unload_area, traffic_flow_to_load_area


def truck_schedule_send_post_start(truck_id, start_area, task_id):
    global shovel_available_time
    global truck_available_time
    global real_shovel_mass
    target = np.argmax(
        1000 * (1 - real_shovel_mass / shovel_target_mass)
        / (np.maximum(shovel_available_time, env.now + com_time[start_area][:]) - env.now))
    # target = np.argmax(traffic_flow_to_load_area[start_area][:]
    #     / (
    #         np.maximum(shovel_available_time, env.now + com_time[start_area][:])
    #         - env.now
    #     )
    # )
    shovel_available_time[target] = max(shovel_available_time[target], truck_available_time[truck_id] + com_time[
        start_area][target])
    truck_available_time[truck_id] = shovel_available_time[target]
    real_shovel_mass[target] += payload
    return target


def truck_schedule_send_post(truck_id, start_area, task_id):
    global truck_available_time
    global shovel_available_time
    global real_shovel_mass
    global dump_available_time
    global real_dump_mass
    global traffic_flow_to_unload_area, traffic_flow_to_load_area

    print("################################# 周期车流规划开始 #################################")
    traffic_flow_to_unload_area, traffic_flow_to_load_area = traffic_flow_plan(real_dump_mass)
    print("################################# 周期车流规划结束 #################################")

    if task_id == 2:
        # target = np.argmax(1000 * (1 - (real_shovel_mass) / shovel_target_mass) /
        #                 (np.maximum(shovel_available_time, env.now + com_time[start_area][:]) + loading_time - env.now))
        target = np.argmax(traffic_flow_to_load_area[start_area][:] /
                           (np.maximum(shovel_available_time,
                                       env.now + com_time[start_area][:]) + loading_time - env.now))
        # print("BASE task 2", target)
    elif task_id == 3:
        # target = np.argmax(1000 * (1 - real_dump_mass / dump_target_mass) /
        #                 (np.maximum(dump_available_time, env.now + go_time[:, start_area]) + unloading_time - env.now))
        target = np.argmax(traffic_flow_to_unload_area[start_area][:] /
                           (np.maximum(dump_available_time,
                                       env.now + go_time[:, start_area]) + unloading_time - env.now))
        # print("BASE task 3", target)
    return target


def walk_process(env, start_area, truck_id, next_q, direction):
    # 模拟矿卡行走的行为

    global go_to_unload_point_vehical_num
    global go_to_excavator_vehical_num

    global walking_go_to_unload_point_vehical_num
    global walking_go_to_excavator_vehical_num

    global wating_in_excavator_vehical_num
    global wating_in_unload_point_vehical_num

    global shovel_available_time
    global dump_available_time
    global truck_available_time

    global real_shovel_mass
    global real_dump_mass

    global truck_stage

    global cost

    # if truck_id > 5:
    #     yield env.timeout(480)

    while True:
        if "go to unload_point" == direction:
            task_id = 3  # 代表从装载点到卸载点
        elif "go to excavator" == direction:
            task_id = 2  # 代表从卸载点到装载点

        # 进行卡调请求，得到目标电铲/卸载点id
        goal_area = truck_schedule_send_post(truck_id, start_area, task_id)

        # 从数据库中获取行走时长,以及装载/卸载时长
        if "go to excavator" == direction:

            # 此时goal_area代表电铲id，start_area代表卸载点id
            # 本次行走时长，除以60用于将秒换算为分钟
            walk_time = com_time[start_area][goal_area]

            # 将该条道路的车辆个数加1
            go_to_excavator_vehical_num[start_area][goal_area] = (
                    go_to_excavator_vehical_num[start_area][goal_area] + 1
            )

            ################################### 关键状态更新 ###################################
            # 更新可用时间
            shovel_available_time[goal_area] = (
                    max(env.now + walk_time, shovel_available_time[goal_area])
                    + loading_time[goal_area]
            )
            truck_available_time[truck_id] = shovel_available_time[goal_area]
            # 产量更新
            real_shovel_mass[goal_area] += payload
            # 修改卡车阶段
            truck_stage[truck_id, :] = [goal_area, -1, 0, start_area]

            cost += (empty_power - idle_power) * walk_time / 60

            ####################################################################################

        elif "go to unload_point" == direction:

            # 此时goal_area代表卸载点id，start_area代表电铲id
            # 本次行走时长，除以60用于将秒换算为分钟
            walk_time = go_time[goal_area][start_area]

            # 将该条道路的车辆个数加1
            go_to_unload_point_vehical_num[start_area][goal_area] = (
                    go_to_unload_point_vehical_num[start_area][goal_area] + 1
            )

            ################################### 关键状态更新 ###################################
            # 修改卡车阶段
            truck_stage[truck_id, :-1] = [start_area, goal_area, 1]
            # 可用时间更新
            dump_available_time[goal_area] = (
                    max(env.now + walk_time, dump_available_time[goal_area])
                    + unloading_time[goal_area]
            )
            truck_available_time[truck_id] = dump_available_time[goal_area]

            cost += (heavy_power - idle_power) * walk_time / 60

            ####################################################################################

        print(
            f"{round(env.now, 2)} - truck_id: {truck_id} - {direction}_{goal_area} - start moving "
        )

        # 行驶开始，统计在路上行走的车辆个数
        if "go to excavator" == direction:
            # 将该条道路正在行驶的车辆个数加1
            walking_go_to_excavator_vehical_num[start_area][goal_area] = (
                    walking_go_to_excavator_vehical_num[start_area][goal_area] + 1
            )
        elif "go to unload_point" == direction:
            walking_go_to_unload_point_vehical_num[start_area][goal_area] = (
                    walking_go_to_unload_point_vehical_num[start_area][goal_area] + 1
            )

        # 阻塞行走时间
        yield env.timeout(float(walk_time))  # 行走时间,单位为分钟

        # 行驶结束，统计在路上行走的车辆个数
        if "go to excavator" == direction:
            # 将该条道路正在行驶的车辆个数减1
            walking_go_to_excavator_vehical_num[start_area][goal_area] = (
                    walking_go_to_excavator_vehical_num[start_area][goal_area] - 1
            )
            # 行走结束，将等待装载的车辆个数加1
            wating_in_excavator_vehical_num[goal_area] = (
                    wating_in_excavator_vehical_num[goal_area] + 1
            )
        elif "go to unload_point" == direction:
            walking_go_to_unload_point_vehical_num[start_area][goal_area] = (
                    walking_go_to_unload_point_vehical_num[start_area][goal_area] - 1
            )
            # 行走结束，将等待卸载的车辆个数加1
            wating_in_unload_point_vehical_num[goal_area] = (
                    wating_in_unload_point_vehical_num[goal_area] + 1
            )
            # 产量更新
            real_dump_mass[goal_area] += payload

        next_q[goal_area].put(truck_id)  # 将到来的truck放到目标队列中
        print(
            f"{round(env.now, 2)} - truck_id: {truck_id} - {direction}_{goal_area} - end moving"
        )

        # env.exit()  # 该函数的作用等同于return,直接退出该函数
        return


def excavator_func(env: simpy.Environment, e_q: simpy.Store, u_q, excavator_id):
    # 模拟一个电铲, 一个电铲同时只能处理一台矿卡
    truck_source = simpy.Resource(env, capacity=1)

    global real_dump_mass
    global real_shovel_mass

    def process(truck_id):
        # 模拟电铲一次工作的进程
        with truck_source.request() as req:
            yield req
            print(
                f"{round(env.now, 2)} - truck_id: {truck_id} - excavator: {excavator_id} - Begin Loading"
            )

            # 开始装载，将等待装载的车辆个数减1
            global wating_in_excavator_vehical_num
            wating_in_excavator_vehical_num[excavator_id] = (
                    wating_in_excavator_vehical_num[excavator_id] - 1
            )

            # 开始装载，将装载车辆个数加1
            global loading_in_excavator_vehical_num
            loading_in_excavator_vehical_num[excavator_id] = (
                    loading_in_excavator_vehical_num[excavator_id] + 1
            )

            global cost
            cost += (
                    (shovel_work_power[excavator_id] - shovel_idle_power[excavator_id])
                    * loading_time[excavator_id]
                    / 60
            )

            # 电铲平均工作装载时间
            # 除以60用于将秒换算为分钟
            load_time = loading_time[excavator_id]
            yield env.timeout(float(load_time))  # 进行装载操作

            # 装载结束，将装载车辆个数减1
            loading_in_excavator_vehical_num[excavator_id] = (
                    loading_in_excavator_vehical_num[excavator_id] - 1
            )

            # 矿卡从电铲处行走到卸载点
            env.process(
                walk_process(env, excavator_id, truck_id, u_q, "go to unload_point")
            )

    while True:
        truck_id = yield e_q.get()
        env.process(process(truck_id))


def unloadpoint_func(env: simpy.Environment, u_q: simpy.Store, e_q, unload_point_id):
    # 模拟一个卸载点, 一个卸载点同时只能处理一台矿卡
    truck_source = simpy.Resource(env, capacity=1)

    global real_dump_mass
    global real_shovel_mass

    def process(truck_id):
        # 模拟卸载点一次工作的进程"""
        with truck_source.request() as req:
            yield req
            print(
                f"{round(env.now, 2)} - truck_id: {truck_id} - UnloadPoint: {unload_point_id} - Begin Unloading"
            )

            # 开始卸载，将等待卸载的车辆个数减1
            global wating_in_unload_point_vehical_num
            wating_in_unload_point_vehical_num[unload_point_id] = (
                    wating_in_unload_point_vehical_num[unload_point_id] - 1
            )

            # 开始卸载，将正在的卸载车辆个数加1
            global unloading_in_unload_point_vehical_num
            unloading_in_unload_point_vehical_num[unload_point_id] = (
                    unloading_in_unload_point_vehical_num[unload_point_id] + 1
            )

            # 卸载点平均工作卸载时间
            # 除以60用于将秒换算为分钟
            unload_time = unloading_time[unload_point_id]
            yield env.timeout(float(unload_time))  # 进行卸载操作

            # 卸载结束后，需要将实际卸载量加上矿卡的运输量
            real_comp_workload[unload_point_id] += payload

            # 卸载结束，将卸载车辆个数减1
            unloading_in_unload_point_vehical_num[unload_point_id] = (
                    unloading_in_unload_point_vehical_num[unload_point_id] - 1
            )

            # 矿卡从卸载点处行走到电铲
            env.process(
                walk_process(env, unload_point_id, truck_id, e_q, "go to excavator")
            )

    while True:
        truck_id = yield u_q.get()
        env.process(process(truck_id))


# 在停车场按照固定时间生成一定数量的矿卡
def generate_truck_in_parking_lot(env, e_q, u_q):
    global shovel_available_time
    global dump_available_time
    global truck_available_time
    global real_shovel_mass
    global truck_stage

    def process(truck_id, walk_time, goal_area, e_q):
        ################################### 关键状态更新 ###################################
        # 记录初始阶段矿卡调度，用于schedule的构造
        truck_stage[truck_id, :] = np.array([goal_area, -1, 0, 0])
        # 更新电铲，矿卡可用时间
        shovel_available_time[goal_area] = (
                max(env.now + walk_time, shovel_available_time[goal_area])
                + loading_time[goal_area]
        )
        truck_available_time[truck_id] = shovel_available_time[goal_area]

        # # 每完成一次调度都需要更新设备参数
        # dispatcher.update(
        #     truck_available_time,
        #     shovel_available_time,
        #     real_shovel_mass,
        #     1,
        #     truck_stage,
        # )

        go_to_excavator_vehical_num[0][goal_area] = (
                go_to_excavator_vehical_num[0][goal_area] + 1
        )

        # 阻塞行走时间
        yield env.timeout(float(walk_time))  # 行走时间,单位为分钟

        # 产量更新
        real_shovel_mass[goal_area] += payload

        e_q[goal_area].put(truck_id)  # 将到来的truck放到电铲的队列中

        # 行走结束，将等待装载的车辆个数加1
        wating_in_excavator_vehical_num[goal_area] = (
                wating_in_excavator_vehical_num[goal_area] + 1
        )

        print(
            f"{round(env.now, 2)} - truck_id: {truck_id} - From Parking Lot to WorkArea:{goal_area} end moving"
        )

    for i in range(truck_num):
        # 模拟矿卡随机请求调度
        t = 0.1  # 固定停0.1*60=6秒
        yield env.timeout(round(t, 1))

        task_id = 1  # task_id等于1，说明是从停车场到装载点
        target = truck_schedule_send_post_start(i, 0, task_id)  # 得到电铲id

        # 本次行走时长
        walk_time = com_time[0][target]

        print(
            f"{round(env.now, 2)} - truck_id: {i} - From Parking Lot to WorkArea:{target} start moving "
        )

        env.process(process(i, walk_time, target, e_q))


# 数据统计函数，每隔一段时间进行数据记录
def mass_control(env: simpy.Environment):
    global cost
    global real_dump_mass
    while True:
        yield env.timeout(1)

        cost += (np.sum(shovel_idle_power) + truck_num * idle_power) / 60

        print(int(env.now))
        if int(env.now) % 10 == 0:
            recording.append([env.now, np.sum(real_comp_workload), cost])

    return


if __name__ == "__main__":

    sim_start_time = datetime.datetime.now().replace(microsecond=0)

    # 实例环境
    env = simpy.Environment()

    # 获取装载点和卸载点的个数
    num_of_load_area = shovels
    num_of_unload_area = dumps

    e_q = []
    for _ in range(num_of_load_area):
        e_q.append(simpy.Store(env))

    u_q = []
    for _ in range(num_of_unload_area):
        u_q.append(simpy.Store(env))

    # 保存每条道路的车辆个数
    go_to_unload_point_vehical_num = np.zeros((num_of_load_area, num_of_unload_area))
    go_to_excavator_vehical_num = np.zeros((num_of_unload_area, num_of_load_area))

    real_comp_workload = np.zeros(dumps)

    # 统计在路上行驶的车辆个数
    walking_go_to_unload_point_vehical_num = np.zeros(
        (num_of_load_area, num_of_unload_area)
    )
    walking_go_to_excavator_vehical_num = np.zeros(
        (num_of_unload_area, num_of_load_area)
    )

    # 统计正在装载或者卸载的车辆个数
    loading_in_excavator_vehical_num = np.zeros(num_of_load_area)
    unloading_in_unload_point_vehical_num = np.zeros(num_of_unload_area)

    # 统计正在排队的车辆个数
    wating_in_excavator_vehical_num = np.zeros(num_of_load_area)
    wating_in_unload_point_vehical_num = np.zeros(num_of_unload_area)

    # 求解初始车流
    traffic_flow_to_load_area, traffic_flow_to_unload_area = traffic_flow_plan(np.array([0, 0, 0, 0]))

    # 初始化设备可用时间
    dump_available_time = np.zeros(num_of_unload_area)
    shovel_available_time = np.zeros(num_of_load_area)
    truck_available_time = np.zeros(truck_num)

    # 初始化实时产量
    real_shovel_mass = np.zeros(num_of_load_area)
    real_dump_mass = np.zeros(num_of_unload_area)

    # 初始化矿卡状态
    truck_stage = np.zeros((truck_num, 4), dtype=np.int)

    # 初始化油耗
    cost = 0

    # 初始化统计表
    recording = [[0, 0, 0]]

    # 启动挖机及卸点进程
    for i in range(num_of_load_area):
        env.process(excavator_func(env, e_q[i], u_q, excavator_id=i))

    for i in range(num_of_unload_area):
        env.process(unloadpoint_func(env, u_q[i], e_q, unload_point_id=i))

    # 从停车位开始向电铲派车
    env.process(generate_truck_in_parking_lot(env, e_q, u_q))

    # 模拟产量满足条件
    proc = env.process(mass_control(env))

    # 开始仿真运行
    env.run(until=482)

    # 结果统计
    print("配对的车辆总个数：")
    print("卸载道路上配对车辆总个数：")
    print(go_to_unload_point_vehical_num)
    print("装载道路上配对车辆总个数：")
    print(go_to_excavator_vehical_num)

    print("正在行驶的车辆个数：")
    print("卸载道路上正在行驶车辆总个数：")
    print(walking_go_to_unload_point_vehical_num)
    print("装载道路上正在行驶车辆总个数：")
    print(walking_go_to_excavator_vehical_num)

    print("正在装载或者卸载的车辆个数：")
    print("电铲正在装载的车辆总个数：")
    print(loading_in_excavator_vehical_num)
    print("卸载点正在卸载的车辆总个数：")
    print(unloading_in_unload_point_vehical_num)

    print("正在电铲处排队的车辆个数：")
    print(wating_in_excavator_vehical_num)

    print("正在卸载点处排队的车辆个数：")
    print(wating_in_unload_point_vehical_num)

    print("各卸点最终产量")
    print(real_dump_mass)

    for x in recording:
        print("时间: ", x[0], "min, 产量: ", x[1], "tonnes, 油耗: ", x[2], "liters")
