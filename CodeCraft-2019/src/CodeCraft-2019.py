# author: RainMoun  2019华为软件精英挑战赛初赛code
# 声明：借鉴了他人的判题器
import sys
import collections
import numpy as np
import os
import logging

np.random.seed(951105)

TIME = [0]
CAR_STATE = [0, 0, 0]
CAR_NAMESPACE, ROAD_NAMESPACE, CROSS_NAMESPACE = [], [], []
CROSS_DICT, CAR_DICT, ROAD_DICT = {}, {}, {}
# CROSS_INDEX = [0]
ROAD_BURDEN_INDEX = 0.45
METHOD_2_BATCH_NUM = [50, 45, 40, 35, 30, 25, 20]
# LAST_NUM = int(METHOD_2_BATCH_NUM * 0.4)  # 0.4
CHANNEL_INDEX = [0, 1.5, 1.1, 1, 0.55, 0.5]
CONNECTED_NUM = [0, 0, 1, 1, 1]
CHANGE_ROAD_TIME_INDEX = 3
# EDGE_REWARD = 0.95
SPEED_LIST = []
SPEED_INDEX = [0]
road_map = None
CAR_LEFT_DIVIDED_BY_SPEED = {}
CAR_LEFT_DIVIDED_BY_SPEED = collections.OrderedDict()
CAR_BEGIN_TIME_DICT = {}
NEXT_RELEASE_CAR_TIME = [1]
INTERVAL_TIME = 7
TIME_GO = [1]


class CAR(object):
    def __init__(self, id_, from_, to_, speed, plan_time, is_prefer, is_preset):
        # **** statistic parameters ****#
        self.id_, self.from_, self.to_, self.speed, self.plan_time = id_, from_, to_, speed, plan_time
        self.is_prefer, self.is_preset = is_prefer, is_preset
        self.car_color = [int(value) for value in np.random.random_integers(0, 255, [3])]
        # **** dynamic parameters ****#
        self.state, self.x, self.y = 0, 0, 0
        self.presentRoad, self.nextCrossId = None, self.from_
        self.deltaX, self.deltaY = 0, 0
        self.wait = False
        self.route, self.routeIndex = None, None

    #
    # simulate initialization
    #
    def simulate_init(self, plan_time, route):
        self.plan_time, self.route, self.routeIndex = plan_time, route, 0

    # def find_better_way(self, road_ids):
    #     road_ids = [i for i in road_ids if i != -1]
    #     if self.routeIndex == len(self.route):
    #         return
    #     next_road_plan = self.route[self.routeIndex]
    #     if ROAD_DICT[next_road_plan].from_ == self.nextCrossId:
    #         is_forward = True
    #     else:
    #         is_forward = False
    #     road_burden_plan = ROAD_DICT[next_road_plan].get_car_num_index_on_road(is_forward)
    #     if road_burden_plan < ROAD_BURDEN_INDEX:
    #         return
    #     else:
    #         road_ids.remove(next_road_plan)
    #         if self.presentRoad != -1:
    #             road_ids.remove(self.presentRoad)
    #         road_ids = [i for i in road_ids if
    #                     not (ROAD_DICT[i].is_duplex == 0 and ROAD_DICT[i].from_ != self.nextCrossId)]
    #         if len(road_ids) == 0:
    #             return 0
    #         road_ids = [[i] for i in road_ids]
    #         run_time = 0
    #         while road_ids:
    #             route_valid = []
    #             route_valid_index = []
    #             temp_road_ids = []
    #             next_cross_id_lst = []
    #             for i in road_ids:
    #                 if len(i) >= 2:
    #                     now_road_cross_from = ROAD_DICT[i[-1]].from_
    #                     if now_road_cross_from in [ROAD_DICT[i[-2]].from_, ROAD_DICT[i[-2]].to_]:
    #                         next_cross_id_lst.append(ROAD_DICT[i[-1]].to_)
    #                     else:
    #                         next_cross_id_lst.append(ROAD_DICT[i[-1]].from_)
    #             for i in range(len(road_ids)):
    #                 now_road = road_ids[i][-1]
    #                 if run_time != 0:
    #                     next_cross_id = next_cross_id_lst[i]
    #                 else:
    #                     next_cross_id = self.nextCrossId
    #                 if ROAD_DICT[now_road].from_ == next_cross_id:
    #                     is_forward = True
    #                 else:
    #                     is_forward = False
    #                 if run_time == 0:
    #                     road_burden = ROAD_DICT[now_road].get_car_num_index_on_road(is_forward)
    #                     if road_burden > road_burden_plan:
    #                         continue
    #                 if run_time == 0:
    #                     next_cross_id = ROAD_DICT[now_road].to_ if is_forward else ROAD_DICT[now_road].from_
    #                 new_route = road_map.route_plan[next_cross_id][self.to_]
    #                 new_route = road_ids[i] + new_route
    #                 if next_road_plan in new_route or len(list(set(new_route))) != len(new_route):
    #                     now_next_cross_id = ROAD_DICT[now_road].from_ if ROAD_DICT[now_road].from_ != self.nextCrossId else ROAD_DICT[now_road].to_
    #                     other_road = [k for k in CROSS_DICT[now_next_cross_id].roadIds if k != -1]
    #                     other_road.remove(now_road)
    #                     for j in other_road:
    #                         if ROAD_DICT[j].is_duplex == 1:
    #                             temp_road_ids.append(road_ids[i] + [j])
    #                         else:
    #                             if ROAD_DICT[j].from_ in [ROAD_DICT[road_ids[i][-1]].from_, ROAD_DICT[road_ids[i][-1]].to_]:
    #                                 temp_road_ids.append(road_ids[i] + [j])
    #                     continue
    #                 route_valid.append(self.route[0: self.routeIndex] + new_route)
    #                 route_valid_index.append(road_map.route_length[next_cross_id][self.to_])
    #             run_time += 1
    #             if route_valid:
    #                 min_index = route_valid[route_valid_index.index(min(route_valid_index))]
    #                 self.route = min_index
    #                 return 1
    #             if run_time == CHANGE_ROAD_TIME_INDEX:
    #                 return 0
    #             road_ids = temp_road_ids[:]

    def find_better_way(self, road_ids):  # 即将经过的路口的其他两条路
        road_ids = [i for i in road_ids if i != -1]
        if self.routeIndex == len(self.route):
            return
        next_road_plan = self.route[self.routeIndex]
        if ROAD_DICT[next_road_plan].from_ == self.nextCrossId:
            is_forward = True
        else:
            is_forward = False
        road_burden_plan = ROAD_DICT[next_road_plan].get_car_num_index_on_road(is_forward)
        if road_burden_plan < ROAD_BURDEN_INDEX:
            return
        else:  # 开始换路
            road_ids.remove(next_road_plan)
            road_ids.remove(self.presentRoad)
            road_ids = [i for i in road_ids if
                        not (ROAD_DICT[i].is_duplex == 0 and ROAD_DICT[i].from_ != self.nextCrossId)]
            if len(road_ids) == 0:  # 仅有一条道路可走
                return
            if len(road_ids) == 1:  # 除计划道路仅有一条路
                if ROAD_DICT[road_ids[0]].from_ == self.nextCrossId:
                    is_forward = True
                else:
                    is_forward = False
                other_road_burden = ROAD_DICT[road_ids[0]].get_car_num_index_on_road(is_forward)
                if road_burden_plan > other_road_burden:  # 如果当前道路拥挤程度低于计划道路
                    next_road = ROAD_DICT[road_ids[0]]
                    if is_forward:
                        next_cross_id = next_road.to_
                    else:
                        next_cross_id = next_road.from_
                    new_route = road_map.route_plan[next_cross_id][self.to_]
                    new_route = [road_ids[0]] + new_route
                    if next_road_plan in new_route or len(list(set(new_route))) != len(new_route):
                        return
                    else:
                        self.route = self.route[0: self.routeIndex] + new_route
            elif len(road_ids) == 2:  # 存在两条可供选择的道路
                is_forward_lst = []
                for i in road_ids:
                    if ROAD_DICT[i].from_ == self.nextCrossId:
                        is_forward_lst.append(True)
                    else:
                        is_forward_lst.append(False)
                road_burden_lst = []
                for i in range(len(road_ids)):
                    road_burden_lst.append(ROAD_DICT[road_ids[i]].get_car_num_index_on_road(is_forward_lst[i]))
                first_select_road = 0 if road_burden_lst[0] < road_burden_lst[1] else 1
                if road_burden_lst[first_select_road] > road_burden_plan:
                    return
                next_road = ROAD_DICT[road_ids[first_select_road]]
                if is_forward_lst[first_select_road]:
                    next_cross_id = next_road.to_
                else:
                    next_cross_id = next_road.from_
                new_route = road_map.route_plan[next_cross_id][self.to_]
                new_route = [road_ids[first_select_road]] + new_route
                if next_road_plan in new_route or len(list(set(new_route))) != len(new_route):  # 当前选择的转向路会转回原计划路径
                    second_select_road = 0 if first_select_road == 1 else 0
                    if road_burden_lst[second_select_road] > road_burden_plan:
                        return
                    next_road = ROAD_DICT[road_ids[second_select_road]]
                    if is_forward_lst[second_select_road]:
                        next_cross_id = next_road.to_
                    else:
                        next_cross_id = next_road.from_
                    new_route = road_map.route_plan[next_cross_id][self.to_]
                    new_route = [road_ids[second_select_road]] + new_route
                    if next_road_plan in new_route or len(list(set(new_route))) != len(new_route):  # 当前选择的转向路会转回原计划路径
                        return
                    else:
                        self.route = self.route[0: self.routeIndex] + new_route
                else:
                    self.route = self.route[0: self.routeIndex] + new_route

    def start_driving(self):
        for road_id in CROSS_DICT[self.from_].receiver:
            ROAD_DICT[road_id].set_bucket(CROSS_DICT[self.from_].id_)
        road_id = CAR_DICT[self.id_].__nextRoad__()
        road = ROAD_DICT[road_id]
        if road_id not in CROSS_DICT[self.from_].receiver:
            print("Car(%d).Road(%d) not in cross(%d).function:class.outOfCarport" %
                  (self.id_, road_id, CROSS_DICT[self.from_].id_))
        act = road.receive_car(self.id_)
        if act != 0:
            other_road_ids = CROSS_DICT[self.from_].receiver[:]
            other_road_ids.remove(road_id)
            new_plan_route_lst = []
            new_plan_route_value_lst = []
            for i in other_road_ids:
                next_cross_id = ROAD_DICT[i].from_ if ROAD_DICT[i].from_ != self.from_ else ROAD_DICT[i].to_
                new_plan_route = road_map.route_plan[next_cross_id][self.to_][:]
                if i in new_plan_route:
                    continue
                new_plan_route = [i] + new_plan_route
                new_plan_value = road_map.route_length[next_cross_id][self.to_]
                new_plan_route_value_lst.append(new_plan_value)
                new_plan_route_value_lst = sorted(new_plan_route_value_lst)
                new_plan_route_lst.insert(new_plan_route_value_lst.index(new_plan_value), new_plan_route)
            car_original_route = self.route[:]
            for i in new_plan_route_lst:
                self.route = i[:]
                now_road = ROAD_DICT[self.route[0]]
                act = now_road.receive_car(self.id_)
                if act != 0:
                    continue
                CAR_DICT[self.id_].plan_time = TIME[0]
                CROSS_DICT[self.from_].carportCarNum -= 1
                CAR_STATE[0] -= 1
                CAR_STATE[1] += 1
                return 1
            self.route = car_original_route[:]
            return 0
        CAR_DICT[self.id_].plan_time = TIME[0]
        CROSS_DICT[self.from_].carportCarNum -= 1
        CAR_STATE[0] -= 1
        CAR_STATE[1] += 1
        return 1

    #
    # dynamic param update
    #
    def update_dynamic(self, state, x=None, y=None, present_road=None, road_speed=None, next_cross_id=None):  # 动态调度
        # car not in carport of car is ready to go
        if self.state != 0 or present_road is not None:
            self.state = state
        if present_road is not None and self.state != 0 and self.routeIndex < self.route.__len__():
            self.routeIndex += 1
        self.x = x if x is not None else self.x
        self.y = y if y is not None else self.y
        self.presentRoad = present_road if present_road is not None else self.presentRoad
        if next_cross_id is not None:
            self.nextCrossId = next_cross_id
            to_x, to_y = CROSS_DICT[self.to_].__loc__()
            next_cross_x, next_cross_y = CROSS_DICT[next_cross_id].__loc__()
            self.deltaX, self.deltaY = to_x - next_cross_x, to_y - next_cross_y

    #
    # show some important info
    #
    def __v__(self):
        return min(self.speed_, ROAD_DICT[self.presentRoad].__speed__())

    def __distance__(self):
        return abs(self.deltaX)+abs(self.deltaY)

    def __nextRoad__(self):
        try:
            return self.route[self.routeIndex]
        except:
            return -1


class ROAD(object):
    def __init__(self, id_, length, speed, channel, from_, to_, is_duplex):
        # **** statistic parameters ****#
        self.id_, self.length, self.speed, self.channel, self.from_, self.to_, self.is_duplex = \
            id_, length, speed, channel, from_, to_, is_duplex
        self.car_capacity = self.channel * self.length
        self.road_value = self.length * CHANNEL_INDEX[self.channel]
        self.road_value_forward = self.road_value
        if self.is_duplex == 1:
            self.road_value_backward = self.road_value
        # **** dynamic parameters ****#
        # absolute bucket
        self.forwardBucket = {i: [None for _ in range(self.channel)] for i in range(self.length)}  # 前向车道
        self.backwardBucket = {i: [None for _ in range(self.channel)] for i in
                               range(self.length)} if self.is_duplex else None
        self.fx, self.fy, self.bx, self.by, self.forwardNum, self.backwardNum = [0], [0], [0], [0], [0], [0]
        self.forwardDone, self.backwardDone = [False], [False]
        # relative bucket
        self.provideBucket, self.receiveBucket = None, None
        self.px, self.py, self.provideNum, self.receiveNum = None, None, None, None
        self.provideDone = None
    #
    # determine relative bucket
    #

    def get_car_num_index_on_road(self, is_forward):
        if is_forward:
            car_num = 0
            for i in self.forwardBucket:
                for j in self.forwardBucket[i]:
                    if j:
                        car_num += 1
            return car_num / self.car_capacity
        else:
            car_num = 0
            for i in self.backwardBucket:
                for j in self.forwardBucket[i]:
                    if j:
                        car_num += 1
            return car_num / self.car_capacity

    def choose_absolute_bucket(self, cross_id, pr):
        if cross_id == self.from_ and pr == 'provide':
            return 'backward'
        elif cross_id == self.from_ and pr == 'receive':
            return 'forward'
        elif cross_id == self.to_ and pr == 'provide':
            return 'forward'
        elif cross_id == self.to_ and pr == 'receive':
            return 'backward'
        else:
            print("Keywords mistake in CAR.chooseAbsoluteBucket()")

    def set_bucket(self, cross_id):
        bucket = self.choose_absolute_bucket(cross_id, 'provide')
        if bucket == 'forward':
            self.provideBucket, self.px, self.py, self.provideDone, self.provideNum = \
                [self.forwardBucket, self.fx, self.fy, self.forwardDone, self.forwardNum]
            if self.is_duplex:
                self.receiveBucket, self.receiveNum = \
                    self.backwardBucket, self.backwardNum
            else:
                self.receiveBucket, self.receiveNum = None, None
        else:
            self.receiveBucket, self.receiveNum = \
                self.forwardBucket, self.forwardNum
            if self.is_duplex:
                self.provideBucket, self.px, self.py, self.provideDone, self.provideNum = \
                    self.backwardBucket, self.bx, self.by, self.backwardDone, self.backwardNum
            else:
                self.provideBucket, self.px, self.py, self.provideDone, self.provideNum = \
                    None, None, None, None, None

    #
    # stepInitial
    #
    def step_init(self):
        # dynamic param initialization
        self.fx, self.fy, self.bx, self.by = [0], [0], [0], [0]
        self.forwardDone, self.backwardDone = [False], [False]
        self.provideBucket, self.receiveBucket = None, None
        self.px, self.py, self.provideNum, self.receiveNum = None, None, None, None
        self.provideDone = None
        # car state initialization
        for i in range(self.length):
            for j in range(self.channel):
                if self.forwardBucket[i][j] is not None:
                    car = CAR_DICT[self.forwardBucket[i][j]]
                    car.update_dynamic(state=1)
                if self.is_duplex:
                    if self.backwardBucket[i][j] is not None:
                        car = CAR_DICT[self.backwardBucket[i][j]]
                        car.update_dynamic(state=1)
        # first step
        for channel in range(self.channel):
            self.move_in_channel(self.forwardBucket, channel)
            if self.is_duplex:
                self.move_in_channel(self.backwardBucket, channel)

    #
    # function for bucket action
    #
    def move_in_channel(self, bucket, channel):
        # car state: 0,1,2,3 in carport,waiting,finishing,end
        # set guard
        previous_car, previous_state = -1, 1
        for i in range(self.length):
            if bucket[i][channel] is not None:
                car = CAR_DICT[bucket[i][channel]]
                v = min(car.speed, self.speed)
                if car.state == 2:
                    previous_car, previous_state = i, 2
                    continue
                elif i - v > previous_car:
                    bucket[i - v][channel] = bucket[i][channel]
                    bucket[i][channel] = None
                    previous_car, previous_state = i - v, 2
                    car.update_dynamic(state=2, x=previous_car)
                elif previous_state == 2:
                    if previous_car + 1 != i:
                        bucket[previous_car + 1][channel] = bucket[i][channel]
                        bucket[i][channel] = None
                    previous_car, previous_state = previous_car + 1, 2
                    car.update_dynamic(state=2, x=previous_car)
                else:
                    previous_car, previous_state = i, 1

    @staticmethod
    def find_car(st, end, channel, bucket):
        # find car backward
        for i in range(end, st, -1):
            if bucket[i][channel] is not None:
                return i
        return -1

    #
    # provide car
    #
    def first_priority_car(self):
        if self.provideBucket is None:
            print("Please do CAR.setBucket() first!")
        while self.px[0] < self.length:
            car_id = self.provideBucket[self.px[0]][self.py[0]]
            if car_id is not None and CAR_DICT[car_id].state != 2:
                car = CAR_DICT[car_id]
                left = min(car.speed, self.speed)
                # speed enough and no front car
                if left > self.px[0] and self.find_car(-1, self.px[0] - 1, self.py[0], self.provideBucket) == -1:
                    return self.provideBucket[self.px[0]][self.py[0]]
            if self.py[0] == self.channel - 1:
                self.px[0], self.py[0] = self.px[0] + 1, 0
            else:
                self.py[0] += 1
        self.provideDone[0] = True
        return -1

    def first_priority_car_act(self, action):
        if self.provideBucket is None:
            print("Please do CAR.setBucket() first!")
        if action == 0:
            self.provideBucket[self.px[0]][self.py[0]] = None
            self.provideNum[0] -= 1
        elif action == 1:
            car_id = self.provideBucket[self.px[0]][self.py[0]]
            self.provideBucket[self.px[0]][self.py[0]] = None
            self.provideBucket[0][self.py[0]] = car_id
        self.move_in_channel(self.provideBucket, self.py[0])

    #
    # receive car
    #
    def receive_car(self, car_id):
        if self.receiveBucket is None:
            print("Please do CAR.setBucket() first!")
        car = CAR_DICT[car_id]
        left_x = min(self.speed, car.speed) - car.x
        next_cross_id = self.from_ if car.nextCrossId != self.from_ else self.to_
        if left_x <= 0:
            car.update_dynamic(state=2, x=0)
            return 1
        # find front car
        for i in range(self.channel):
            front_car_loc = self.find_car(self.length - left_x - 1, self.length - 1, i, self.receiveBucket)
            # if no front car
            if front_car_loc == -1:
                self.receiveBucket[self.length - left_x][i] = car_id
                self.receiveNum[0] += 1
                car.update_dynamic(state=2, x=self.length - left_x, y=i, present_road=self.id_, road_speed=self.speed,
                                   next_cross_id=next_cross_id)
                return 0
            front_car = CAR_DICT[self.receiveBucket[front_car_loc][i]]
            # if front_car.state == waiting
            if front_car.state == 1:
                return 2
            # if front_car.state == finish and front_car.x != road.__length__()-1
            elif front_car_loc != self.length - 1:
                self.receiveBucket[front_car_loc + 1][i] = car_id
                self.receiveNum[0] += 1
                car.update_dynamic(state=2, x=front_car_loc + 1, y=i, present_road=self.id_, road_speed=self.speed,
                                   next_cross_id=next_cross_id)
                return 0
            # if front_car.state == finish and front_car.x == road.__length__()-1
            else:
                continue
        # if road is full
        car.update_dynamic(state=2, x=0)
        return 1

    #
    # show statistic parameters
    #
    def __forwardBucket__(self):
        return self.forwardBucket

    def __backwardBucket__(self):
        return self.backwardBucket

    def __fx__(self):
        return self.fx[0]

    def __fy__(self):
        return self.fy[0]

    def __bx__(self):
        return self.bx[0]

    def __by__(self):
        return self.by[0]

    def __forwardNum__(self):
        return self.forwardNum[0]

    def __backwardNum__(self):
        return self.backwardNum[0]

    def __forwardDone__(self):
        return self.forwardDone[0]

    def __backwardDone__(self):
        return self.backwardDone[0]

    def __provideBucket__(self):
        return self.provideBucket

    def __receiveBucket__(self):
        return self.receiveBucket

    def __px__(self):
        return self.px[0]

    def __py__(self):
        return self.py[0]

    def __provideNum__(self):
        return self.provideNum[0]

    def __receiveNum__(self):
        return self.receiveNum[0]

    def __provideDone__(self):
        return self.provideDone[0]


class CROSS(object):
    def __init__(self, id_, north_, east_, south_, west_):
        # **** statistic parameters ****#
        self.id_ = id_
        self.roadIds = [north_, east_, south_, west_]
        self.connect_index = len([i for i in self.roadIds if i != -1])
        self.carport = {}
        self.left = []
        self.x, self.y = 0, 0
        self.mapX, self.mapY = 0, 0
        # priorityMap
        self.directionMap = {north_: {east_: 1, south_: 2, west_: -1},
                             east_: {south_: 1, west_: 2, north_: -1},
                             south_: {west_: 1, north_: 2, east_: -1},
                             west_: {north_: 1, east_: 2, south_: -1}}
        # relationship with roads
        self.providerDirection, self.receiverDirection, self.valid_road_direction = [], [], []
        for index, roadId in enumerate(self.roadIds):
            road = ROAD_DICT[roadId] if roadId != -1 else None
            if road is not None and (road.is_duplex or road.to_ == self.id_):
                self.providerDirection.append(index)
            if road is not None and (road.is_duplex or road.from_ == self.id_):
                self.receiverDirection.append(index)
            if road is not None:
                self.valid_road_direction.append(index)
        self.provider = [[direction, self.roadIds[direction]] for direction in self.providerDirection]
        self.receiver = [self.roadIds[direction] for direction in self.receiverDirection]
        self.validRoad = [self.roadIds[direction] for direction in self.valid_road_direction]
        self.provider.sort(key=take_second)
        self.providerDirection = [self.provider[i][0] for i in range(self.provider.__len__())]
        self.provider = [self.provider[i][1] for i in range(self.provider.__len__())]
        # **** dynamic parameters ****#
        self.readyCars = []
        self.carportCarNum = 0
        self.finishCarNum = 0
        # **** flag ****#
        self.done = False
        self.update = False

    # main functions
    def step(self):
        self.update = False
        for roadId in self.validRoad:
            ROAD_DICT[roadId].set_bucket(self.id_)
        # data prepare
        next_car_id, next_car, next_road, next_direction = [], [], [], []
        #
        # 0,1,2,3 denote north,east,south,west
        #
        # for index in range(self.provider.__len__()):
        #     car_id = ROAD_DICT[self.provider[index]].first_priority_car()
        #     if car_id != -1:
        #         road_ids = self.roadIds[:]
        #         CAR_DICT[car_id].find_better_way(road_ids)
        for index in range(self.provider.__len__()):
            car_id = ROAD_DICT[self.provider[index]].first_priority_car()
            if car_id != -1 and CAR_DICT[car_id].is_preset == 0:
                road_ids = self.roadIds[:]
                CAR_DICT[car_id].find_better_way(road_ids)
            next_car_id.append(car_id)
            # if first priority car exists
            if next_car_id[index] != -1:
                next_car.append(CAR_DICT[next_car_id[index]])
                next_road.append(next_car[index].__nextRoad__())
                # nextRoad == -1 => terminal
                if next_road[index] == -1:
                    next_direction.append(2)
                else:
                    # print(next_car_id[index])
                    next_direction.append(self.direction(self.provider[index], next_road[index]))
                    # next_direction.append(self.direction(CAR_DICT[next_car_id[index]].presentRoad, next_road[index]))
            else:
                next_car.append(-1)
                next_road.append(-1)
                next_direction.append(-1)
        # loop
        for presentRoadIndex in range(self.provider.__len__()):
            while next_car[presentRoadIndex] != -1:
                # same next road and high priority lead to conflict
                provider = ROAD_DICT[self.provider[presentRoadIndex]]
                for otherRoadIndex in range(self.provider.__len__()):
                    # conflict
                    # first priority car exists at road self.provider[otherRoadIndex]
                    if next_car[otherRoadIndex] != -1 and \
                            self.is_conflict(self.providerDirection[presentRoadIndex], next_direction[presentRoadIndex],
                                             self.providerDirection[otherRoadIndex], next_direction[otherRoadIndex]):
                        break
                if next_road[presentRoadIndex] == -1:
                    provider.first_priority_car_act(0)
                    CAR_STATE[1] -= 1
                    CAR_STATE[2] += 1
                    self.finishCarNum += 1
                    self.update = True
                else:
                    next_road_ = ROAD_DICT[next_road[presentRoadIndex]]
                    action = next_road_.receive_car(next_car[presentRoadIndex].id_)
                    if action == 2:
                        # waiting conflict
                        break
                    self.update = True
                    provider.first_priority_car_act(action)
                next_car_id[presentRoadIndex] = provider.first_priority_car()
                if next_car_id[presentRoadIndex] != -1:
                    next_car[presentRoadIndex] = CAR_DICT[next_car_id[presentRoadIndex]]
                    next_road[presentRoadIndex] = next_car[presentRoadIndex].__nextRoad__()
                    # nextRoad == -1 => terminal
                    if next_road[presentRoadIndex] == -1:
                        next_direction[presentRoadIndex] = 2
                    else:
                        next_direction[presentRoadIndex] = self.direction(self.provider[presentRoadIndex],
                                                                          next_road[presentRoadIndex])
                else:
                    next_car[presentRoadIndex] = -1
                    next_road[presentRoadIndex] = -1
                    next_direction[presentRoadIndex] = -1
        done = True
        for fromA in range(self.provider.__len__()):
            if next_car[fromA] != -1:
                done = False
        self.done = done

    def find_car_to_go(self):
        if TIME[0] in self.carport.keys():
            self.carport[TIME[0]].sort()
            self.left.extend(self.carport[TIME[0]])

    def random_find_car(self, speed):
        ready_cars = self.left[:]
        for road_id in self.receiver:
            ROAD_DICT[road_id].set_bucket(self.id_)
        for car_id in self.left:
            if speed != 0:
                if CAR_DICT[car_id].speed != speed:
                    continue
            road_id = CAR_DICT[car_id].__nextRoad__()
            road = ROAD_DICT[road_id]
            if road_id not in self.receiver:
                print("Car(%d).Road(%d) not in cross(%d).function:class.outOfCarport" % (car_id, road_id, self.id_))
            act = road.receive_car(car_id)
            if act != 0:
                continue
            ready_cars.remove(car_id)
            self.left = ready_cars[:]
            CAR_DICT[car_id].plan_time = TIME[0]
            self.carportCarNum -= 1
            CAR_STATE[0] -= 1
            CAR_STATE[1] += 1
            return 1
        return 0

    # other functions
    #
    @staticmethod
    def is_conflict(from_a, direction_a, from_b, direction_b):
        if (from_a + direction_a) % 4 == (from_b + direction_b) % 4 and direction_a < direction_b:
            return True
        else:
            return False

    def direction(self, provider_id, receiver_id):
        return self.directionMap[provider_id][receiver_id]

    def set_done(self, flag):
        self.done = flag

    def set_loc(self, x, y):
        self.x, self.y = x, y

    def set_map_loc(self, map_x, map_y):
        self.mapX, self.mapY = map_x, map_y

    def road_direction(self, road_id):
        if self.roadIds[0] == road_id:
            return 0
        elif self.roadIds[1] == road_id:
            return 1
        elif self.roadIds[2] == road_id:
            return 2
        elif self.roadIds[3] == road_id:
            return 3
        else:
            return -1

    def carport_initial(self, time_plan, car_id):
        if time_plan not in self.carport.keys():
            self.carport[time_plan] = [car_id]
        else:
            self.carport[time_plan].append(car_id)
        self.carportCarNum += 1

    #
    # show statistic parameters
    #
    def __id__(self):
        return self.id_

    def __roadIds__(self):
        return self.roadIds

    def __providerDirection__(self):
        return self.providerDirection

    def __receiverDirection__(self):
        return self.receiverDirection

    def __validRoadDirection__(self):
        return self.validRoadDirection

    def __provider__(self):
        return self.provider

    def __receiver__(self):
        return self.receiver

    def __validRoad__(self):
        return self.validRoad

    def __x__(self):
        return self.x

    def __y__(self):
        return self.y

    def __mapX__(self):
        return self.mapX

    def __mapY__(self):
        return self.mapY

    def __done__(self):
        return self.done

    #
    # show dynamic parameters
    #
    def __carportCarNum__(self):
        return self.carportCarNum

    def __finishCarNum__(self):
        return self.finishCarNum

    def __update__(self):
        return self.update

    #
    # show some important info
    #
    def __loc__(self):
        return self.x, self.y

    def __mapLoc__(self):
        return self.mapX, self.mapY


class Simulation(object):
    def __init__(self):
        self.dead = False

    def step(self):
        print("time:%d" % TIME[0])
        for cross_id in CROSS_NAMESPACE:
            CROSS_DICT[cross_id].set_done(False)
        print("pre-movement...")
        for road in ROAD_NAMESPACE:
            ROAD_DICT[road].step_init()
        print("while loop...")
        unfinished_cross = CROSS_NAMESPACE
        while unfinished_cross.__len__() > 0:
            self.dead = True
            next_cross = []
            for cross_id in unfinished_cross:
                cross = CROSS_DICT[cross_id]
                cross.step()
                if not cross.__done__():
                    next_cross.append(cross_id)
                if cross.__update__() or cross.__done__():
                    self.dead = False
            unfinished_cross = next_cross
            assert self.dead is False, print("dead lock in", unfinished_cross)
        print("car pulling away from carport")
        if TIME[0] == 0:
            for i in range(CROSS_NAMESPACE.__len__()):
                cross_id = CROSS_NAMESPACE[i]
                for roadId in CROSS_DICT[cross_id].__validRoad__():
                    ROAD_DICT[roadId].set_bucket(cross_id)
        else:
            if TIME[0] in CAR_BEGIN_TIME_DICT:  # 先将即将出发的车放入发车列表中
                begin_car_dict = CAR_BEGIN_TIME_DICT[TIME[0]]
                for i in CAR_LEFT_DIVIDED_BY_SPEED:
                    begin_car_lst = begin_car_dict[i][:]
                    CAR_LEFT_DIVIDED_BY_SPEED[i].extend(begin_car_lst)
            if TIME[0] == TIME_GO[0]:
                TIME_GO[0] += INTERVAL_TIME
                now_speed = 0
                for speed_value in CAR_LEFT_DIVIDED_BY_SPEED:
                    if CAR_LEFT_DIVIDED_BY_SPEED[speed_value]:
                        now_speed = speed_value
                        break
                print(CAR_LEFT_DIVIDED_BY_SPEED)
                if now_speed != 0:
                    this_time_car_num = METHOD_2_BATCH_NUM[SPEED_LIST.index(now_speed)]
                else:
                    this_time_car_num = METHOD_2_BATCH_NUM[-1]
                now_left_car_num = 0
                for speed_value in CAR_LEFT_DIVIDED_BY_SPEED:
                    if not CAR_LEFT_DIVIDED_BY_SPEED[speed_value]:
                        continue
                    temp_car_left_speed = CAR_LEFT_DIVIDED_BY_SPEED[speed_value][:]
                    for car_id in CAR_LEFT_DIVIDED_BY_SPEED[speed_value]:
                        now_process_car = CAR_DICT[car_id]
                        result = now_process_car.start_driving()
                        if result == 1:
                            now_left_car_num += 1
                            temp_car_left_speed.remove(car_id)
                        if now_left_car_num >= this_time_car_num:
                            break
                    CAR_LEFT_DIVIDED_BY_SPEED[speed_value] = temp_car_left_speed[:]
                    # if is_finish == 1:
                    #     break
                    # if TIME[0] in list(range(1, 11)):
                    break

    def simulate(self):
        # visualize = visualization()
        # visualize.crossLocGen()
        while True:
            self.step()
            # visualize.drawMap()
            if CAR_STATE[2] == CAR_NAMESPACE.__len__():
                # print(CAR_STATE[2])
                break
            if self.dead:
                break
            TIME[0] += 1
            print(CAR_STATE)


class RoadMap:
    def __init__(self, cross_dict, road_dict):
        self.cross_dict = cross_dict
        self.road_dict = road_dict
        self.road_num = len(road_dict)
        self.cross_num = len(cross_dict)
        self.adjacency_matrix = [[float('inf') for _ in range(self.cross_num + 1)] for _ in range(self.cross_num + 1)]
        for road_id in self.road_dict:
            now_road = self.road_dict[road_id]
            self.adjacency_matrix[now_road.from_][now_road.to_] = now_road.road_value_forward
            if self.road_dict[road_id].is_duplex == 1:
                self.adjacency_matrix[now_road.to_][now_road.from_] = now_road.road_value_backward
        self.road_message = [[float('inf') for _ in range(self.cross_num + 1)] for _ in range(self.cross_num + 1)]
        for road_id in self.road_dict:
            now_road = self.road_dict[road_id]
            self.road_message[now_road.from_][now_road.to_] = road_id
            if self.road_dict[road_id].is_duplex == 1:
                self.road_message[now_road.to_][now_road.from_] = road_id
        self.route_plan = [[[] for _ in range(self.cross_num + 1)] for _ in range(self.cross_num + 1)]
        self.route_length = [[float('inf') for _ in range(self.cross_num + 1)] for _ in range(self.cross_num + 1)]

    def dijkstra(self):
        for i in range(1, self.cross_num + 1):
            flag = [0 for _ in range(self.cross_num + 1)]
            flag[i] = 1
            dis = [float('inf') for _ in range(self.cross_num + 1)]
            route = [[] for _ in range(self.cross_num + 1)]
            for _ in range(self.cross_num - 1):
                for j in range(1, self.cross_num + 1):
                    if self.adjacency_matrix[i][j] < dis[j]:
                        route[j].append(self.road_message[i][j])
                        dis[j] = self.adjacency_matrix[i][j]
                min_path = float('inf')
                for j in range(1, self.cross_num):
                    if flag[j] == 0 and min_path > dis[j]:
                        start = j
                        min_path = dis[j]
                flag[start] = 1
                for j in range(1, self.cross_num + 1):
                    if dis[start] + self.adjacency_matrix[start][j] < dis[j]:
                        temp = route[start][:]
                        temp.append(self.road_message[start][j])
                        route[j] = temp[:]
                        dis[j] = dis[start] + self.adjacency_matrix[start][j]
            # for j in range(1, self.cross_num + 1):
            #     route[j] = [dis[j]] + route[j]
            self.route_plan[i] = route[:]
            self.route_length[i] = dis[:]


def take_second(elem):
    return elem[1]


def open_txt(path):
    content = []
    with open(path) as f:
        _ = f.readline()
        line = f.readline()
        while line:
            content.append(eval(line.strip('\n')))
            line = f.readline()
    return content


def save_answer(answer_path, answer):
    with open(answer_path, 'w') as f:
        for i in answer:
            f.write(i + '\n')


def main():
    # if len(sys.argv) != 6:
    #     logging.info('please input args: car_path, road_path, cross_path, answerPath')
    #     exit(1)
    #
    # car_path = sys.argv[1]
    # road_path = sys.argv[2]
    # cross_path = sys.argv[3]
    # preset_answer_path = sys.argv[4]
    # answer_path = sys.argv[5]

    # logging.info("car_path is %s" % car_path)
    # logging.info("road_path is %s" % road_path)
    # logging.info("cross_path is %s" % cross_path)
    # logging.info("answer_path is %s" % answer_path)

    file_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config')
    car_path = os.path.join(file_path, 'car.txt')
    road_path = os.path.join(file_path, 'road.txt')
    cross_path = os.path.join(file_path, 'cross.txt')
    preset_answer_path = os.path.join(file_path, 'presetAnswer.txt')
    answer_path = os.path.join(file_path, 'answer.txt')

    car_list = open_txt(car_path)
    road_list = open_txt(road_path)
    cross_list = open_txt(cross_path)
    present_answer_list = open_txt(preset_answer_path)
    present_dict = {}
    for i in present_answer_list:
        present_dict[int(i[0])] = {'time': int(i[1]),
                                   'route': [i for i in i[2:]]}
    cross_map = [0]
    for i in cross_list:
        cross_map.append(i[0])
    for i in road_list:
        road_from = cross_map.index(int(i[4]))
        road_to = cross_map.index(int(i[5]))
        ROAD_NAMESPACE.append(int(i[0]))
        ROAD_DICT[int(i[0])] = ROAD(int(i[0]), int(i[1]), int(i[2]), int(i[3]), road_from, road_to, int(i[6]))
    for i in cross_list:
        CROSS_NAMESPACE.append(cross_map.index(int(i[0])))
        CROSS_DICT[cross_map.index(int(i[0]))] = CROSS(cross_map.index(int(i[0])), int(i[1]), int(i[2]), int(i[3]), int(i[4]))  # 将道路实例作为路口实例的一个属性
    for road_id in ROAD_DICT:
        ROAD_DICT[road_id].road_value_forward = ROAD_DICT[road_id].road_value_forward * CONNECTED_NUM[CROSS_DICT[ROAD_DICT[road_id].to_].connect_index]
        if ROAD_DICT[road_id].is_duplex == 1:
            ROAD_DICT[road_id].road_value_backward = ROAD_DICT[road_id].road_value_forward * CONNECTED_NUM[
                CROSS_DICT[ROAD_DICT[road_id].from_].connect_index]
    global road_map, SPEED_LIST
    road_map = RoadMap(CROSS_DICT, ROAD_DICT)
    road_map.dijkstra()
    # car_lst = []
    for i in car_list:
        CAR_NAMESPACE.append(int(i[0]))
        # car_lst.append(CAR(int(i[0]), cross_map.index(int(i[1])), cross_map.index(int(i[2])), int(i[3]), int(i[4]),
        #                    int(i[5]), int(i[6])))
        CAR_DICT[int(i[0])] = CAR(int(i[0]), cross_map.index(int(i[1])), cross_map.index(int(i[2])), int(i[3]),
                                  int(i[4]), int(i[5]), int(i[6]))
        if int(i[6]) == 0:
            route = road_map.route_plan[CAR_DICT[int(i[0])].from_][CAR_DICT[int(i[0])].to_]
            CAR_DICT[int(i[0])].simulate_init(int(CAR_DICT[int(i[0])].plan_time), route)
        else:
            route = present_dict[int(i[0])]['route']
            plan_time = present_dict[int(i[0])]['time']
            CAR_DICT[int(i[0])].simulate_init(plan_time, route)
        if i[3] not in SPEED_LIST:
            SPEED_LIST.append(i[3])
    SPEED_LIST = sorted(SPEED_LIST, reverse=True)
    for i in SPEED_LIST:
        CAR_LEFT_DIVIDED_BY_SPEED[i] = []
    for i in range(len(CAR_NAMESPACE)):
        car_id = CAR_NAMESPACE[i]
        if CAR_DICT[car_id].plan_time not in CAR_BEGIN_TIME_DICT.keys():
            CAR_BEGIN_TIME_DICT[CAR_DICT[car_id].plan_time] = {}
            for j in SPEED_LIST:
                if j == CAR_DICT[car_id].speed:
                    CAR_BEGIN_TIME_DICT[CAR_DICT[car_id].plan_time][j] = [car_id]
                else:
                    CAR_BEGIN_TIME_DICT[CAR_DICT[car_id].plan_time][j] = []
        else:
            CAR_BEGIN_TIME_DICT[CAR_DICT[car_id].plan_time][CAR_DICT[car_id].speed].append(car_id)
    sum_value = 0
    for i in CAR_BEGIN_TIME_DICT:
        for j in CAR_BEGIN_TIME_DICT[i]:
            sum_value += len(CAR_BEGIN_TIME_DICT[i][j])
    CAR_STATE[0] = CAR_NAMESPACE.__len__()
    # **** cross initialization ****#
    for carId in CAR_NAMESPACE:
        CROSS_DICT[CAR_DICT[carId].from_].carport_initial(CAR_DICT[carId].plan_time, carId)
    CAR_NAMESPACE.sort()
    CROSS_NAMESPACE.sort()
    # simulator
    simulate = Simulation()
    simulate.simulate()
    answer = []
    for carId in CAR_NAMESPACE:
        if CAR_DICT[car_id].is_preset == 0:
            answer.append('(' + str(carId) + ', ' + str(CAR_DICT[carId].plan_time) + ', '
                          + ', '.join(list(map(str, CAR_DICT[carId].route))) + ')')
    save_answer(answer_path, answer)


if __name__ == "__main__":
    main()
