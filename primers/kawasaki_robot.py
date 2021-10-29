import sys
import json
import time
import socket
import math
from math import sin, cos
from math import *

import numpy as np
from scipy.spatial.transform import Rotation as R


# 欧拉旋转
def euler_rotation(uvw, xyz, degrees=False):
    return R.from_euler('xyz', uvw, degrees=degrees).apply(xyz)


rs10 = {                        # rs10 基卡特
    'r15': 1400,                # 作用区间
    'cea': 35.9 / 50 * 9 / 16,  # 相机焦距的相关参数  0.20
}


def pose_human_to_kawasaki(pose):
    x, y, z, u, v, w = pose
    x, y, z = -y, x, z
    m1 = R.from_euler('zxy', (-w, v, -u), degrees=True)
    m2 = R.from_euler('x', 180, degrees=True)
    u, v, w = (m1 * m2).as_euler('zyz', degrees=True)
    return x, y, z, u, v, w


def pose_kawasaki_to_human(pose):
    x, y, z, u, v, w = pose
    x, y, z = y, -x, z
    m3 = R.from_euler('zyz', (u, v, w), degrees=True)
    m2 = R.from_euler('x', -180, degrees=True)  # m2.inv()
    w, v, u = (m3 * m2).as_euler('zxy', degrees=True)
    return [x, y, z, -u, v, -w]


class Coord:

    # 预防误操作
    __slots__ = (
        'robot_params',
        'ax', 'ay', 'az',   # a: 绝对
        'rx', 'ry', 'rz',   # r: 相对
        'su', 'sv', 'sw',
        'ar', 'rr',
        'ox', 'oy', 'oz',   # o: 物体
        'od', 'ow', 'oh')

    def __init__(self, robot_params=rs10):
        self.robot_params = robot_params        # 机械臂的相关参数
        self.ax, self.ay, self.az = 0, 0, 0     # 绝对位移
        self.rx, self.ry, self.rz = 0, 0, 0     # 相对位移
        self.su, self.sv, self.sw = 0, 0, 0     # 旋转角度
        self.ar, self.rr = 0, 1                 # 半径

    def place_object(self, ox, oy, oz, od, ow, oh):
        self.ox, self.oy, self.oz = ox, oy, oz  # 物品的所在位置，前上左
        self.od, self.ow, self.oh = od, ow, oh  # 物体的尺寸，深宽高

    def gen_world_n(self):
        # 最终相机需要对准的点位
        ox = self.ox + self.ax + self.od * self.rx
        oy = self.oy + self.ay + self.ow * self.ry
        oz = self.oz + self.az + self.oh * self.rz
        # 最终相机到物体的距离
        ar = self.ar + self.oh * self.rr / self.robot_params['cea']
        # 翻译成机械臂的世界坐标
        u, v, w = self.su, self.sv, self.sw
        # 俯视运动
        x, y, z = euler_rotation((u, v, w), [-ar, 0, 0], degrees=True) + [ox, oy, oz]
        # 编辑输出
        return [x, y, z, u, v, w]


class Robot:

    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.debug = False

    def recv(self, feedback='>'):
        r = b''
        print('recv: ---------------------------------')
        while True:
            c = self.sock.recv(256)
            try:
                print(c.decode('GBK'))
            except:
                print(c)
            r = r + c
            if ord(feedback) in c:
                break
            time.sleep(self.timeout)
        print('end recv: =============================')
        return r

    def execute(self, cmd, debug=True, feedback='>'):
        if isinstance(cmd, str):
            cmd = cmd.encode()
        if not cmd.endswith(b'\n'):
            cmd = cmd + b'\n'
        # 类似ipython
        # >>> print(1)
        # 1
        # >>>
        self.sock.send(cmd)
        r = self.recv()
        return r

    def connect(self, host='192.168.0.2', timeout=0.5):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, 23))
        return self.execute(b'as\n')

    # 5.6 系统控制指令
    def get_status(self):
        '''
        STATUS
        机器人状态
        再现模式

        环境设定状况:
         监控速度(%)=       20.0
         ALWAYS程序速度(%)=      100.0     100.0
         ALWAYS 精度[mm] =      100.0

        Stepper状态   程序未运行
        程序运行次数
          已运行完成次数:       1
          剩余运行次数:         0
        程序名              优先级 步骤号
         multipose_move             0    4         JMOVE TRANS(-215.02,1355.3,-106.25,-146.04,142.52,46.435)
        >
        '''
        cmd = b'STATUS\n'
        return self.execute(cmd).decode('GBK')

    def get_where(self, mode=''):
        '''
        WHERE
             JT1       JT2       JT3       JT4       JT5       JT6
            -6.552    23.595   -87.421  -147.271    88.570  -167.663
            X[mm]     Y[mm]     Z[mm]     O[deg]    A[deg]    T[deg]
          -214.985  1355.266  -106.276  -146.040   142.521    46.437
        >
        '''
        if mode:
            cmd =f'WHERE {mode}\n'
            r = self.execute(cmd, feedback='\n')
            self.sock.send(b'\n')
        else:
            cmd = b'WHERE\n'
            r = self.execute(cmd)
        return r.decode('GBK')

    def get_io(self):
        cmd = b'IO\n'
        return self.execute(cmd).decode('GBK')

    def get_switch(self):
        cmd = b'SWITCH\n\n'
        return self.execute(cmd).decode('GBK')

    def set_rep_once(self, p=False):
        p = 'ON' if p else 'OFF'
        cmd = f'REP_ONCE {p}\n'
        return self.execute(cmd).decode('GBK')

    @property
    def _is_moving(self):
        # r = self.execute(b'DO SWITCH RUN\n')
        # r = self.execute(b'DO TASK(1)\n')
        cmd = b'STATUS'
        # cmd = b'DO TASK("linemove_multipose")'
        r = self.execute(cmd)
        print(r.decode('GBK'))
        return r

    @property
    def is_moving(self):
        return '程序运行中' in self.get_status()

    def wait(self):
        time.sleep(0.5)
        while self.is_moving:
            time.sleep(0.1)

    def get_joint_and_pose(self):
        r = self.execute(b'wh\n')
        r = r.split(b'\r\n')
        joint = r[2].split()
        joint = [float(x) for x in joint]
        pose = r[4].split()
        pose = [float(x) for x in pose]
        return joint, pose

    @property
    def world_n(self):
        pose = self.get_joint_and_pose()[1]
        return pose_kawasaki_to_human(pose)

    @property
    def axis_n(self):
        return self.get_joint_and_pose()[0]

    def pose_move(self, pose, cmd):
        # LMOVE or JMOVE
        pose = pose_human_to_kawasaki(pose)
        params = ', '.join(str(round(x, 3)) for x in pose)
        cmd = f'DO {cmd} TRANS({params})\n'
        return self.execute(cmd)

    def linemove(self, pose):
        return self.pose_move(pose, cmd='LMOVE')

    def freemove(self, pose):
        # 自由的方式移动到指定坐标点
        return self.pose_move(pose, cmd='JMOVE')

    def _print(self, s):
        self.sock.send(f'PRINT "{s}"\n'.encode())

    def _edit_project(self, project_name):
        self._cur_project_name = project_name
        cmd = f'EDIT {project_name}, 1\n'.encode()
        self.sock.send(cmd)  # 编辑程序
        self.sock.send(b'D 100\n')          # 删除之前的代码
        self.sock.send(b'ACCURACY 100 ALWAYS\n')
        self.sock.send(b'progress = 0\n')

    def _execute_project(self, waiting=False):
        self._break()
        self.sock.send(b'progress = -2\n')
        self.execute(b'E\n')              # 退出编辑
        r = self.execute(f'EXECUTE {self._cur_project_name}\n')
        if waiting:
            while '程序结束'.encode('GBK') not in self.recv(feedback='\n'):
                pass
        return r

    def set_progress(self, value):
        r = self.execute(f'progress = {value}\n')

    def get_progress(self):
        r = self.execute('PRINT progress')
        r = r.split(b'\n')[-2]
        print(r)
        return int(r)

    def _end_edit_and_execute_project(self):
        return self._execute_project()

    def tool(self, x=0, y=0, z=0):
        x, y, z = -y, x, z
        self.execute(f'POINT p = TRANS({x}, {y}, {z})\n\n')
        self.execute(b'DO TOOL p\n')

    def _tool(self, x=0, y=0, z=0):
        x, y, z = -y, x, z
        self.sock.send(f'TOOL TRANS({x}, {y}, {z})\n'.encode())

    def _break(self):
        # 等待上一个动作停下来
        self.sock.send(b'BREAK\n')

    def _cmove(self, p0, p1, p2, _break=True):
        self._move('JMOVE', p0)
        if _break:
            self._break()
        self._move('C1MOVE', p1)
        self._move('C2MOVE', p2)

    def _move(self, cmd, pose):
        # LMOVE or JMOVE
        pose = pose_human_to_kawasaki(pose)
        params = ', '.join(str(round(x, 3)) for x in pose)
        statement = f'{cmd} TRANS({params})\n'
        statement = statement.encode()
        self.sock.send(statement)

    def _multipose_move(self, poses, cmd):
        for pose in poses:
            self._move(cmd, pose)

    def _uwrist(self):
        # 改变形态，使JT5的角度为正值
        self.sock.send(b'UWRIST\n')

    def _set_speed(self, s, unit=''):
        cmd = f'SPEED {s}{unit}\n'.encode()
        self.sock.send(cmd)

    def draw_ex(self, x=0, y=0, z=0, u=0, v=0, w=0):
        # 相对世界坐标系移动
        self._edit_project('draw_ex')

        statement = b'REP_ONCE OFF\n'
        self.sock.send(statement)

        params = -y, x, z, -v, u, w
        params = ', '.join(str(round(x, 3)) for x in params)
        statement = f'DRAW {params}\n'
        statement = statement.encode()
        self.sock.send(statement)

        return self._execute_project()

    def tdraw_ex(self, x=0, y=0, z=0, u=0, v=0, w=0):
        # 相对相机坐标系移动
        self._edit_project('draw_ex')

        statement = b'REP_ONCE OFF\n'
        self.sock.send(statement)

        params = -y, -x, -z, -v, u, w
        params = ', '.join(str(round(x, 3)) for x in params)
        statement = f'TDRAW {params}\n'
        statement = statement.encode()
        self.sock.send(statement)

        return self._execute_project()

    def ereset(self):
        self.execute(b'ereset\n')

    def multipose_move(self, poses, cmd):
        self._edit_project('multipose_move')
        self._multipose_move(poses, cmd)
        return self._execute_project()

    def linemove_multipose(self, poses):
        return self.multipose_move(poses, cmd='LMOVE')

    def freemove_multipose(self, poses):
        return self.multipose_move(poses, cmd='JMOVE')

    def draw(self, x=0, y=0, z=0, u=0, v=0, w=0):
        # 相对世界坐标系移动
        pose = self.world_n
        pose[0] += x
        pose[1] += y
        pose[2] += z

        m1 = R.from_euler('xyz', (u, v, w), degrees=True)
        m2 = R.from_euler('xyz', pose[3:], degrees=True)
        pose[3:] = (m1 * m2).as_euler('xyz', degrees=True)

        self.freemove(pose)

    def tdraw(self, x=0, y=0, z=0, u=0, v=0, w=0):
        # 相对相机坐标系移动
        pose = self.world_n
        x, y, z = R.from_euler('xyz', pose[3:], degrees=True).apply([x, y, z])
        pose[0] += x
        pose[1] += y
        pose[2] += z

        m1 = R.from_euler('xyz', (u, v, w), degrees=True)
        m2 = R.from_euler('xyz', pose[3:], degrees=True)
        pose[3:] = (m2 * m1).as_euler('xyz', degrees=True)

        self.freemove(pose)

    def drive(self, joint_id, degrees):
        cmd = f'DO DRIVE {joint_id}, {degrees}\n'
        return self.execute(cmd)

    def cmove(self, p0, p1, p2):
        self._edit_project('cmove')     # 编辑程序
        self._cmove(p0, p1, p2)
        return self._execute_project()  # 退出编辑并执行

    def set_speed(self, s):
        cmd = f'SPEED {s}\n'
        return self.execute(cmd)

    def stop(self):
        return self.execute(b'HOLD\n')

    def disconnect(self):
        self.sock.close()


def tests():
    robot = Robot()
    robot.connect()

    joint, pose = robot.get_joint_and_pose()
    print('joint:', joint)
    print('pose:', pose)
    # robot.freemove([-0.059, 48.668, -113.737, -3.626, -44.971, -195.341])
    # robot.freemove([-50.0, 700.551, -512.056, -84.352, 152.529, -12.946])
    robot.set_speed(10)
    # robot.linemove( [150.0, 700.551, -512.056, -84.352, 152.529, -12.946])
    # time.sleep(1)
    # robot.stop()
    # robot.linemove_multipose([[150.0, 700.551, -512.056, -84.352, 152.529, -12.946], [-50.0, 700.551, -512.056, -84.352, 152.529, -12.946], [-50.0, 700.551, -462.056, -84.352, 152.529, -12.946]])
    robot.cmove([-50.0, 700.551, -512.056, -84.352, 152.529, -12.946], [150.0, 700.551, -512.056, -84.352, 152.529, -12.946], [-50.0, 700.551, -512.056, -84.352, 152.529, -12.946])
    robot.disconnect()

    '''
    起点(当前点)
    物品点(圆心点)
    旋转角度
    旋转方向
    '''


def tests_2():
    poses = []
    c = Coord()
    c.place_object(1170, 0, -500, 1, 50, 210)
    c.ar, c.rr = 400, 0
    c.sw = -30
    pose = c.gen_world_n()
    poses.append(pose)
    c.sw = 0
    pose = c.gen_world_n()
    poses.append(pose)
    c.sw = 30
    pose = c.gen_world_n()
    poses.append(pose)
    # poses.append(poses[0])
    print(poses)
    exit()

    robot = Robot()
    robot.connect()

    joint, pose = robot.get_joint_and_pose()
    print('joint:', joint)
    print('pose:', pose)
    # robot.freemove([-50.0, 700.551, -512.056, -84.352, 152.529, -12.946])
    # robot.freemove([-0.059, 48.668, -113.737, -3.626, -44.971, -195.341])
    robot.set_speed(10)
    # robot.freemove([11.035, 44.309, -125.425, 0.068, -10.214, -161.107])
    # robot.linemove([0.0, 770.0, -500.0, 0, 180, 180])
    # robot.linemove([69.459, 776.077, -500.0, -170.0, 180.0, 0.0])
    # time.sleep(1)
    # robot.stop()
    # robot.linemove(poses[0])
    print(poses[1])
    robot.linemove_multipose(poses)
    # robot.cmove(poses)
    robot.disconnect()


def tests_3():
    poses = []
    c = Coord()
    c.place_object(800, 10, -100, 1, 50, 210)   # 物品的摆放位置及其尺寸
    c.ar, c.rr = 400, 0                         # 相机到物品的距离
    # c.sv = 10
    # c.sw = -90
    # pose = c.gen_world_n()
    c.su, c.sv, c.sw = 0, 1, 0
    pose = c.gen_world_n()
    poses.append(pose)              # 初始点

    c.ar, c.rr = 700, 0

    c.su, c.sv, c.sw = 0, 1, -91    # 起点
    pose = c.gen_world_n()
    poses.append(pose)

    c.su, c.sv, c.sw = 0, 10, -95  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 20, -100  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 30, -105  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 40, -110  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 50, -120  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 60, -140  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 70, -160  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)

    c.su, c.sv, c.sw = 0, 80, -180  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)

    c.su, c.sv, c.sw = 0, 70, -200  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 60, -220  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 50, -240  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 40, -250  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 30, -255  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 10, -260  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 10, -265  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)

    c.su, c.sv, c.sw = 0, 0, -270  # 终点
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 10, -270  # 终点
    c.ar, c.rr = 300, 0
    pose = c.gen_world_n()
    poses.append(pose)
    for pose in poses:
        print(pose)
    # exit()

    robot = Robot()
    robot.connect()

    joint, pose = robot.get_joint_and_pose()
    print('joint:', joint)
    print('pose:', pose)
    robot.set_speed(10)
    robot.linemove_multipose(poses)
    # robot.linemove_multipose(poses[0:1])
    # robot.linemove_multipose(poses[0:10])
    # robot.linemove_multipose(poses[1:2])
    # robot.linemove_multipose(poses[2:3])
    # robot.linemove_multipose(poses[3:4])
    # robot.linemove_multipose(poses[5:8])
    # robot.linemove_multipose(poses[8:10])
    # robot.linemove_multipose(poses[10:])

    for i in range(5):
        robot.get_status()
        print('=' * 80)
        time.sleep(1)

    while '程序未运行' not in robot.get_status():
        print('=' * 80)
        time.sleep(1)

    robot.disconnect()


def tests_4():
    poses = []
    c = Coord()
    c.place_object(800, 0, -500, 1, 50, 210)   # 物品的摆放位置及其尺寸
    c.ar, c.rr = 400, 0                         # 相机到物品的距离
    c.sv = 80
    c.sw = 0
    pose = c.gen_world_n()
    print(pose)


def tests_5():
    # pose = [0, 540, 140, 0, 180, 0]
    pose = [0, 400, 141, -180, 180, 0]
    u, v, w = euler_human_to_kawasaki((0, 10, 0))
    pose[3:] = u, v, w
    robot = Robot()
    robot.connect()
    robot.set_speed(10)
    robot.linemove_multipose([pose])
    robot.disconnect()


def tests_6():
    poses = []
    c = Coord()
    c.place_object(800, 0, -100, 1, 50, 210)   # 物品的摆放位置及其尺寸
    c.ar, c.rr = 400, 0                         # 相机到物品的距离
    pose = c.gen_world_n()
    poses.append(pose)              # 初始点
    c.su, c.sv, c.sw = 0, 0, -90    # 起点
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 0, 0  # 背面俯拍
    pose = c.gen_world_n()
    poses.append(pose)
    c.su, c.sv, c.sw = 0, 0, 90  # 终点
    pose = c.gen_world_n()
    poses.append(pose)
    for pose in poses:
        print(pose)
    exit()

    robot = Robot()
    robot.connect()

    joint, pose = robot.get_joint_and_pose()
    print('joint:', joint)
    print('pose:', pose)
    robot.set_speed(10)
    # robot.linemove_multipose(poses[0:1])
    robot.linemove_multipose(poses[1:2])
    # robot.linemove_multipose(poses[2:3])
    robot.disconnect()


def tests_7():
    robot = Robot()
    robot.connect()

    print(robot.get_status())
    print('程序未运行' in robot.get_status())

    robot.disconnect()


def tests_8():
    robot = Robot()
    robot.connect()

    pose = robot.world_n
    pose[3] = 0
    pose[4] = 30
    pose[5] = 0
    robot.freemove(pose)
    robot.disconnect()


def tests_9():
    robot = Robot()
    robot.connect()
    robot.draw(-100, 0, 0)
    robot.disconnect()


def tests_10():
    robot = Robot()
    robot.connect()
    robot.drive(2, -10)
    robot.disconnect()


def tests_11():
    robot = Robot()
    robot.connect()
    # w = robot.world_n
    # print(w)
    # w[0] -= 100
    # print(w)
    # robot.linemove(w)
    # robot.freemove(w)
    robot.tdraw(-100,0,0)
    robot.disconnect()


def tests_12():
    robot = Robot()
    robot.connect()
    c = Coord()
    c.place_object(1200, 10, 0, 1, 50, 210)     # 物品的摆放位置及其尺寸
    c.ar, c.rr = 600, 0                         # 相机到物品的距离
    c.su = 0
    c.sv = 0
    c.sw = 0
    pose = c.gen_world_n()
    poses = [pose]
    robot.freemove_multipose(poses)
    robot.disconnect()


def tests_13():
    robot = Robot()
    robot.connect()
    # robot.tdraw(v=30)
    robot.draw(y=100)
    robot.disconnect()


def tests_14():
    robot = Robot()
    robot.connect()

    # robot.drive(1, 30)
    robot.tdraw_multipose(x=100, n=1)
    robot.disconnect()


def tests_15():
    robot = Robot()
    robot.connect()
    # 相对相机坐标系移动
    # robot.tdraw(z=100)
    robot.tdraw_ex(x=1, y=0, z=0, q=1)
    robot.disconnect()


def tests_16():
    robot = Robot()
    robot.connect()

    poses = []
    c = Coord()
    c.place_object(800, 10, 100, 1, 50, 210)   # 物品的摆放位置及其尺寸
    c.ar, c.rr = 400, 0                         # 相机到物品的距离
    c.su, c.sv, c.sw = 0, 10, -90    # 起点
    p0 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 80, -180    # 起点
    p1 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 90    # 起点
    p2 = c.gen_world_n()

    # robot.freemove(p2)

    robot.cmove(p2, p1, p0)
    # robot.cmove(p0, p1, p2)

    robot.disconnect()


def tests_17():
    robot = Robot()
    robot.connect()

    poses = []
    c = Coord()
    c.place_object(800, 10, -120, 1, 50, 210)   # 物品的摆放位置及其尺寸
    c.ar, c.rr = 400, 0                         # 相机到物品的距离
    c.su, c.sv, c.sw = 0, 10, -90    # 起点
    p0 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 80, -180    # 起点
    p1 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 90    # 起点
    p2 = c.gen_world_n()

    robot._edit_project('tests_19')

    robot._uwrist()
    # robot._move('JMOVE', p1)
    # robot._move('JMOVE', p2)
    # robot._break()
    # robot._move('C1MOVE', p1)
    # robot._move('C2MOVE', p0)

    # robot._move('JMOVE', p0)
    # robot._move('JMOVE', p1)
    # robot._move('JMOVE', p0)
    # robot._cmove(p0, p1, p2)
    robot._cmove(p2, p1, p0)
    robot._end_edit_and_execute_project()

    robot.disconnect()


def tests_18():
    robot = Robot()
    robot.connect()

    poses = []
    c = Coord()
    c.place_object(1300, 10, 0, 1, 50, 210)   # 物品的摆放位置及其尺寸
    c.ar, c.rr = 600, 0                         # 相机到物品的距离
    c.su, c.sv, c.sw = 0, 10, -40    # 起点
    p0 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 0    # 起点
    p1 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 40    # 起点
    p2 = c.gen_world_n()

    robot.freemove_multipose([p0, p1, p2])

    robot.disconnect()


def tests_19():
    robot = Robot()
    robot.connect()

    # robot.tool(z=0)

    robot._edit_project('tests_19')
    # robot._tool(z=200)

    poses = []
    c = Coord()
    c.place_object(600 + 730, 20, -50, 1, 50, 210)   # 物品的摆放位置及其尺寸

    c.ax = 200
    c.ar, c.rr = 800, 0                         # 相机到物品的距离
    c.su, c.sv, c.sw = 0, 10, -40   # 起点
    p0 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 0     # 中间点
    p1 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 40    # 终点
    p2 = c.gen_world_n()

    # robot.sock.send(b'#pose = 0.00, 0.00, 0.00, 0.00, 0.00, 0.00\n')

    # robot._set_speed(1.0, unit='s')
    robot._multipose_move([p0], 'JMOVE')
    robot._print('p0 start')
    robot._multipose_move([p1], 'JMOVE')
    robot._print('p1 start')
    robot._multipose_move([p2], 'JMOVE')
    robot._print('p2 start')

    # robot._cmove(p0, p1, p2)

    '''
    robot._break()                  # 停顿

    c.ar, c.rr = 540, 0
    c.su, c.sv, c.sw = 0, 20, 40    # 靠近
    p3 = c.gen_world_n()

    robot._multipose_move([p3], 'LMOVE')
    robot._break()

    c.su, c.sv, c.sw = 0, 20, 0    # 靠近
    p4 = c.gen_world_n()

    c.ar, c.rr = 800, 0
    c.su, c.sv, c.sw = 0, 0, 0    # 全屏
    c.ay = 20
    p5 = c.gen_world_n()
    robot._multipose_move([p4, p5], 'LMOVE')
    '''

    robot._execute_project(waiting=True)

    robot.disconnect()


def tests_20():
    robot = Robot()
    robot.connect()

    robot.draw(10000)
    # robot.tdraw_ex(x=0, y=0, z=1, q=1)

    robot.disconnect()


def tests_21():
    robot = Robot()
    robot.connect()
    status = robot.get_status()
    print(status)
    where = robot.get_where()
    print(where)
    # where = robot.get_where(16)
    # print(where)
    # io = robot.get_io()
    # print(io)
    r = robot.get_switch()
    print(r)
    robot.disconnect()


def tests_22():
    robot = Robot()
    robot.connect()
    # robot.set_rep_once()
    robot.tdraw_ex(u=10)
    # robot.ereset()

    time.sleep(5)
    robot.stop()

    robot.disconnect()


def tests_23():
    robot = Robot()
    robot.connect()

    robot._edit_project('tests_19')

    poses = []
    c = Coord()
    c.place_object(600 + 730, 20, -50 - 200, 1, 50, 210)   # 物品的摆放位置及其尺寸

    c.ax = 200
    c.ar, c.rr = 800, 0                         # 相机到物品的距离
    c.su, c.sv, c.sw = 0, 10, -40   # 起点
    p0 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 0     # 中间点
    p1 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 40    # 终点
    p2 = c.gen_world_n()

    robot._cmove(p0, p1, p2)
    robot._execute_project()

    while robot.is_moving:
        print('moving...')
    print('finished')

    robot.disconnect()


def tests_24():
    robot = Robot()
    robot.connect()
    robot.draw_ex(w=-10)
    robot.disconnect()


def tests_25():
    robot = Robot()
    robot.connect()

    robot.set_progress(-1)
    robot._edit_project('tests_19')

    poses = []
    c = Coord()
    c.place_object(600 + 730, 20, -50 - 200, 1, 50, 210)   # 物品的摆放位置及其尺寸

    c.ax = 200
    c.ar, c.rr = 800, 0                         # 相机到物品的距离
    c.su, c.sv, c.sw = 0, 10, -40   # 起点
    p0 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 0     # 中间点
    p1 = c.gen_world_n()

    c.su, c.sv, c.sw = 0, 10, 40    # 终点
    p2 = c.gen_world_n()

    robot._cmove(p0, p1, p2)
    robot._execute_project()

    while True:
        p = robot.get_progress()
        print('pppppppppp:', p)
        if p == -2:
            break

    print('finished')
    robot.disconnect()


if __name__ == '__main__':
    # tests_3()
    # tests_8()
    # tests_10()
    # tests_11()
    # tests_12()
    # tests_13()
    # tests_14()
    # tests_15()
    # tests_16()
    # tests_17()
    # tests_18()
    # tests_19()
    # tests_20()
    # tests_21()
    # tests_22()
    # tests_23()
    tests_25()


'''
a = R.from_euler('zyz', (90, 180, -90), degrees=True).as_quat()
b = R.from_euler('zyz', (-180, 180, 0), degrees=True).as_quat()
c = R.from_euler('xyz', (180, 0, 0), degrees=True).as_quat()
print(a, b)
print(c)
'''
