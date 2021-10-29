import socket
import json
import time

from scipy.spatial.transform import Rotation as R

import kawasaki_robot


host, port = '172.16.44.147', 88
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((host, port))


def get_phone_uvw(sock):
    sock.send(b'h')
    # data = b''
    while True:
        # data = data + sock.recv(1024)
        data = sock.recv(1024)
        try:
            data = json.loads(data.decode('utf-8'))
            break
        except:
            print(data)
    uvw = data['attitude']
    print('phone.uvw:', uvw)
    uvw = R.from_euler('xyz', uvw)
    return uvw, data['immediately']


haha = True
robot = kawasaki_robot.Robot()
if haha:
    robot.connect()

status = 0    # 0 初始， 1 转动， 2 稳定

while True:
    print('listen(1)')
    sock.listen(1)
    client_sock, client_address = sock.accept()
    print('accept:', client_address)

    if haha:
        pose = [1000, 0, -100, 0, 0, 0]
        robot._edit_project('haha')
        robot._move('JMOVE', pose)
        robot._execute_project(True)
    robot_uvw = R.from_euler('xyz', [0, 0, 0], degrees=True)
    phone_uvw, _ = get_phone_uvw(client_sock)
    t_uvw = robot_uvw * phone_uvw.inv()

    while True:
        # client's request
        p_uvw, immediately = get_phone_uvw(client_sock)
        t = (t_uvw * p_uvw) * robot_uvw.inv()
        if not immediately and R.magnitude(t) > 20 / 180 * 3.14:
            status = 1
            robot_uvw = t_uvw * p_uvw
        elif not immediately and 0 < status and status < 10:
            status += 1
        elif immediately or status == 10:
            robot_uvw = t_uvw * p_uvw
            print(robot_uvw.as_euler('xyz', degrees=True), p_uvw.as_euler('xyz', degrees=True))
            status = 0
            if haha and robot.get_progress() < 0:
                pose = robot.world_n
                pose[3:] = robot_uvw.as_euler('xyz', degrees=True)
                pose[4] = -pose[4]

                robot.set_progress(0)
                robot._edit_project('haha')
                robot._move('JMOVE', pose)
                robot._execute_project()

        time.sleep(0.1)

    client_sock.close()
