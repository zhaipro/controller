
import json
import socket

import console
import dialogs
import motion
import ui


@ui.in_background
def message_box(title, message):
  console.alert(title, message)


class MyView(ui.View):

  def __init__(self):
    self.px, self.py, self.pz = 0, 0, 0
    self.vx, self.vy, self.vz = 0, 0, 0
    self.ax, self.ay, self.az = 0, 0, 0
    self.i = 0

    self.max_a = 0
    self.max_v = 0

  def did_load(self):
    motion.start_updates()
    self.update_interval = 0.01

  def do_connect(self):
    mainview = self.superview
    ip = mainview['ip_textfield'].text
    button = mainview['connect_button']
    try:
      host, port = ip, 88
      mainview.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      mainview.sock.connect((host, port))
      button.enabled = False
    except:
      # button.enabled = True
      message_box('异常', '连接机械臂失败！')

  def will_close(self):
    print(self.max_a, self.max_v)
    motion.stop_updates()
    if hasattr(self, 'sock'):
      self.sock.close()

  def exe(self, immediately=True):
    attitude = motion.get_attitude()
    data = {
      'attitude': attitude,
      'immediately': immediately,
    }
    data = json.dumps(data).encode('utf-8')
    if not hasattr(self, 'sock'):
      self.superview.sock.send(data)
    else:
      self.sock.send(data)

  def update(self):
    t = 0.01
    ax, ay, az = motion.get_user_acceleration()

    vx = self.vx + (ax + self.ax) / 2 * t
    vy = self.vy + (ay + self.ay) / 2 * t
    vz = self.vz + (az + self.az) / 2 * t

    a = (ax ** 2 + ay ** 2 + az ** 2) ** 0.5
    v = (vx ** 2 + vy ** 2 + vz ** 2) ** 0.5
    if a < 0.1:
      if v < 0.15 * t:
        vx, vy, vz = 0, 0, 0
      else:
        q = (v - 0.15 * t) / v
        vx, vy, vz = q * vx, q * vy, q * vz

    self.px = self.px + (vx + self.vx) / 2 * t
    self.py = self.py + (vy + self.vy) / 2 * t
    self.pz = self.pz + (vz + self.vz) / 2 * t

    self.vx = vx
    self.vy = vy
    self.vz = vz

    self.ax = ax
    self.ay = ay
    self.az = az

    self.max_a = max(self.max_a, a)
    self.max_v = max(self.max_v, v)

    self.i += 1
    if self.i % 10 == 0:
      self['stdout'].text = f'{self.px:.2f}, {self.py:.2f}, {self.pz:.2f}'
      self['label4'].text = f'{v:.2f}, {a:.2f}'

    a = self['switch1'].value
    if a:
      self.exe(immediately=False)

  def auto(self):
    a = self.superview['switch1'].value
    self.superview['button1'].enabled = not a

v = ui.load_view()
v.present('fullscreen')
