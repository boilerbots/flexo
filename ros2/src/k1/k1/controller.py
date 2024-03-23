import enum
import math
import array
import yaml
import serial
import time
import threading
from threading import Lock
import copy
import rclpy
from rclpy.node import Node
from ament_index_python import packages
from rclpy.callback_groups import ReentrantCallbackGroup
from k1_interfaces.msg import Status, Joints
from k1_interfaces.srv import JointControl

def add_crc(data):
    checksum = sum(data)
    checksum %= 256
    return checksum

def crc_good(data):
    """ Return True if checksum is good """
    checksum = sum(data[:-1])
    checksum %= 256
    return checksum == data[-1]


class Controller(Node):
    State = enum.Enum('State', ['IDLE', 'START1', 'START2', 'ADDRESS', 'LENGTH', 'DATA'])
    REAL_CHANNELS = 17
    MAX_CHANNELS = 24

    def __init__(self):
        super().__init__('controller')
        self.l = self.get_logger()
        self.timer_period = 100 # in ms
        self.running = False
        self.msg_mutex = Lock()
        self.declare_parameter('servo_config_file', 'servo_config.yaml')
        self.declare_parameter('pose_file', 'poses.yaml')
        self.declare_parameter('serial_device', '/dev/ttyAMA0')
        self.serial_device = self.get_parameter('serial_device').get_parameter_value().string_value

        servo_config_file = self.get_parameter('servo_config_file').get_parameter_value().string_value
        pose_file = self.get_parameter('pose_file').get_parameter_value().string_value

        package_path = packages.get_package_share_directory('k1')
        self.l.info('servo_config_file: ' + servo_config_file)
        with open(package_path + '/config/' + servo_config_file, 'r') as f:
            self.servo_config = yaml.full_load(f)
        self.l.info('pose_file: ' + pose_file)
        with open(package_path + '/config/' + pose_file, 'r') as f:
            self.poses = yaml.full_load(f)

        self.initialize = self.servo_config['initialize']
        self.enable = self.servo_config['enable']
        self.disable = self.servo_config['disable']

        # In 16bit user space values
        self.stand = self.poses['standing'] # our zero reference

        # Next goal
        self.goal_ticks = 0  # count down dt
        self.joint_inc = [ 0 ] * self.MAX_CHANNELS
        self.goal = [ 0 ] * self.MAX_CHANNELS
        self.current_position = [ 0 ] * self.MAX_CHANNELS  # current postion

        # In 2 x 8bit hardware values
        self.hw_val = [ 0 ] * 2 * self.MAX_CHANNELS   # current message we are sending

        self.callback_group = ReentrantCallbackGroup()
        self.pub_status = self.create_publisher(Status, '~/status', 10)
        self.sub = self.create_subscription(Joints, '~/cmd', self.cmd_callback, 10, callback_group=self.callback_group)
        self.srv_joints = self.create_service(JointControl, '~/control', self.control_callback, callback_group=self.callback_group)

    def compute_hwval(self):
        for ch in range(self.MAX_CHANNELS):
            self.hw_val[ch * 2] = int(self.current_position[ch] / 256)  # integer division
            self.hw_val[ch * 2 + 1] = int(self.current_position[ch] % 256)
        #self.l.info('compute_hwval: {}'.format(self.hw_val))

    def set_goal(self, goal, dt, synchronous):
        if synchronous:  # calculate step sizes
            step_count = math.floor(dt / self.timer_period)
            if step_count == 0:
                self.l.warning('step_count=' + step_count)
            delta = [goal[x] - self.current_position[x] for x in range(len(goal))]
            self.joint_inc = [delta[x] / step_count for x in range(len(goal))]
            self.goal_ticks = step_count

            max_delta = max(delta)
            min_delta = min(delta)
            #max_change = max(abs(max_delta), abs(min_delta)))
            self.l.info('set_goal min_delta={} max_delta={} step_count={}'.format(
                min_delta, max_delta, step_count))
        else:
            self.current_position = goal
            self.compute_hwval()
        self.goal = goal

    def control_callback(self, control, response):
        self.l.info('Service request: ch={} enable={} home={}'.format(
            control.channel, 
            control.enable,
            control.home))
        response.position = 0
        if control.home:
            self.set_goal(self.stand, 3000, True)

        return response

    def cmd_callback(self, msg):
        self.l.info('next pose: dt={}'.format(msg.dt))
        self.set_goal(msg.position, msg.dt, msg.synchronous)

    def receiver(self):
        state = self.State.IDLE
        address = 0
        type_code = None
        data_len = 0
        data = []

        self.running = True
        while self.running:
            readbuf = self.ser.read(8)
            #self.l.info("received {} data while in state={}".format(len(data), state))
            for b in readbuf:
                if state == self.State.IDLE and b == 0xFF:
                    state = self.State.START1
                elif state == self.State.START1 and b == 0xFF:
                    state = self.State.START2
                elif state == self.State.START2:
                    address = b
                    data = [address]
                    state = self.State.ADDRESS
                elif state == self.State.ADDRESS:
                    expected_data_len = b
                    data.append(expected_data_len)
                    data_len = 0  # does count itself but CRC does
                    state = self.State.DATA
                elif state == self.State.DATA:
                    data.append(b)
                    data_len += 1
                    if data_len == expected_data_len:
                        state = self.State.IDLE
                        if not crc_good(data):
                            self.l.warn('Bad CRC')
                            continue
                        if data[2] == 0xA3:
                            position = data[3] * 256 + data[4]
                            self.l.info('servo {}  position={}'.format(address, position))
                            self.current_position[address] = position
                            #self.l.info('Captured addr={0:02x} len={1:02x} data={2}'.format(address, data_len, data))


    def status_publisher(self):
        status = Status()
        self.pub_status.publish(status)
                        
    def send(self, data):
        crc = add_crc(data)
        msg = [0xFF, 0xFF]
        msg += data
        msg.append(crc)
        #self.l.info('sending: {}'.format(msg))
        self.ser.write(msg) 

    def connect(self):
        self.l.info('Opening {}'.format(self.serial_device))
        self.ser = serial.Serial(self.serial_device, 115200, timeout=1, rtscts=True)
        self.receiver_thread = threading.Thread(target=self.receiver).start()

    def get_position(self):
        """ Readback all the current joint positions """
        channel = 0
        while channel < self.REAL_CHANNELS:
            # responses seem to respond with 0xAx so 0x03 => 0xA3
            test = [channel, 0x02, 0x03]
            self.send(test)
            time.sleep(0.002)
            channel += 1

    def enable_all(self):
        """ Enable 1 at a time """
        channel = 0
        while channel < self.MAX_CHANNELS:
            test = [channel, 3, 2, 1]
            self.send(test)
            time.sleep(0.002)
            channel += 1


    def controller(self):
        """ Run on a periodic timer sending position to servos. """
        msg = [254, 50, 254]
        if self.goal_ticks > 0:
            self.l.info('moving ticks={}'.format(self.goal_ticks))
            for x in range(self.MAX_CHANNELS):
                if self.current_position[x] != self.goal[x]:
                    self.current_position[x] = self.current_position[x] + self.joint_inc[x]
            self.compute_hwval()
            self.goal_ticks -= 1

        msg.extend(self.hw_val)
        self.send(msg)

    def start(self):
        while not self.running:
            time.sleep(0.5)
        self.send(self.initialize)
        time.sleep(0.302)
        #self.send(self.disable)
        #time.sleep(0.002)
        #self.enable_all()
        self.send(self.enable)
        time.sleep(0.002)
        for x in range(10):
            self.get_position()
        #self.compute_hwval()
        self.set_goal(self.stand, 3000, True)
        self.timer = self.create_timer(self.timer_period / 1000, self.controller)


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    controller = Controller()
    executor.add_node(controller)
    controller.connect()
    controller.start()

    executor.spin()
    controller.running = False
    rclpy.shutdown()

if __name__ == '__main__':
    main()
