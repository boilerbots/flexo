import enum
import array
import yaml
import serial
import time
import threading
import rclpy
from rclpy.node import Node
from ament_index_python import packages

def add_crc(data):
    checksum = sum(data)
    checksum %= 256
    return checksum

class Controller(Node):
    State = enum.Enum('State', ['IDLE', 'START1', 'START2', 'ADDRESS', 'LENGTH', 'DATA'])
    MAX_CHANNELS = 17

    def __init__(self):
        super().__init__('controller')
        self.l = self.get_logger()
        self.running = False
        self.declare_parameter('servo_config_file', 'servo_config.yaml')
        self.declare_parameter('pose_file', 'poses.yaml')
        self.declare_parameter('serial_device', '/dev/ttyUSB0')
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

        self.stand = self.poses['standing']
        self.current_position = [ 0 ] * 48  # enough for 24 channels


    def receiver(self):
        self.running = True
        state = self.State.IDLE
        address = 0
        type_code = None
        data_len = 0
        data = []

        while self.running:
            readbuf = self.ser.read(8)
            #print("received {} data while in state={}".format(len(data), state))
            for b in readbuf:
                if state == self.State.IDLE and b == 0xFF:
                    state = self.State.START1
                elif state == self.State.START1 and b == 0xFF:
                    state = self.State.START2
                elif state == self.State.START2:
                    address = b
                    state = self.State.ADDRESS
                elif state == self.State.ADDRESS:
                    expected_data_len = b
                    data_len = 0
                    data = []
                    state = self.State.DATA
                elif state == self.State.DATA:
                    data.append(b)
                    data_len += 1
                    if data_len >= expected_data_len:
                        state = self.State.IDLE
                        if data[0] == 0xA3:
                            l.info('servo {}  position={}'.format(address, data[1] * 256 + data[2]))
                            self.current_position[address * 2] = data[1]
                            self.current_position[address * 2 + 1] = data[2]

                        #if address == 0xFE:
                        #print('Captured addr={0:02x} len={1:02x} data={2}'.format(address, expected_data_len, data))
                        

                      
    def send(self, data):
        crc = add_crc(data)
        msg = [0xFF, 0xFF]
        msg += data
        msg.append(crc)
        self.ser.write(msg) 


    def connect(self):
        self.l.info('Opening {}'.format(self.serial_device))
        self.ser = serial.Serial(self.serial_device, 115200, timeout=1)
        self.receiver_thread = threading.Thread(target=self.receiver).start()

    def get_position(self):
        """ Readback all the current joint positions """
        channel = 0
        while channel < self.MAX_CHANNELS:
            # responses seem to respond with 0xAx so 0x03 => 0xA3
            test = [channel, 0x02, 0x03]
            self.send(test)
            time.sleep(0.005)
            channel += 1


    def controller(self):
        msg = [254, 50, 254]
        msg.extend(self.current_position)
        self.send(msg)

    def start(self):
        timer_period = 0.01
        self.send(self.initialize)
        time.sleep(0.002)
        self.send(self.disable)
        time.sleep(0.002)
        self.get_position()
        self.send(self.enable)
        time.sleep(0.002)
        self.timer = self.create_timer(timer_period, self.controller)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.connect()
    controller.start()

    rclpy.spin(controller)
    controller.running = False
    rclpy.shutdown()

if __name__ == '__main__':
    main()
