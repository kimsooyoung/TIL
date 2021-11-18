import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24             # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 3                 # Dynamixel#1 ID : 1
DXL2_ID                     = 4                 # Dynamixel#1 ID : 2

DXL3_ID                     = 6                 # Dynamixel#1 ID : 1
DXL4_ID                     = 7                 # Dynamixel#1 ID : 2

DXL5_ID                     = 9                 # Dynamixel#1 ID : 1
DXL6_ID                     = 10                 # Dynamixel#1 ID : 2

DXL7_ID                     = 1                 # Dynamixel#1 ID : 1
DXL8_ID                     = 8                 # Dynamixel#1 ID : 2

DXL9_ID                     = 2                 # Dynamixel#1 ID : 1
DXL10_ID                     = 5 
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque

class MinseoQuadrupped(Node):

    DXL1_MINIMUM_POSITION_VALUE  = 600           # Dynamixel will rotate between this value
    DXL1_MAXIMUM_POSITION_VALUE  = 700            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL2_MINIMUM_POSITION_VALUE  = 350           # Dynamixel will rotate between this value
    DXL2_MAXIMUM_POSITION_VALUE  = 450            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL3_MINIMUM_POSITION_VALUE  = 500          # Dynamixel will rotate between this value
    DXL3_MAXIMUM_POSITION_VALUE  = 400            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL4_MINIMUM_POSITION_VALUE  = 450           # Dynamixel will rotate between this value
    DXL4_MAXIMUM_POSITION_VALUE  = 350            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL5_MINIMUM_POSITION_VALUE  = 340           # Dynamixel will rotate between this value
    DXL5_MAXIMUM_POSITION_VALUE  = 440            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL6_MINIMUM_POSITION_VALUE  = 370           # Dynamixel will rotate between this value
    DXL6_MAXIMUM_POSITION_VALUE  = 470            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL7_MINIMUM_POSITION_VALUE  = 780           # Dynamixel will rotate between this value
    DXL7_MAXIMUM_POSITION_VALUE  = 680            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL8_MINIMUM_POSITION_VALUE  = 950           # Dynamixel will rotate between this value
    DXL8_MAXIMUM_POSITION_VALUE  = 850            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL9_MINIMUM_POSITION_VALUE  = 500           # Dynamixel will rotate between this value
    DXL9_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

    DXL10_MINIMUM_POSITION_VALUE  = 520           # Dynamixel will rotate between this value
    DXL10_MAXIMUM_POSITION_VALUE  = 520            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)


    DXL1_MOVING_STATUS_THRESHOLD = 50                # Dynamixel moving status threshold
    DXL2_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

    dxl1_goal_position = [DXL1_MINIMUM_POSITION_VALUE, DXL1_MAXIMUM_POSITION_VALUE]         # Goal position
    dxl2_goal_position = [DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE]         # Goal position

    dxl3_goal_position = [DXL3_MINIMUM_POSITION_VALUE, DXL3_MAXIMUM_POSITION_VALUE]         # Goal position
    dxl4_goal_position = [DXL4_MINIMUM_POSITION_VALUE, DXL4_MAXIMUM_POSITION_VALUE]         # Goal position

    dxl5_goal_position = [DXL5_MINIMUM_POSITION_VALUE, DXL5_MAXIMUM_POSITION_VALUE]         # Goal position
    dxl6_goal_position = [DXL6_MINIMUM_POSITION_VALUE, DXL6_MAXIMUM_POSITION_VALUE]         # Goal position

    dxl7_goal_position = [DXL7_MINIMUM_POSITION_VALUE, DXL7_MAXIMUM_POSITION_VALUE]         # Goal position
    dxl8_goal_position = [DXL8_MINIMUM_POSITION_VALUE, DXL8_MAXIMUM_POSITION_VALUE]         # Goal position

    dxl9_goal_position = [DXL9_MINIMUM_POSITION_VALUE, DXL9_MAXIMUM_POSITION_VALUE]         # Goal position
    dxl10_goal_position = [DXL10_MINIMUM_POSITION_VALUE, DXL10_MAXIMUM_POSITION_VALUE]         # Goal position


    def __init__(self):
        super().__init__('minseo_quadrupped_node')

        self.publisher = self.create_publisher(
            Twist, 'skidbot/cmd_vel', 10
        )  # queue size
        
        timer_period = 0.5  # seconds
        
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.get_logger().info(
            'DriveForward node Started, move forward during 5 seconds \n'
        )

        self.index = 0

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)
        self.openPort() 

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.setBaudrate()

        # Enable Dynamixel Torque
        self.dxl1_comm_result, self.dxl1_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl2_comm_result, self.dxl2_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl3_comm_result, self.dxl3_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl4_comm_result, self.dxl4_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        
        self.dxl5_comm_result, self.dxl5_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl6_comm_result, self.dxl6_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl7_comm_result, self.dxl7_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl8_comm_result, self.dxl8_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        
        self.dxl9_comm_result, self.dxl9_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dxl10_comm_result, self.dxl10_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL10_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    
        self.checkCommSuccess(self.dxl1_comm_result, self.dxl1_error)
        self.checkCommSuccess(self.dxl2_comm_result, self.dxl2_error)
        self.checkCommSuccess(self.dxl3_comm_result, self.dxl3_error)
        self.checkCommSuccess(self.dxl4_comm_result, self.dxl4_error)
        self.checkCommSuccess(self.dxl5_comm_result, self.dxl5_error)
        self.checkCommSuccess(self.dxl6_comm_result, self.dxl6_error)
        self.checkCommSuccess(self.dxl7_comm_result, self.dxl7_error)
        self.checkCommSuccess(self.dxl8_comm_result, self.dxl8_error)
        self.checkCommSuccess(self.dxl9_comm_result, self.dxl9_error)
        self.checkCommSuccess(self.dxl10_comm_result, self.dxl10_error)

    def checkCommSuccess(self, result, err):
        if result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(result))
        elif err != 0:
            print("%s" % self.packetHandler.getRxPacketError(err))
        else:
            print("Dynamixel has been successfully connected")

    def openPort(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def setBaudrate(self):
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def publish_callback(self):

        # Read present position
        dxl1_present_position, self.dxl1_comm_result, self.dxl1_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
        dxl2_present_position, self.dxl2_comm_result, self.dxl2_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
        dxl3_present_position, self.dxl3_comm_result, self.dxl3_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
        dxl4_present_position, self.dxl4_comm_result, self.dxl4_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION)
        dxl5_present_position, self.dxl5_comm_result, self.dxl5_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL5_ID, ADDR_MX_PRESENT_POSITION)
        dxl6_present_position, self.dxl6_comm_result, self.dxl6_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL6_ID, ADDR_MX_PRESENT_POSITION)
        dxl7_present_position, self.dxl7_comm_result, self.dxl7_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL7_ID, ADDR_MX_PRESENT_POSITION)
        dxl8_present_position, self.dxl8_comm_result, self.dxl8_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL8_ID, ADDR_MX_PRESENT_POSITION)
        dxl9_present_position, self.dxl9_comm_result, self.dxl9_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL9_ID, ADDR_MX_PRESENT_POSITION)
        dxl10_present_position, self.dxl10_comm_result, self.dxl10_error = self.packetHandler.read2ByteTxRx(self.portHandler, DXL10_ID, ADDR_MX_PRESENT_POSITION)
        
        self.checkCommSuccess(self.dxl1_comm_result, self.dxl1_error)
        self.checkCommSuccess(self.dxl2_comm_result, self.dxl2_error)
        self.checkCommSuccess(self.dxl3_comm_result, self.dxl3_error)
        self.checkCommSuccess(self.dxl4_comm_result, self.dxl4_error)
        self.checkCommSuccess(self.dxl5_comm_result, self.dxl5_error)
        self.checkCommSuccess(self.dxl6_comm_result, self.dxl6_error)
        self.checkCommSuccess(self.dxl7_comm_result, self.dxl7_error)
        self.checkCommSuccess(self.dxl8_comm_result, self.dxl8_error)
        self.checkCommSuccess(self.dxl9_comm_result, self.dxl9_error)
        self.checkCommSuccess(self.dxl10_comm_result, self.dxl10_error)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, self.dxl1_goal_position[self.index], self.dxl1_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL2_ID, self.dxl2_goal_position[self.index], self.dxl2_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL3_ID, self.dxl3_goal_position[self.index], self.dxl3_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL4_ID, self.dxl4_goal_position[self.index], self.dxl4_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL5_ID, self.dxl5_goal_position[self.index], self.dxl5_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL6_ID, self.dxl6_goal_position[self.index], self.dxl6_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL7_ID, self.dxl7_goal_position[self.index], self.dxl7_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL8_ID, self.dxl8_goal_position[self.index], self.dxl8_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL9_ID, self.dxl9_goal_position[self.index], self.dxl9_present_position))
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL10_ID, self.dxl10_goal_position[self.index], self.dxl10_present_position))

        self.exceptionHandler()

    def exceptionHandler(self, 
                dxl1_present_position, dxl2_present_position, dxl3_present_position,
                dxl4_present_position, dxl5_present_position, dxl6_present_position,
                dxl7_present_position, dxl8_present_position, dxl9_present_position,
                dxl10_present_position):

        if not abs(self.dxl1_goal_position[self.index] - dxl1_present_position) > self.DXL1_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl2_goal_position[self.index] - dxl2_present_position) > self.DXL2_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl3_goal_position[self.index] - dxl3_present_position) > self.DXL1_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl4_goal_position[self.index] - dxl4_present_position) > self.DXL2_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl5_goal_position[self.index] - dxl5_present_position) > self.DXL1_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl6_goal_position[self.index] - dxl6_present_position) > self.DXL2_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl7_goal_position[self.index] - dxl7_present_position) > self.DXL1_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl8_goal_position[self.index] - dxl8_present_position) > self.DXL2_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl9_goal_position[self.index] - dxl9_present_position) > self.DXL1_MOVING_STATUS_THRESHOLD:
            return
        if not abs(self.dxl10_goal_position[self.index] - dxl10_present_position) > self.DXL2_MOVING_STATUS_THRESHOLD:
            return

    def __del__(self):
        # Disable Dynamixel Torque
        self.dxl1_comm_result, self.dxl1_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl1_comm_result, self.dxl1_error)

        self.dxl2_comm_result, self.dxl2_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl2_comm_result, self.dxl2_error)

        self.dxl3_comm_result, self.dxl3_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl3_comm_result, self.dxl3_error)

        self.dxl4_comm_result, self.dxl4_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl4_comm_result, self.dxl4_error)

        self.dxl5_comm_result, self.dxl5_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl5_comm_result, self.dxl5_error)

        self.dxl6_comm_result, self.dxl6_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl6_comm_result, self.dxl6_error)

        self.dxl7_comm_result, self.dxl7_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl7_comm_result, self.dxl7_error)

        self.dxl8_comm_result, self.dxl8_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl8_comm_result, self.dxl8_error)

        self.dxl9_comm_result, self.dxl9_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl9_comm_result, self.dxl9_error)

        self.dxl10_comm_result, self.dxl10_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL10_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.checkCommSuccess(self.dxl10_comm_result, self.dxl10_error)

        # Close port
        self.portHandler.closePort()


def main(args=None):
    rclpy.init(args=args)

    minseo_quadrupped = MinseoQuadrupped()

    rclpy.spin(minseo_quadrupped)

    minseo_quadrupped.stop_robot()
    minseo_quadrupped.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()