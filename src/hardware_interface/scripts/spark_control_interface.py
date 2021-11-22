import rospy
import serial
from threading import Thread

class SparkControlInterface:
    def __init__(self, drive_motor_port="/dev/ttyACM0", steer_port="/dev/ttyUSB0"):
        try:
            self.drive_motor_serial = serial.Serial(port=drive_motor_port, baudrate=115200) # raspberry for motor control 
            self.steer_serial = serial.Serial(port=steer_port, baudrate=9600) # arduino for steer and break control
        except serial.serialutil.SerialException as e:
            rospy.logerr("Ports are not opened: %s" % e)
            rospy.signal_shutdown("ports_closed")
        self.end_serial_loop = False
        self.steer_angle = 0.0 
        
        # self.thread_check_serial = Thread(target=self.check_steer_serial)
        # self.thread_check_serial.start()


    def command_vehicle(self, cmd_msg):
        command_str_drive = "<"
        command_str_drive += str(cmd_msg.speed) + ","
        if len(cmd_msg.modes) != 0:
            for mode in cmd_msg.modes:
                if mode.data == "autonomous":
                    continue
                command_str_drive += mode.data[0] + ","

        command_str_drive = command_str_drive[:-1]
        command_str_drive += ">"
        command_str_steer = "<{}>".format(int(min(max(cmd_msg.steer, -100), 100)/2))
        # OPEN/CLOSE LOGGING WHAT IS WRITTEN TO ARDUINOS
        print(command_str_drive)
        print(command_str_steer)
        self.drive_motor_serial.write(bytes(command_str_drive, "utf-8"))
        self.steer_serial.write(bytes(command_str_steer, "ascii"))
    
    def check_steer_serial(self):
        serial_string_wheel = ""
        while True:
            if self.end_serial_loop:
                break
            if self.steer_serial.in_waiting > 0:
                serial_string_steer = self.steer_serial.readline().decode("ascii").strip()
                self.steer_serial.flushInput()
                try:
                    self.steer_angle = float(serial_string_steer)
                except:
                    rospy.logwarn("SPARK Control Interface: Cannot convert steer angle message to float!")