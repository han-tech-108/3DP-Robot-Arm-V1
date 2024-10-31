from machine import Pin, PWM
import bluetooth
from ble_simple_peripheral import BLESimplePeripheral
from machine import Timer
import micropython

# Setup to create exceptions during interrupt processing
micropython.alloc_emergency_exception_buf(100)
TIMER_FREQ = 500


class ServoControl:
    DUTY_MAX = 65535
    FREQ = 50  #Hz
    FREQ_TIME = 20  # ms
    # SG90_MIN = 0.5
    # SG90_CENTOR = 1.5
    # SG90_MAX = 2.5
    ANGLE_0 = int(0.5 / FREQ_TIME * DUTY_MAX)
    ANGLE_45 = int(1.0 / FREQ_TIME * DUTY_MAX)
    ANGLE_90 = int(1.5 / FREQ_TIME * DUTY_MAX)
    ANGLE_135 = int(2.0 / FREQ_TIME * DUTY_MAX)
    ANGLE_180 = int(2.5 / FREQ_TIME * DUTY_MAX)

    def __init__(self):
        self.ble = bluetooth.BLE()  # Bluetooth Low Energy (BLE) object creation
        self.sp = BLESimplePeripheral(self.ble)  # Instantiation of the BLESimplePeripheral class
        self.led = Pin("LED", Pin.OUT) 
        
        self.servo_base = PWM(Pin(2))
        self.servo_base.freq(self.FREQ)

        self.servo_front_rear = PWM(Pin(3))
        self.servo_front_rear.freq(self.FREQ)

        self.servo_up_down = PWM(Pin(12))
        self.servo_up_down.freq(self.FREQ)

        self.servo_catch = PWM(Pin(13))
        self.servo_catch.freq(self.FREQ)

        self.servo_init = PWM(Pin(22))
        self.servo_init.freq(self.FREQ)

        self.servo_base_flag = 0
        self.servo_front_rear_flag = 0
        self.servo_up_down_flag = 0
        self.servo_catch_flag = 0
        
        # Servo position initialization
        self.servo_base_angle = self.ANGLE_90
        self.servo_front_rear_angle = self.ANGLE_90
        self.servo_up_down_angle = self.ANGLE_45
        self.servo_catch_angle = self.ANGLE_90

        self.servo_base.duty_u16(self.servo_base_angle)  
        self.servo_front_rear.duty_u16(self.servo_front_rear_angle)
        self.servo_up_down.duty_u16(self.servo_up_down_angle) 
        self.servo_catch.duty_u16(self.servo_catch_angle)
        self.servo_init.duty_u16(self.ANGLE_0)

        print(f"{self.servo_base_angle}, {self.servo_front_rear_angle}, {self.servo_up_down_angle}, {self.servo_catch_angle}")

    def on_rx(self, data):
        """
        Callback function to process received data
        """
        print("Data received: ", data)            
        if data in [b'f', b'l', b'r', b'u', b'd', b're', b'c']:
            self.led.value(True)
            self.sp.send("ON\n")
        elif data == b's':
            self.led.value(False)
            self.sp.send("OFF\n")

        # Servo Drive Flag
        if data == b'l':
            self.servo_base_flag = 1
        elif data == b'r':
            self.servo_base_flag = -1
        
        if data == b'f':
            self.servo_front_rear_flag = 1
        elif data == b're':
            self.servo_front_rear_flag = -1

        if data == b'u':
            self.servo_up_down_flag = 1
        elif data == b'd':
            self.servo_up_down_flag = -1

        if data == b'c':
            self.servo_catch_flag = 1
        
        if data == b's':
            self.servo_base_flag = 0
            self.servo_front_rear_flag = 0
            self.servo_up_down_flag = 0

    def servo_task(self):
        # Servo position update
        if self.servo_base_flag == 1 and self.servo_base_angle < self.ANGLE_180:
                self.servo_base_angle = self.servo_base_angle + 1
        elif self.servo_base_flag == -1 and self.ANGLE_0 < self.servo_base_angle:
                self.servo_base_angle = self.servo_base_angle - 1                

        if self.servo_front_rear_flag == 1 and self.servo_front_rear_angle < self.ANGLE_180:
            self.servo_front_rear_angle = self.servo_front_rear_angle + 1
        elif self.servo_front_rear_flag == -1 and self.ANGLE_0 < self.servo_front_rear_angle:
            self.servo_front_rear_angle = self.servo_front_rear_angle - 1
        
        if self.servo_up_down_flag == 1 and self.servo_up_down_angle < self.ANGLE_180:
            self.servo_up_down_angle = self.servo_up_down_angle + 1
        elif self.servo_up_down_flag == -1 and self.ANGLE_0 < self.servo_up_down_angle:
            self.servo_up_down_angle = self.servo_up_down_angle - 1
        
        if self.servo_catch_flag == 1:
            if self.servo_catch_angle == self.ANGLE_180:
                self.servo_catch_angle = self.ANGLE_90
            elif self.servo_catch_angle == self.ANGLE_90:
                self.servo_catch_angle = self.ANGLE_180
            self.servo_catch_flag = 0
        
        self.servo_base.duty_u16(self.servo_base_angle)
        self.servo_front_rear.duty_u16(self.servo_front_rear_angle)
        self.servo_up_down.duty_u16(self.servo_up_down_angle)
        self.servo_catch.duty_u16(self.servo_catch_angle)

    def loop(self, timer):
        if self.sp.is_connected():  # Check if BLE connection is established
            self.sp.on_write(self.on_rx)  # Set callback function for data reception 
        self.servo_task()   
    

def main():
    sc_agent = ServoControl()    
    timer = Timer()
    timer.init(mode=Timer.PERIODIC, freq=TIMER_FREQ, callback=sc_agent.loop)  # timer interrupt


if __name__ == "__main__":
    main()
