from machine import Pin, ADC, PWM, I2C, SPI, TouchPad
from time import sleep, sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
import onewire
from ds18x20 import DS18X20
from neopixel import NeoPixel
from ssd1306 import SSD1306_I2C


# D15: Photoresistor (Light Sensor)
pin_light = Pin(32, Pin.IN)
adc_light = ADC(pin_light)


# D2: LED
pin_led = Pin(2, Pin.OUT)


# D4: Temperature sensor
pin_temp = Pin(4, Pin.IN)
ds_temp = DS18X20(onewire.OneWire(pin_temp))

def get_temp(wait_func=None):
    sensors = ds_temp.scan()
    ds_temp.convert_temp()

    # Sleep for 750ms
    start = ticks_ms()
    if wait_func:
        wait_func()
    end = ticks_ms()
    diff = ticks_diff(end, start)
    if diff > 0:
        sleep_ms(diff)

    # Read value
    temp = ds_temp.read_temp(sensors[0])
    return temp


# D26, D27, D13: Motor A
# D16, D17, D18: Motor B
pin_ma = Pin(26, Pin.OUT)
pin_ma1 = Pin(27, Pin.OUT)
pin_ma2 = Pin(12, Pin.OUT)
pin_mb = Pin(16, Pin.OUT)
pin_mb1 = Pin(17, Pin.OUT)
pin_mb2 = Pin(5, Pin.OUT)
pwm_ma = PWM(pin_ma, freq=100, duty=1023)
pwm_mb = PWM(pin_mb, freq=100, duty=1023)
def _set_speed(pwm, p1, p2, speed=None):
    if speed is None:
        p1.value(0)
        p2.value(0)
        pwm.duty(0)
        pwm.deinit()
        return

    if speed == 0:
        p1.value(0)
        p2.value(0)
        duty = 1023
    elif speed > 0:
        p1.value(0)
        p2.value(1)
        if speed > 1:
            speed = 1
        duty = int(speed * 1023)
    else:
        p1.value(1)
        p2.value(0)
        if speed < -1:
            speed = -1
        duty = int(-speed * 1023)
    pwm.init(freq=100, duty=duty)

def speed_a(speed):
    _set_speed(pwm_ma, pin_ma1, pin_ma2, speed)
def speed_b(speed):
    _set_speed(pwm_mb, pin_mb1, pin_mb2, speed)

# D5: LED Strip
ws_pin = Pin(32, Pin.OUT)
ws = NeoPixel(ws_pin, 8)


# D0: BOOT button
# D19: Button
boot_pin = Pin(0, Pin.IN)
btn_pin = Pin(19, Pin.IN)


# D21, D22: I2C (SDA, SCL)
i2c = I2C(0, sda=Pin(21), scl=Pin(22))
def oled_init():
    return SSD1306_I2C(width=128, height=64, i2c=i2c, addr=0x3c)


# D13: PWM (servo)
s0_pin = Pin(13, Pin.OUT)
s0_pwm = PWM(s0_pin, freq=50, duty=77)
def _set_servo_angle(pwm, angle):
    pwm.deinit()
    if angle == None:
        return

    if angle < -90:
        angle = 90
    if angle > 90:
        angle = 90
    pwm.init(freq=50, duty=int(80 + 50*angle//90))

def s0_angle(angle):
    _set_servo_angle(s0_pwm, angle)

# D13, D25: digits & additional PWM
s1_pin = Pin(14, Pin.OUT)
s1_pwm = PWM(s1_pin, freq=50, duty=77)
s1_pwm.deinit()

s2_pin = Pin(25, Pin.OUT)
s2_pwm = PWM(s2_pin, freq=50, duty=77)
s2_pwm.deinit()

def s1_angle(angle):
    _set_servo_angle(s1_pwm, angle)

def s2_angle(angle):
    _set_servo_angle(s2_pwm, angle)


# D18, D23, D19: vSPI (SCK, MOSI, MISO)
# D33: RCLK
spi = SPI(
    2, baudrate=3000000, polarity=0, phase=0, bits=8, firstbit=0,
    sck=Pin(18), mosi=Pin(23), miso=Pin(19)
)
rclk_pin = Pin(33, Pin.OUT, Pin.PULL_UP)
rclk_pin.value(0)

def sr_write(n):
    rclk_pin.value(0)
    spi.write(bytes([n]))
    rclk_pin.value(1)


# D36, D39: X/Y joystick
pin_x = Pin(36, Pin.IN)
pin_y = Pin(39, Pin.IN)
adc_x = ADC(pin_x)
adc_y = ADC(pin_y)
adc_x.atten(ADC.ATTN_11DB)
adc_y.atten(ADC.ATTN_11DB)


## D33: Touchpad  (won't work on v0.1)
#pin_touch = Pin(33, Pin.IN)
#touch = TouchPad(pin_touch)

