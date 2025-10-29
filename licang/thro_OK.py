
import sensor, image, time, math,pyb,struct
from pyb import UART,Servo,Pin,LED
from ws2812 import WS2812
from pid import PID

catch_disc_status = False
catch_stepped_status = False
catch_pillar_status = True
catch_pillar_dir_status = False
stack_chansfer_status = False
dir_up_status = False
stair_up_status = False
check_color_status = False
first = False
center_x = 0

disc_threshold_cold = (27, 88, -66, -22, -25, 60)#%OLD

disc_threshold_warm = (25, 91, -55, -17, -16, 45)


red_threshold = (41, 72, 30, 85, -33, 65)
red_threshold_night =(41, 72, 30, 85, -33, 65)


red_stair_threshold = (30, 82, -1, 68, 0, 73)#(30, 82, -4, 68, 17, 73)#(32, 86, 1, 66, 13, 68)#(41, 78, 13, 70, -7, 66)
red_stair_threshold2 =(36, 100, 8, 84, -15, 63)
red_pillar_dir_threshold = (42, 75, 13, 75, -9, 60)

red_pillar_threshold =(48, 83, 15, 82, -15, 62)

red_stack_threshold = (50, 65, 32, 106, -25, 47)
red_stack_threshold2 = (50, 65, 32, 106, -25, 47)


blue_threshold = (58, 93, -52, -15, -46, -4)
blue_threshold2 = (57, 100, -53, -16, -43, -6)

blue_stair_threshold = (57, 100, -53, -16, -43, -6)

blue_stair_threshold2 = (57, 100, -53, -16, -43, -6)


blue_pillar_dir_threshold = (49, 84, -49, -2, -54, -22)

blue_pillar_threshold = (65, 100, -53, 0, -34, -4)#(49, 84, -49, -2, -54, -22)
blue_pillar_threshold2 =(40, 100, -53, 2, -58, -8)
blue_stack_threshold =(44, 84, -40, 11, -51, -18)
blue_stack_threshold2 = (60, 93, -42, 2, -52, -19)

#--------------------old_threshold------------------------------
disc_New =(27, 88, -66, -22, -25, 60)

stack_threshold_list = [red_stack_threshold,red_stack_threshold2]
stair_threshold_list = [red_stair_threshold,red_stair_threshold2]
pillar_dir_threshold_list = [red_pillar_dir_threshold]
pillar_threshold_list = [red_pillar_threshold]
disc_threshold_list = [disc_threshold_warm,disc_threshold_cold]

Stack_ROI = [35, 30, 100, 60]#[35, 50, 100, 60]
Pillar_ROI = [0,10,150,100]
Disc_POI = [0,10,140,100]

led_red= pyb.LED(1)
led_green = pyb.LED(2)
led_blue = pyb.LED(3)
ring = WS2812(spi_bus=2, led_count=8)
data1 = [
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
    (0xff, 0xff, 0xff),
]
ring.show(data1)

data = [0x00,0x00,0x00,0x00,0x00]

def turnoffAllLED():
    led_red.off()
    led_green.off()
    led_blue.off()


hand = Servo(2)
hand.angle(-65)
time.sleep_ms(200)

def claw_catch():
    hand.angle(-65)

def claw_loose():
    hand.angle(-47)

def claw_loose2():
    hand.angle(-50)

def claw_loose3():
    hand.angle(30)

def claw_loose4():
    hand.angle(-20)

uart = UART(3,1000000)
uart.init(1000000, bits=8, parity=None, stop=1)
headbyte1 = 0x2C
headbyte2 = 0x12

headbyte_arm1 = 0x3C
headbyte_arm2 = 0x13

cmdbyte = 0x01
statusbyte = 0x00
statusbyte2 = 0x01
endbyte = 0x5B
buffer_size = 5

aim_threshold_list = [red_threshold,red_threshold_night]

def uart_receive():
    global uart
    global headbyte1
    global headbyte2
    global cmdbyte
    global statusbyte
    global endbyte

    global data

    global catch_disc_status
    global catch_stepped_status
    global catch_pillar_dir_status
    global catch_pillar_status
    global stack_chansfer_status
    global check_color_status

    global aim_threshold_list
    global stack_threshold_list
    global pillar_threshold_list
    global stair_threshold_list
    global pillar_dir_threshold_list

    global red_threshold
    global red_stack_threshold
    global red_pillar_threshold
    global red_stair_threshold
    global red_pillar_dir_threshold

    global blue_threshold
    global blue_threshold2
    global blue_stack_threshold
    global blue_pillar_threshold
    global blue_stair_threshold
    global blue_pillar_dir_threshold
    global blue_stair_threshold2
    global blue_pillar_threshold2
    global blue_stack_threshold2
    global red_threshold_night
    global red_stair_threshold2
    global red_stack_threshold2

    global disc_threshold
    '''
    #define CMD_BLMOD	0x01
    #define CMD_REMOD	0x02

    #define CMD_DISC	0x11
    #define CMD_STEP	0x21
    #define CMD_COLU	0x31
    #define CMD_STKTS	0x41

    #define CMD_CATH	0xF0
    #define CMD_LOSE	0xF1
    #define CMD_LOSE2	0xF2
    #define CMD_LOSE3	0xF3

    #define CMD_RESART	0xFF
    '''
    if uart.any():
        data = uart.read()
        if(data[0] == headbyte1 and data[1] == headbyte2 and data[3] == statusbyte and data[4] == endbyte):
            if(data[2] == 0x01):
                print("bluemod")
                aim_threshold_list = [blue_threshold,blue_threshold2]
                pillar_threshold_list = [blue_pillar_threshold2,blue_pillar_threshold]
                stack_threshold_list = [blue_stack_threshold,blue_stack_threshold2]
                stair_threshold_list = [blue_threshold,blue_threshold2]
            elif(data[2] == 0x02):
                print("redmod")
                sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0206, -5.49402, -4.0824))#red_white_bal
                aim_threshold_list = [red_threshold,red_threshold_night]
                pillar_threshold_list = [red_pillar_threshold]
                stack_threshold_list = [red_stack_threshold,red_stack_threshold2]
                stair_threshold_list = [red_threshold,red_threshold_night]
            elif(data[2] == 0x11):
                print("disc")
                catch_disc_status = True
            elif(data[2] == 0x21):
                catch_stepped_status = True
                aim_threshold_list = stair_threshold_list
            elif(data[2] == 0x31):
                catch_pillar_status = True
                aim_threshold_list = pillar_threshold_list
            elif(data[2] == 0x41):
                stack_chansfer_status = True
                aim_threshold_list = stack_threshold_list
            elif(data[2] == 0xF0):
                claw_catch()
            elif(data[2] == 0xF1):
                claw_loose()
            elif(data[2] == 0xF2):
                claw_loose2()
            elif(data[2] == 0xF3):
                claw_loose3()
            elif(data[2] == 0xF4):
                claw_loose4()
            else:
                uart_transmit_error()
        uart_reply()
        return

def uart_transmit(param):
    global uart
    global headbyte1
    global headbyte2
    global cmdbyte
    global statusbyte2
    global endbyte
    cmd = struct.pack("<BBBBB",
                       headbyte1,
                       headbyte2,
                       statusbyte2,
                       param,
                       endbyte
                       )
    print("out:")
    print(cmd[3])
    uart.write(cmd)

def uart_transmit2(angferq0,angferq1,angferq2,angferq3):
    global uart
    global headbyte1
    global headbyte2
    global cmdbyte
    global statusbyte2
    global endbyte
    cmd2 = struct.pack("<BBBHHHHB",
                       headbyte_arm1,
                       headbyte_arm2,
                       statusbyte2,
                       angferq0,
                       angferq1,
                       angferq2,
                       angferq3,
                       endbyte
                       )
    print("armout:")
    print(angferq0,angferq1,angferq2,angferq3)
    uart.write(cmd2)

def uart_reply():
    uart_transmit(0x01)

def uart_ball_distingushed():
    uart_transmit(0x02)

def uart_stack_NOT_distingushed():
    uart_transmit(0x03)

def uart_transmit_error():
    uart_transmit(0xFF)

def distingush_disc():
    global catch_disc_status
    if(catch_disc_status):
        claw_loose()
        tnow = pyb.millis()
        while(True):
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 12000):
                uart_stack_NOT_distingushed()
                catch_disc_status = False
                return
            blobs = img.find_blobs(aim_threshold_list,x_stride = 10,y_stride = 10,pixels_threshold= 4200,merge = True,margin=2)
            if blobs:
                max_b = find_max(blobs)
                img.draw_rectangle(max_b[0:4])
                img.draw_cross(max_b[5],max_b[6])
                if max_b.cy() > 65 and max_b.cy() < 78:             #TestOK
                    hand.angle(-65)
                    uart_ball_distingushed()
                    catch_disc_status = False
                    return
    return

def distingush_stairs():
    global catch_stepped_status
    global stair_up_status
    if(catch_stepped_status):
        claw_loose2()
        tnow = pyb.millis()
        while(True):
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 1000):
                uart_stack_NOT_distingushed()
                catch_stepped_status = False
                return
            blobs = img.find_blobs(aim_threshold_list,pixels_threshold=4200,merge = True,margin=2)
            if blobs:
                max_b = find_max(blobs)
                img.draw_rectangle(max_b[0:4])
                img.draw_cross(max_b[5],max_b[6])
                if max_b.cy() < 100 and max_b.cy() > 20:
                    print("阶梯识别成功")
                    uart_ball_distingushed()
                    catch_stepped_status = False
                    return
    return

def distingush_pillar():
    global catch_pillar_status
    if(catch_pillar_status):
        claw_loose4()
        tnow = pyb.millis()
        while(True):
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 25000):
                uart_stack_NOT_distingushed()
                catch_pillar_status = False
                return
            blobs = img.find_blobs(aim_threshold_list,x_stride = 10,y_stride = 10,pixels_threshold= 4000,merge = True,margin=2)
            if blobs:
                max_b = find_max(blobs)
                img.draw_rectangle(max_b[0:4])
                img.draw_cross(max_b[5],max_b[6])
                if max_b.cy() < 65 and max_b.cy() > 35:
                    print("目标达成！")
                    catch_pillar_status = False
                    return
    return


def distingush_stack():
    global stack_chansfer_status
    if(stack_chansfer_status):
        tnow = pyb.millis()
        while True:
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 1000):
                uart_stack_NOT_distingushed()
                stack_chansfer_status = False
                return
            blobs = img.find_blobs(aim_threshold_list,pixels_threshold=1100,merge = True,margin=2)
            if blobs:
                max_b = find_max(blobs)
                img.draw_rectangle(max_b[0:4])
                img.draw_cross(max_b[5],max_b[6])
                print("成功")
                uart_ball_distingushed()
                stack_chansfer_status = False
                return
    return
'''
def check_color():
    global check_color_status
    if(check_color_status):
        tnow = pyb.millis()
        while(True):
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 10000):
                uart_stack_NOT_distingushed()
                check_color_status = False
                return
            blobs = img.find_blobs([red_threshold,blue_disc2,b_dic],pixels_threshold=400,merge = True,margin=2)
            if blobs:
                max_b = find_max(blobs)
                check_color_status = False
                uart_ball_distingushed()
                return
    return
'''

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    return max_blob

claw_loose()
turnoffAllLED()
clock = time.clock()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
#print("Initial gain == %f db" % sensor.get_gain_db())
#print("Initial exposure == %d" % sensor.get_exposure_us())
#print(clock.fps(), \
#        sensor.get_rgb_gain_db())
sensor.set_auto_gain(False,gain_db = 2.361986)
sensor.set_auto_exposure(False, exposure_us = 6076)
sensor.set_auto_whitebal(False, rgb_gain_db = (-1.9722, -3.65861, -6.0206)) #blue whitebal
sensor.set_hmirror(True)
sensor.skip_frames(time = 2000)
while(True):
    clock.tick()
    uart_receive()
    distingush_disc()
    distingush_stairs()
    distingush_pillar()
    distingush_stack()

