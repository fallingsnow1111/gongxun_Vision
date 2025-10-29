# 立桩处蓝球看不到是像素阈值和白平衡共同作用
# 红蓝球立桩与仓库处都有各自的白平衡
import sensor, image, time, pyb, math, struct
from pyb import Servo,UART
from ws2812 import WS2812 #灯

redmode_status = False
bluemode_status = False

catch_disc_status = False
catch_stepped_status = False
catch_pillar_status = False
stack_chansfer_status = False
catch_pillar_dir_status = False

# ===== 颜色阈值 =====
red_disc_threshold = (46, 69, 49, 80, -19, 59)
red_stair_threshold = (46, 74, 38, 80, -24, 54)
red_pillar_threshold = (55, 79, 48, 78, -20, 54)
red_stack_threshold_day = (35, 60, 47, 83, 23, 64)
red_stack_threshold_night = (43, 57, 57, 80, 20, 64)

blue_disc_threshold = (66, 96, -31, 8, -40, -6) # (58, 91, -26, 15, -60, -10)
blue_stair_threshold = (73, 96, -28, 9, -43, -3)
blue_pillar_threshold =(57, 89, -28, 7, -43, -4) #(55, 97, -25, 11, -45, 0)
blue_stack_threshold_night = (35, 64, -29, 8, -41, -13) # 夜晚阈值
blue_stack_threshold_day = (32, 70, -31, 4, -40, -9) #  白天阈值

disc_threshold_list = [blue_disc_threshold]
stair_threshold_list = [blue_stair_threshold]
pillar_threshold_list = [blue_pillar_threshold]
stack_threshold_list = [blue_stack_threshold_day,blue_stack_threshold_night]

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

# ===== 舵机（夹爪） =====
# 不同的loose代表不同位置的夹球任务
hand = Servo(2)
def claw_catch():
    hand.angle(-65)   # 夹紧

def claw_loose():
    hand.angle(-40)   # 松开

def claw_loose2():
    hand.angle(-35)

def claw_loose3():
    hand.angle(30)

def claw_loose4():
    hand.angle(-20)

# 串口初始化
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

def uart_receive():
    global uart
    global headbyte1
    global headbyte2
    global cmdbyte
    global statusbyte
    global endbyte

    global data

    global redmode_status, bluemode_status

    global catch_disc_status
    global catch_stepped_status
    global catch_pillar_status
    global stack_chansfer_status

    global aim_threshold_list
    global stack_threshold_list
    global pillar_threshold_list
    global stair_threshold_list

    global red_disc_threshold
    global red_stair_threshold
    global red_pillar_threshold
    global red_stack_threshold

    global blue_disc_threshold
    global blue_stair_threshold
    global blue_pillar_threshold
    global blue_stack_threshold

    if uart.any():
        data = uart.read()
        if(data[0] == headbyte1 and data[1] == headbyte2 and data[3] == statusbyte and data[4] == endbyte):
            if(data[2] == 0x01):
                print("bluemod")
                bluemode_status = True
                redmode_status = False
                sensor.set_auto_gain(False,gain_db = 5.74483)      # 自动增益
                sensor.set_auto_exposure(False, exposure_us = 3048) # 曝光
                sensor.set_auto_whitebal(False, rgb_gain_db = (3.61183, -4.0824, -6.0206))
                aim_threshold_list = [blue_disc_threshold]
                stair_threshold_list = [blue_stair_threshold]
                pillar_threshold_list = [blue_pillar_threshold]
                stack_threshold_list = [blue_stack_threshold_day,blue_stack_threshold_night]
            elif(data[2] == 0x02):
                print("redmod")
                redmode_status = True
                bluemode_status = False
                sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0206, -5.49402, -4.0824))#red_white_bal
                aim_threshold_list = [red_disc_threshold]
                stair_threshold_list = [red_stair_threshold]
                pillar_threshold_list = [red_pillar_threshold]
                stack_threshold_list = [red_stack_threshold_day,red_stack_threshold_night]
            elif(data[2] == 0x11):
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
    # print("out:")
    # print(cmd[3])
    uart.write(cmd)

def uart_transmit2(angferq0,angferq1,angferq2,angferq3):
    global uart
    global headbyte1
    global headbyte2
    global cmdbyte
    global statusbyte2
    global endbyte
    # H是两个字节
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

    if catch_disc_status:
        claw_loose()
        if(bluemode_status):
            sensor.set_auto_gain(False,gain_db = 5.74483)      # 自动增益
            sensor.set_auto_exposure(False, exposure_us = 3048) # 曝光
            sensor.set_auto_whitebal(False, rgb_gain_db = (3.61183, -4.0824, -6.0206))
        else:
            sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0206, -5.49402, -4.0824))#red_white_bal
        tnow = pyb.millis()
        while True:
            img = sensor.snapshot()
            # 超时保护
            if (pyb.millis() - tnow > 12000):
                uart_stack_NOT_distingushed()
                catch_disc_status = False
                return
            # ===== 加快识别的参数 =====
            blobs = img.find_blobs(aim_threshold_list,
                                   pixels_threshold=100,   # 降低像素阈值
                                   area_threshold=100,     # 降低面积阈值
                                   merge=True)             # 合并相邻色块
            if blobs:
                max_b = find_max(blobs)
                if max_b:
                    cx, cy = max_b.cx(), max_b.cy()
                    # 画ROI和中心点
                    img.draw_rectangle(max_b.rect(), color=(255, 0, 0))
                    img.draw_cross(cx, cy, color=(0, 255, 0))
                    # ===== 判断是否在中心区域 =====
                    if 65 <= cy <= 90:  # ROI条件
                        claw_catch()
                        print('catch at (%d,%d)' % (cx,cy))
                        uart_ball_distingushed()
                        catch_disc_status = False
                        return

def distingush_stairs():
    global catch_stepped_status

    if(catch_stepped_status):
        claw_loose2()
        tnow = pyb.millis()
        while(True):
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 1000):
                uart_stack_NOT_distingushed()
                catch_stepped_status = False
                return

            blobs = img.find_blobs(aim_threshold_list,
                                   pixels_threshold=100,   # 降低像素阈值
                                   area_threshold=100,     # 降低面积阈值
                                   merge=True)             # 合并相邻色块
            if blobs:
                max_b = find_max(blobs)
                if max_b:
                    cx, cy = max_b.cx(), max_b.cy()
                    img.draw_rectangle(max_b.rect(), color=(255, 0, 0))
                    img.draw_cross(cx, cy, color=(0, 255, 0))
                    if 20 < cy < 100:  # ROI条件
                        claw_catch()
                        print("catch")
                        uart_ball_distingushed()
                        catch_stepped_status = False
                        return

def distingush_pillar():
    global catch_pillar_status
    if(catch_pillar_status):
        claw_loose4()
        if(redmode_status):
            sensor.set_auto_gain(False,gain_db = 0)      # 自动增益
            sensor.set_auto_exposure(False, exposure_us = 2136) # 曝光
            sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0206, 1.99232, 4.50457))
        else:
            sensor.set_auto_gain(False,gain_db = 0)      # 自动增益
            sensor.set_auto_exposure(False, exposure_us = 2256) # 曝光
            sensor.set_auto_whitebal(False, rgb_gain_db = (5.9866, -1.63808, -6.0206))

        tnow = pyb.millis()
        while(True):
            img = sensor.snapshot()
            if(pyb.millis()-tnow > 10000):
                catch_pillar_status = False
                uart_stack_NOT_distingushed()
                return

            if(bluemode_status):
                blobs = img.find_blobs(aim_threshold_list,x_stride = 10,y_stride = 10,pixels_threshold= 3000,merge = True,margin=2)
            else:
                blobs = img.find_blobs(aim_threshold_list,x_stride = 10,y_stride = 10,pixels_threshold= 4000,merge = True,margin=2)

            if(blobs):
                max_b = find_max(blobs)
                ratio = max_b.w() / max_b.h()
                if 0.8 < ratio < 1.2:
                    cx, cy= max_b.cx(),  max_b.cy()
                    img.draw_rectangle(max_b.rect(), color=(0,255,0))
                    img.draw_cross(cx,cy, color=(0,255,0))
                    if 35 < cy < 80:
                        uart_ball_distingushed()
                        print('catch at (%d,%d)' % (cx,cy))
                        catch_pillar_status = False
                        return

def distingush_stack():
    global stack_chansfer_status
    global redmode_status, bluemode_status
    if(stack_chansfer_status):
        if(redmode_status):
            sensor.set_auto_whitebal(False, rgb_gain_db = (-5.24224, -3.86792, -6.0206))
        else:
            # 仓库蓝球参数组
            sensor.set_auto_gain(False,gain_db = 1.02305)      # 自动增益
            sensor.set_auto_exposure(False, exposure_us = 3048) # 曝光
            sensor.set_auto_whitebal(False, rgb_gain_db = (-1.39567, -2.68164, -6.0206))     # blue_whitebal
        tnow = pyb.millis()
        while True:
            img = sensor.snapshot()
            if (pyb.millis()-tnow > 1500):
                uart_stack_NOT_distingushed()
                stack_chansfer_status = False
                return
            blobs = img.find_blobs(aim_threshold_list,pixels_threshold=200,merge = True,margin=2)
            if blobs:
                max_b = find_max(blobs)
                img.draw_rectangle(max_b[0:4])
                img.draw_cross(max_b[5],max_b[6])
                print("成功")
                uart_ball_distingushed()
                stack_chansfer_status = False
                return
    return

def find_max(blobs):
    max_size = 0
    max_blob = None
    for blob in blobs:
        size = blob.w() * blob.h()
        if size > max_size:
            max_blob = blob
            max_size = size
    return max_blob

# 到现场调用这个函数确定参数后锁定
def get_param():
    # Step 1：自动调整，让摄像头适应当前环境
    sensor.set_auto_whitebal(True)
    sensor.set_auto_gain(True)
    sensor.set_auto_exposure(True)
    sensor.skip_frames(time=2000)

    # Step 2：读取当前状态
    gain_db = sensor.get_gain_db()
    exposure_us = sensor.get_exposure_us()
    rgb_gain_db = sensor.get_rgb_gain_db()
    print("Calibrated values:")
    print("gain_db:", gain_db)
    print("exposure_us:", exposure_us)
    print("rgb_gain_db:", rgb_gain_db)

    # Step 3：锁定所有参数
    sensor.set_auto_gain(False, gain_db = gain_db)
    sensor.set_auto_exposure(False, exposure_us = exposure_us)
    sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain_db)
    sensor.skip_frames(time=500)

    return

blue_gain_db_day = 1.02305
blue_exposure_day = 3048
blue_rgb_gain_day = (-1.39567, -2.68164, -6.0206)

# 开始时先松开
claw_loose()
turnoffAllLED()
clock = time.clock()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_hmirror(True)    # 先确定镜像方向再设置白平衡

sensor.skip_frames(time=2000)  #开启自动白平衡的时候，让mv适应光线两秒
sensor.set_auto_gain(False,gain_db = 5.74483)      # 自动增益
sensor.set_auto_exposure(False, exposure_us = 3048)
# sensor.set_auto_whitebal(False, rgb_gain_db = (3.61183, -4.0824, -6.0206))    # blue_whitebal
sensor.set_auto_whitebal(False, rgb_gain_db = (-6.0206, -5.49402, -4.0824))#red_white_bal

# get_param()

aim_threshold_list = [red_pillar_threshold]

while True:
    clock.tick()

    uart_receive()
    distingush_disc()
    distingush_stairs()
    distingush_pillar()
    distingush_stack()

