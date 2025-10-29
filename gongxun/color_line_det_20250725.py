import numpy as np
import cv2 as cv
import time
import serial
import threading

width = 320
height = 240
FPS = 18

cap_1 = cv.VideoCapture(0, cv.CAP_V4L2)  # 车前方的摄像头(未启用)
ret = cap_1.set(3, 640)
ret = cap_1.set(4, 480)

action_flag = 0  # 功能标志位(默认为颜色、圆环识别模式)

# cap_1 = cv.VideoCapture(0,cv.CAP_V4L2)  # 车前方的摄像头(未启用)
# ret = cap_1.set(3, 800)
# ret = cap_1.set(4, 600)
# 开启串口
ser = serial.Serial(
    port="/dev/ttyAMA0",
    # port="COM5",
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.001,
)

# 设定HSV图像的上下限阈值
# 红色
#HSV_MIN_RED_YUAN_1 = (128, 65, 155)
#HSV_MAX_RED_YUAN_1 = (180, 255, 255)
#lower_color_R_YUAN_1 = np.array([HSV_MIN_RED_YUAN_1[0], HSV_MIN_RED_YUAN_1[1], HSV_MIN_RED_YUAN_1[2]], np.uint8)
#upper_color_R_YUAN_1 = np.array([HSV_MAX_RED_YUAN_1[0], HSV_MAX_RED_YUAN_1[1], HSV_MAX_RED_YUAN_1[2]], np.uint8)

#red
# 区间1
lower_red1 =(0, 43, 46)
upper_red1 =(10, 255, 255)
lower_color_R_1 = np.array([lower_red1[0], lower_red1[1], lower_red1[2]], np.uint8)
upper_color_R_1 = np.array([upper_red1[0], upper_red1[1], upper_red1[2]], np.uint8)
# 区间2
lower_red_1_1 = (156, 43, 46)
upper_red_1_1 = (180, 255, 255)
lower_color_R_1_1 = np.array([lower_red_1_1[0], lower_red_1_1[1], lower_red_1_1[2]], np.uint8)
upper_color_R_1_1 = np.array([upper_red_1_1[0], upper_red_1_1[1], upper_red_1_1[2]], np.uint8)


HSV_MIN_RED_YUAN_2 = (110, 30, 75)
HSV_MAX_RED_YUAN_2 = (180, 150, 210)
lower_color_R_YUAN_2 = np.array([HSV_MIN_RED_YUAN_2[0], HSV_MIN_RED_YUAN_2[1], HSV_MIN_RED_YUAN_2[2]], np.uint8)
upper_color_R_YUAN_2 = np.array([HSV_MAX_RED_YUAN_2[0], HSV_MAX_RED_YUAN_2[1], HSV_MAX_RED_YUAN_2[2]], np.uint8)

HSV_MIN_RED_CIRCLE_1 = (0, 43, 140)  # (0,43,46)
HSV_MAX_RED_CIRCLE_1 = (10, 255, 255)  # (0,43,46)
lower_color_R_CIRCLE_1 = np.array([HSV_MIN_RED_CIRCLE_1[0], HSV_MIN_RED_CIRCLE_1[1], HSV_MIN_RED_CIRCLE_1[2]], np.uint8)
upper_color_R_CIRCLE_1 = np.array([HSV_MAX_RED_CIRCLE_1[0], HSV_MAX_RED_CIRCLE_1[1], HSV_MAX_RED_CIRCLE_1[2]], np.uint8)
HSV_MIN_RED_CIRCLE_2 = (156, 43, 46)  # (110, 110 ,75)
HSV_MAX_RED_CIRCLE_2 = (180, 255, 255)  # (180, 225, 255)
lower_color_R_CIRCLE_2 = np.array([HSV_MIN_RED_CIRCLE_2[0], HSV_MIN_RED_CIRCLE_2[1], HSV_MIN_RED_CIRCLE_2[2]], np.uint8)
upper_color_R_CIRCLE_2 = np.array([HSV_MAX_RED_CIRCLE_2[0], HSV_MAX_RED_CIRCLE_2[1], HSV_MAX_RED_CIRCLE_2[2]], np.uint8)
# 绿色
HSV_MIN_GREEN_YUAN = (35, 92, 170)
HSV_MAX_GREEN_YUAN = (94, 255, 255)
lower_color_G_YUAN = np.array([HSV_MIN_GREEN_YUAN[0], HSV_MIN_GREEN_YUAN[1], HSV_MIN_GREEN_YUAN[2]], np.uint8)
upper_color_G_YUAN = np.array([HSV_MAX_GREEN_YUAN[0], HSV_MAX_GREEN_YUAN[1], HSV_MAX_GREEN_YUAN[2]], np.uint8)
HSV_MIN_GREEN2_CIRCLE = (28, 18, 166)
HSV_MAX_GREEN2_CIRCLE = (68, 255, 255)
lower_color_G_CIRCLE = np.array([HSV_MIN_GREEN2_CIRCLE[0], HSV_MIN_GREEN2_CIRCLE[1], HSV_MIN_GREEN2_CIRCLE[2]],
                                np.uint8)
upper_color_G_CIRCLE = np.array([HSV_MAX_GREEN2_CIRCLE[0], HSV_MAX_GREEN2_CIRCLE[1], HSV_MAX_GREEN2_CIRCLE[2]],
                                np.uint8)
# 蓝色
HSV_MIN_BLUE_YUAN = (82, 79, 189)
HSV_MAX_BLUE_YUAN = (110, 255, 255)
lower_color_B_YUAN = np.array([HSV_MIN_BLUE_YUAN[0], HSV_MIN_BLUE_YUAN[1], HSV_MIN_BLUE_YUAN[2]], np.uint8)
upper_color_B_YUAN = np.array([HSV_MAX_BLUE_YUAN[0], HSV_MAX_BLUE_YUAN[1], HSV_MAX_BLUE_YUAN[2]], np.uint8)
HSV_MIN_BLUE2_CIRCLE = (100, 50, 46)
HSV_MAX_BLUE2_CIRCLE = (125, 255, 205)
lower_color_B_CIRCLE = np.array([HSV_MIN_BLUE2_CIRCLE[0], HSV_MIN_BLUE2_CIRCLE[1], HSV_MIN_BLUE2_CIRCLE[2]], np.uint8)
upper_color_B_CIRCLE = np.array([HSV_MAX_BLUE2_CIRCLE[0], HSV_MAX_BLUE2_CIRCLE[1], HSV_MAX_BLUE2_CIRCLE[2]], np.uint8)
# up green
HSV_MIN_GREEN_YUAN_1 = (58, 89, 130)
HSV_MAX_GREEN_YUAN_1 = (79, 255, 255)
lower_color_G_YUAN_1 = np.array([HSV_MIN_GREEN_YUAN_1[0], HSV_MIN_GREEN_YUAN_1[1], HSV_MIN_GREEN_YUAN_1[2]], np.uint8)
upper_color_G_YUAN_1 = np.array([HSV_MAX_GREEN_YUAN_1[0], HSV_MAX_GREEN_YUAN_1[1], HSV_MAX_GREEN_YUAN_1[2]], np.uint8)
# ALLCOLOR
HSV_MIN_ALLCOLOR_CIRCLE = (0, 48, 126)
HSV_MAX_ALLCOLOR_CIRCLE = (173, 255, 255)
lower_color_ALL_CIRCLE = np.array([HSV_MIN_ALLCOLOR_CIRCLE[0], HSV_MIN_ALLCOLOR_CIRCLE[1], HSV_MIN_ALLCOLOR_CIRCLE[2]],
                                  np.uint8)
upper_color_ALL_CIRCLE = np.array([HSV_MAX_ALLCOLOR_CIRCLE[0], HSV_MAX_ALLCOLOR_CIRCLE[1], HSV_MAX_ALLCOLOR_CIRCLE[2]],
                                  np.uint8)


# 串口获取flag函数,为后台调度函数，用于模式切换
def get_action_flag():
    global action_flag  # 声明该变量为全局变量
    # 判断串口是否打开成功
    while ser.isOpen():
        # 0.1秒接收一次信息
        time.sleep(0.01)
        # 接收到的信息转化为字符串数组
        line = ser.readline()
        res = line.decode()
        # 根据最后发送的数字 确定模式
        if res is not None and len(res) > 0:
            if res == "1":
                print(res)
                action_flag = 1
            elif res == "2":
                print(res)
                action_flag = 2
            elif res == "3":
                print(res)
                action_flag = 3
            elif res == "4":
                print(res)
                action_flag = 4
            elif res == "5":
                print(res)
                action_flag = 5
            elif res == "6":
                print(res)
                action_flag = 6
            elif res == "7":
                print(res)
                action_flag = 7
            elif res == "8":
                print(res)
                action_flag = 8
            elif res == "0":
                print(res)
                action_flag = 0


# 开启get_action_flag任务调度，获取action_flag以调整工作模式
get_flag_thread = threading.Thread(target=get_action_flag)
get_flag_thread.start()
    # 定义3x3矩形核
kernel = np.ones((3, 3), np.uint8)
    # 定义5x5矩形核
kernel2 = np.ones((5, 5), np.uint8)

def cnt_area(cnt):
    area = cv.contourArea(cnt)
    return area


def get_color_mask(median):
    color_mask = None  # 初始化 color_mask 为 None
    if action_flag == 1:  # 红色
        color_mask = cv.inRange(median, lower_color_R_1, upper_color_R_1)+cv.inRange(median, lower_color_R_1_1, upper_color_R_1_1)
    elif action_flag == 2:  # 绿色
        color_mask = cv.inRange(median, lower_color_G_YUAN, upper_color_G_YUAN)
    elif action_flag == 3:  # 蓝色
        color_mask = cv.inRange(median, lower_color_B_YUAN, upper_color_B_YUAN)
    elif action_flag == 4:  # 红色圆环
        color_mask = cv.inRange(median, lower_color_R_CIRCLE_1, upper_color_R_CIRCLE_1)
    elif action_flag == 5:  # 绿色圆环
        color_mask = cv.inRange(median, lower_color_G_CIRCLE, upper_color_G_CIRCLE)
    elif action_flag == 6:  # 蓝色圆环
        color_mask = cv.inRange(median, lower_color_B_CIRCLE, upper_color_B_CIRCLE)
    elif action_flag == 7:  # up green
        color_mask = cv.inRange(median, lower_color_G_YUAN_1, upper_color_G_YUAN_1)
    elif action_flag == 8:  # all_color
        color_mask = cv.inRange(median, lower_color_ALL_CIRCLE, upper_color_ALL_CIRCLE)
    else:
        print("Invalid action flag:", action_flag)
        # 返回一个空的掩码，而不是 None
        color_mask = np.empty((0, 0), dtype=np.uint8)
    return color_mask


def process_image(color_mask):
    if color_mask.size == 0:
        print("Empty color mask in process_image, returning empty mask.")
        return np.empty((0, 0), dtype=np.uint8)  # 返回一个空的掩码

    eroded = cv.erode(color_mask, kernel, iterations=1)
    
    if action_flag==1:
        filtered_mask = cv.dilate(eroded, kernel, iterations=6)
    else:
        filtered_mask = cv.dilate(eroded, kernel, iterations=4)
        
    return filtered_mask


def find_contours(filtered_mask):
    # 在 OpenCV 4 中，findContours 返回两个值：contours 和 hierarchy
    contours, _ = cv.findContours(filtered_mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return contours


#
#  name: process_contours
#  @param
#  @return
#
def process_contours(contours, frame_color, action):
    has_valid_contour = False  # 标记是否存在有效轮廓
    process_every_n = 5  # 采样间隔（实际根据帧率调整）
    # 按面积降序排序（优化处理顺序）
    sorted_contours = sorted(contours, key=cv.contourArea, reverse=True)[:5]
    for idx, contour in enumerate(sorted_contours):
        # 面积过滤前置（快速跳过无效轮廓）
        area = cv.contourArea(contour)
        if (action < 4 or action == 8) and not (2000 < area < 90000):
            continue
        elif not (2000 < area < 90000):
            continue
        # 圆形度计算
        perimeter = cv.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < 0.5:
            continue
        # 仅处理满足条件的最大前N个轮廓
        if idx == 0:  # 只处理最大轮廓
            draw_contour_and_send_data(contour, frame_color, action, circularity)
            has_valid_contour = True
        # 采样处理逻辑（示例：每5帧发送附加数据）
        if idx % process_every_n == 0:
            pass  # 这里可以添加特殊采样逻辑

    # 无有效轮廓时发送空数据（避免频繁发送）
    if not has_valid_contour:
        ser.write(bytes([0x55, 0x5A, 0, 0, 0, 0xAA]))

def draw_contour_and_send_data(contour, frame, action, circularity):
    # 绘制轮廓
    cv.drawContours(frame, [contour], -1, (0, 255, 0), 2)
    # 计算质心
    M = cv.moments(contour)
    if M["m00"] == 0:
        return
    cX, cY = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    # 绘制质心
    cv.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
    # 发送数据（添加校验机制）
    data_bytes = bytes([0x55, 0x5A, action, int(cX /3.5), int(cY / 2.5), 0xAA])
    if len(data_bytes) == 6:  # 数据完整性检查
        ser.write(data_bytes)
        print(int(action_flag),int(cX/3.5),int(cY/2.5),circularity)
        

def detect_color_Scan():
    while cap_1.isOpened() and action_flag in [1, 2, 3, 7]:
        # print("camera is open")
        ret_color, frame_color = cap_1.read()
        if not ret_color:
            break
        frame_color = frame_color[100:650, 100:700]  # 窗口裁剪
        start_time = time.time()
        hsv_image = cv.cvtColor(frame_color, cv.COLOR_BGR2HSV)  # HSV变换
        median = cv.GaussianBlur(hsv_image, (3, 3), 0)  # 中值滤波
        color_mask = get_color_mask(median)
        if color_mask.size == 0:
            print("Empty color mask, skipping processing.")
            continue  # 跳过当前循环的剩余部分
        filtered_mask = process_image(color_mask)
        contours = find_contours(filtered_mask)

        process_contours(contours, frame_color, action_flag)

        cv.imshow('Color Detection', filtered_mask)
        cv.imshow('Color Detection2', frame_color)
        cv.waitKey(1)
        time.sleep(0.01)
    cap_1.release()
    cv.destroyAllWindows()


def detect_color_circle():  # 定位圆环
    while cap_1.isOpened() and action_flag in [4, 5, 6, 8]:  # 判断是否成功开启视频
        ret_color, frame_color = cap_1.read()
        # frame_color = frame_color[40:770,50:590]
        # print("camera is open")
        if not ret_color:
            print("break")
            break
        start_time = time.time()
        hsv_image = cv.cvtColor(frame_color, cv.COLOR_BGR2HSV)
        median = cv.GaussianBlur(hsv_image, (3, 3), 0)

        color_mask = get_color_mask(median)
        filtered_mask = process_image(color_mask)
        contours = find_contours(filtered_mask)

        process_contours(contours, frame_color, action_flag)

        cv.imshow('Color Detection', filtered_mask)
        cv.imshow('Color Detection2', frame_color)
        cv.waitKey(1)
        time.sleep(0.01)
    cap_1.release()
    cv.destroyAllWindows()


while True:
    cap_1 = cv.VideoCapture(0, cv.CAP_V4L2)  # 车前方的摄像头(未启用)
    ret = cap_1.set(3, 800)
    ret = cap_1.set(4, 600)
    if action_flag == 0:
        print("0")
        ser.write(bytes([0x55, 0x5A, 0, 0, 0, 0xAA]))
    elif action_flag == 1 or action_flag == 2 or action_flag == 3 or action_flag == 7:
        detect_color_Scan()
    elif action_flag == 4 or action_flag == 5 or action_flag == 6 or action_flag == 8:
        detect_color_circle()
