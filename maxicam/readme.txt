MaixCam 工训视觉项目说明
========================

一、当前推荐方案
----------------

运行脚本：maixcam_cv_ring.py

模式 0：待机，关闭补光灯。
模式 1：Champion_material2 YOLO 检测红、绿、蓝物料。
模式 2：轻量霍夫圆检测三个单线黑环，不加载色环模型。

三个黑环按圆心横坐标从左到右编号为 cls=1、2、3。这里的 cls 表示
左、中、右位置，不再表示红、绿、蓝颜色。


二、文件用途
------------

maixcam_black_ring.py
    黑线阈值和轮廓层级实验版。

maixcam_cv_ring.py
    当前轻量霍夫版：Champion_material2 物料 YOLO + 三个黑环 HoughCircles 定位。

maixcam_threshold_ring.py
    旧的彩色色环方案：物料 YOLO + 绿色色环 YOLO/HSV 混合定位。

maixcam_hsv_threshold_tool.py
    MaixCam 触屏 HSV 调参工具。保存参数到：
    /root/hsv_thresholds.json

maixcam.py
    全 YOLO 备选版本：物料 YOLO + 色环 YOLO，不做阈值精定位。

capture_material.py
    触屏拍照脚本，JPG 保存到 /root/material_photos。

Champion_material1/
    旧物料模型，主文件 model_292144.mud。

Champion_material2/
    当前物料模型，主文件 model_293122.mud。

Champion_ring1/
    当前色环模型，主文件 model_291990.mud。

dist/
    以前生成的安装包，不一定包含当前最新混合版代码。

三、模型路径
------------

maixcam_cv_ring.py 的物料模型路径：
    /root/models/maixhub/Champion_material2/model_293122.mud
黑环仍使用轻量 HoughCircles，不加载色环模型。


四、轻量霍夫参数
----------------

RING_PROCESS_W/H = 240/180：色环处理分辨率。
RING_ROI_SIZE = 140：只在画面中心 140x140 区域运行霍夫圆。
RING_MIN/MAX_RADIUS = 6/65：处理图上的半径范围。
RING_MIN_DISTANCE = 20：两个圆心的最小间距。
param1 = 120：霍夫内部边缘检测阈值。
param2 = 0.85：HOUGH_GRADIENT_ALT 圆形置信门槛。


五、串口通信
------------

UART0：/dev/ttyS0
波特率：9600
TX：A16
RX：A17

主控发送 ASCII 字符切换模式：

'0'：待机
'1'：物料检测
'2'：三个黑环定位
'L'：打开补光灯
'l'：关闭补光灯

视觉返回帧：

55 5B mode count [cls x y]... AA

模式 1 类别：1=红，2=绿，3=蓝。
模式 2 类别：单独检测到一个环时固定为 2；检测到多个环时按从左到右编号 1、2、3。
坐标由 640x480 映射到 0～240。


六、实车使用顺序
----------------

1. 上传并启动 maixcam_cv_ring.py。
2. 主控发送 '1' 检测物料，发送 '2' 定位黑环，发送 '0' 待机。
3. 当前 DEBUG_MODE=False，模式只由主控切换，触屏双击切换已关闭。
4. 当前 SHOW_IMAGE=True，屏幕显示检测画面和结果框。


七、性能设置
------------

DEBUG_MODE=False：关闭触屏切换和 FPS 串口打印，只接受主控模式命令。
SHOW_IMAGE=True：开启屏幕显示；改为 False 可关闭显示以提高实车帧率。
DISPLAY_EVERY_N=3：屏幕每三帧刷新一次，检测和串口返回仍逐帧运行。

当前 app.yaml 仍指向 maixcam.py。直接上传脚本时请选择
maixcam_cv_ring.py。
