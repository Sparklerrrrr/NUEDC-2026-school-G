from libs.PipeLine import PipeLine
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import os, sys, ujson, gc, math
from media.media import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aicube
import machine

class PersonDetectionApp(AIBase):
    def __init__(self, kmodel_path, model_input_size, labels, anchors, confidence_threshold=0.2, nms_threshold=0.5, nms_option=False, strides=[8,16,32], rgb888p_size=[224,224], display_size=[1920,1080], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.model_input_size = model_input_size
        self.labels = labels
        self.anchors = anchors
        self.strides = strides
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.nms_option = nms_option
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0],16), rgb888p_size[1]]
        self.display_size = [ALIGN_UP(display_size[0],16), display_size[1]]
        self.debug_mode = debug_mode
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)

    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, _ = center_pad_param(self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0,0,0,0,top,bottom,left,right], 0, [0,0,0])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1,3,ai2d_input_size[1],ai2d_input_size[0]], [1,3,self.model_input_size[1],self.model_input_size[0]])

    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            dets = aicube.anchorbasedet_post_process(results[0], results[1], results[2], self.model_input_size, self.rgb888p_size, self.strides, len(self.labels), self.confidence_threshold, self.nms_threshold, self.anchors, self.nms_option)
            return dets

    def draw_result(self, pl, dets):
        with ScopedTiming("display_draw", self.debug_mode > 0):
            if dets:
                pl.osd_img.clear()
                for det_box in dets:
                    x1, y1, x2, y2 = det_box[2], det_box[3], det_box[4], det_box[5]
                    w = float(x2 - x1) * self.display_size[0] // self.rgb888p_size[0]
                    h = float(y2 - y1) * self.display_size[1] // self.rgb888p_size[1]
                    x1 = int(x1 * self.display_size[0] // self.rgb888p_size[0])
                    y1 = int(y1 * self.display_size[1] // self.rgb888p_size[1])
                    x2 = int(x2 * self.display_size[0] // self.rgb888p_size[0])
                    y2 = int(y2 * self.display_size[1] // self.rgb888p_size[1])
                    if (h < (0.1 * self.display_size[0])):
                        continue
                    if (w < (0.25 * self.display_size[0]) and ((x1 < (0.03 * self.display_size[0])) or (x2 > (0.97 * self.display_size[0])))):
                        continue
                    if (w < (0.15 * self.display_size[0]) and ((x1 < (0.01 * self.display_size[0])) or (x2 > (0.99 * self.display_size[0])))):
                        continue
                    pl.osd_img.draw_rectangle(x1, y1, int(w), int(h), color=(255, 0, 255, 0), thickness=2)
                    pl.osd_img.draw_string_advanced(x1, y1-50, 32, " " + self.labels[det_box[0]] + " " + str(round(det_box[1], 2)), color=(255, 0, 255, 0))
            else:
                pl.osd_img.clear()

from machine import UART, FPIOA

class UARTComm:
    def __init__(self, baudrate=115200):
        try:
            # 使用 FPIOA 配置 UART2 引脚
            fpioa = FPIOA()
            # 将引脚 5 配置为 UART2_TXD
            fpioa.set_function(5, FPIOA.UART2_TXD)
            # 将引脚 6 配置为 UART2_RXD
            fpioa.set_function(6, FPIOA.UART2_RXD)
            print("UART2 引脚配置成功")
            
            # 初始化 UART
            self.uart = UART(
                UART.UART2,
                baudrate=baudrate,
                bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE,
                stop=UART.STOPBITS_ONE
            )
            print("串口初始化成功: UART2")
        except Exception as e:
            print(f"串口初始化失败: {e}")
            self.uart = None

    def send_command(self, command):
        if not self.uart:
            print("串口未初始化")
            return False
        try:
            self.uart.write((command + '\n').encode())
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False

    def receive_response(self):
        if not self.uart:
            return None
        try:
            response = self.uart.readline()
            if response:
                return response.decode().strip()
            return None
        except Exception as e:
            print(f"接收响应失败: {e}")
            return None

    def close(self):
        if self.uart:
            self.uart.deinit()
            print("串口已关闭")
            self.uart = None

class VisionFollow:
    def __init__(self):
        self.uart = UARTComm()  # UART2
        self.screen_center = (640, 360)  # 1280x720分辨率的中心
        self.threshold = 100  # 中心偏差阈值
        self.base_speed = 30  # 基础速度
        self.current_direction = "STOP"  # 当前运动方向
        self.last_command = "X"  # 上次发送的命令

    def process_detection(self, dets):
        if not dets:
            # 没有检测到目标，停止
            if self.last_command != "X":
                self.uart.send_command('X')
                self.current_direction = "STOP"
                self.last_command = "X"
                print(f"方向: {self.current_direction} (无目标)")
            return self.current_direction

        # 选择最接近中心的目标
        target = self.select_target(dets)
        if not target:
            # 没有有效目标，停止
            if self.last_command != "X":
                self.uart.send_command('X')
                self.current_direction = "STOP"
                self.last_command = "X"
                print(f"方向: {self.current_direction} (无有效目标)")
            return self.current_direction

        x1, y1, x2, y2 = target[2], target[3], target[4], target[5]
        target_center = ((x1 + x2) // 2, (y1 + y2) // 2)
        x_error = target_center[0] - self.screen_center[0]

        # 根据偏差发送控制命令
        if abs(x_error) < self.threshold:
            # 目标在中心附近，前进
            if self.last_command != "W":
                self.uart.send_command('W')
                self.current_direction = "FORWARD"
                self.last_command = "W"
                print(f"方向: {self.current_direction} (目标在中心，偏差: {x_error})")
        elif x_error > self.threshold:
            # 目标在右侧，右转
            if self.last_command != "D":
                self.uart.send_command('D')
                self.current_direction = "RIGHT"
                self.last_command = "D"
                print(f"方向: {self.current_direction} (目标在右侧，偏差: {x_error})")
        else:
            # 目标在左侧，左转
            if self.last_command != "A":
                self.uart.send_command('A')
                self.current_direction = "LEFT"
                self.last_command = "A"
                print(f"方向: {self.current_direction} (目标在左侧，偏差: {x_error})")
        
        return self.current_direction

    def select_target(self, dets):
        if not dets:
            return None

        closest_target = None
        min_distance = float('inf')

        for det in dets:
            x1, y1, x2, y2 = det[2], det[3], det[4], det[5]
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            distance = abs(center[0] - self.screen_center[0])

            if distance < min_distance:
                min_distance = distance
                closest_target = det

        return closest_target

if __name__ == "__main__":
    print("=== K230视觉智能小车控制程序 ===")
    print("正在初始化...")
    
    display_mode = "hdmi"
    rgb888p_size = [1280, 720]
    kmodel_path = "/sdcard/examples/kmodel/person_detect_yolov5n.kmodel"
    confidence_threshold = 0.2
    nms_threshold = 0.6
    labels = ["person"]
    anchors = [10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326]

    print("初始化管道...")
    pl = PipeLine(rgb888p_size=rgb888p_size, display_mode=display_mode)
    pl.create()
    display_size = pl.get_display_size()
    print(f"显示尺寸: {display_size}")

    print("初始化人体检测...")
    print(f"模型路径: {kmodel_path}")
    person_det = PersonDetectionApp(kmodel_path, model_input_size=[640, 640], labels=labels, anchors=anchors, confidence_threshold=confidence_threshold, nms_threshold=nms_threshold, nms_option=False, strides=[8, 16, 32], rgb888p_size=rgb888p_size, display_size=display_size, debug_mode=0)
    person_det.config_preprocess()
    print("人体检测初始化完成")

    print("初始化视觉跟随...")
    vision_follow = VisionFollow()
    print("切换到视觉模式")
    vision_follow.uart.send_command('3')  # 切换到视觉模式
    
    print("初始化完成，开始运行...")

    try:
        frame_count = 0
        while True:
            frame_count += 1
            print(f"\n=== 第 {frame_count} 帧 ===")
            
            with ScopedTiming("total", 1):
                print("获取图像帧...")
                img = pl.get_frame()
                
                print("执行目标检测...")
                res = person_det.run(img)
                print(f"检测到 {len(res) if res else 0} 个目标")
                
                print("处理视觉跟随...")
                direction = vision_follow.process_detection(res)
                
                print("绘制结果...")
                person_det.draw_result(pl, res)
                
                # 在图像上显示运动方向
                pl.osd_img.draw_string_advanced(50, 50, 32, f"Direction: {direction}", color=(0, 255, 0, 0))
                
                print("显示图像...")
                pl.show_image()
                
                print("垃圾回收...")
                gc.collect()
                print("帧处理完成")
    except KeyboardInterrupt:
        print("程序被用户中断")
        vision_follow.uart.send_command('X')
        vision_follow.uart.send_command('1')  # 切换到停止模式
    finally:
        print("清理资源...")
        person_det.deinit()
        pl.destroy()
        vision_follow.uart.close()
