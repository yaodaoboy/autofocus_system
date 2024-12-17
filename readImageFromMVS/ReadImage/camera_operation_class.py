# -- coding: utf-8 --
import threading
import numpy as np
import time
import inspect
import random

from readImageFromMVS.MVSImport.mv_camera_control_class import *
from readImageFromMVS.MVSImport.mv_error_define_const import *
from readImageFromMVS.MVSImport.camera_params_header import *
from readImageFromMVS.MVSImport.pixel_type_header import *
from func_utils.index import to_hex_str


# 强制关闭线程
def async_raise(tid, exctype):
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")


# 停止线程
def stop_thread(thread):
    async_raise(thread.ident, SystemExit)

# 是否是Mono图像
def is_mono_data(en_gvsp_pixel_type):
    if PixelType_Gvsp_Mono8 == en_gvsp_pixel_type or PixelType_Gvsp_Mono10 == en_gvsp_pixel_type \
            or PixelType_Gvsp_Mono10_Packed == en_gvsp_pixel_type or PixelType_Gvsp_Mono12 == en_gvsp_pixel_type \
            or PixelType_Gvsp_Mono12_Packed == en_gvsp_pixel_type:
        return True
    else:
        return False

# 是否是彩色图像
def is_color_data(en_gvsp_pixel_type):
    if PixelType_Gvsp_BayerGR8 == en_gvsp_pixel_type or PixelType_Gvsp_BayerRG8 == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGB8 == en_gvsp_pixel_type or PixelType_Gvsp_BayerBG8 == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGR10 == en_gvsp_pixel_type or PixelType_Gvsp_BayerRG10 == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGB10 == en_gvsp_pixel_type or PixelType_Gvsp_BayerBG10 == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGR12 == en_gvsp_pixel_type or PixelType_Gvsp_BayerRG12 == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGB12 == en_gvsp_pixel_type or PixelType_Gvsp_BayerBG12 == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGR10_Packed == en_gvsp_pixel_type or PixelType_Gvsp_BayerRG10_Packed == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGB10_Packed == en_gvsp_pixel_type or PixelType_Gvsp_BayerBG10_Packed == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGR12_Packed == en_gvsp_pixel_type or PixelType_Gvsp_BayerRG12_Packed == en_gvsp_pixel_type \
            or PixelType_Gvsp_BayerGB12_Packed == en_gvsp_pixel_type or PixelType_Gvsp_BayerBG12_Packed == en_gvsp_pixel_type \
            or PixelType_Gvsp_YUV422_Packed == en_gvsp_pixel_type or PixelType_Gvsp_YUV422_YUYV_Packed == en_gvsp_pixel_type:
        return True
    else:
        return False

# Mono图像转为python数组
def mono_numpy(data, nWidth, nHeight):
    data_ = np.frombuffer(data, count=int(nWidth * nHeight), dtype=np.uint8, offset=0)
    data_mono_arr = data_.reshape(nHeight, nWidth)
    num_array = np.zeros([nHeight, nWidth, 1], "uint8")
    num_array[:, :, 0] = data_mono_arr
    return num_array

# 彩色图像转为python数组
def color_numpy(data, nWidth, nHeight):
    data_ = np.frombuffer(data, count=int(nWidth * nHeight * 3), dtype=np.uint8, offset=0)
    data_r = data_[0:nWidth * nHeight * 3:3]
    data_g = data_[1:nWidth * nHeight * 3:3]
    data_b = data_[2:nWidth * nHeight * 3:3]

    data_r_arr = data_r.reshape(nHeight, nWidth)
    data_g_arr = data_g.reshape(nHeight, nWidth)
    data_b_arr = data_b.reshape(nHeight, nWidth)
    num_array = np.zeros([nHeight, nWidth, 3], "uint8")

    num_array[:, :, 0] = data_r_arr
    num_array[:, :, 1] = data_g_arr
    num_array[:, :, 2] = data_b_arr
    return num_array

# 相机操作类
class CameraOperation:

    def __init__(self, obj_cam, st_device_list, n_connect_num=0, b_open_device=False, b_start_grabbing=False,
                 h_thread_handle=None,
                 b_thread_closed=False, st_frame_info=None, b_exit=False, b_save_bmp=False, b_save_jpg=False,
                 buf_save_image=None,
                 n_save_image_size=0, n_win_gui_id=0, frame_rate=0, exposure_time=0, gain=0):

        self.obj_cam = obj_cam
        self.st_device_list = st_device_list
        self.n_connect_num = n_connect_num
        self.b_open_device = b_open_device
        self.b_start_grabbing = b_start_grabbing
        self.b_thread_closed = b_thread_closed
        self.st_frame_info = st_frame_info
        self.b_exit = b_exit
        self.b_save_bmp = b_save_bmp
        self.b_save_jpg = b_save_jpg
        self.buf_save_image = buf_save_image
        self.n_save_image_size = n_save_image_size
        self.h_thread_handle = h_thread_handle
        self.frame_rate = frame_rate
        self.exposure_time = exposure_time
        self.gain = gain
        self.buf_lock = threading.Lock()  # 取图和存图的buffer锁

    # 打开相机
    def open_device(self):
        if not self.b_open_device:
            if self.n_connect_num < 0:
                return MV_E_CALLORDER

            # ch:选择设备并创建句柄 | en:Select device and create handle
            n_connection_num = int(self.n_connect_num)
            st_device_list = cast(self.st_device_list.pDeviceInfo[int(n_connection_num)],
                                POINTER(MV_CC_DEVICE_INFO)).contents
            self.obj_cam = MvCamera()
            ret = self.obj_cam.MV_CC_CreateHandle(st_device_list)
            if ret != 0:
                self.obj_cam.MV_CC_DestroyHandle()
                return ret

            ret = self.obj_cam.MV_CC_OpenDevice()
            if ret != 0:
                return ret
            print("open device successfully!")
            self.b_open_device = True
            self.b_thread_closed = False

            # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
            if st_device_list.nTLayerType == MV_GIGE_DEVICE:
                n_packet_size = self.obj_cam.MV_CC_GetOptimalPacketSize()
                if int(n_packet_size) > 0:
                    ret = self.obj_cam.MV_CC_SetIntValue("GevSCPSPacketSize", n_packet_size)
                    if ret != 0:
                        print("warning: set packet size fail! ret[0x%x]" % ret)
                else:
                    print("warning: set packet size fail! ret[0x%x]" % n_packet_size)

            st_bool = c_bool(False)
            ret = self.obj_cam.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", st_bool)
            if ret != 0:
                print("get acquisition frame rate enable fail! ret[0x%x]" % ret)

            # ch: 设置触发模式为off | en: Set trigger mode as off
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                print("set trigger mode fail! ret[0x%x]" % ret)
            return MV_OK

    # 开始取图
    def start_grabbing(self, win_handle):
        if not self.b_start_grabbing and self.b_open_device:
            self.b_exit = False
            ret = self.obj_cam.MV_CC_StartGrabbing()
            if ret != 0:
                return ret
            self.b_start_grabbing = True
            print("start grabbing successfully!")
            try:
                thread_id = random.randint(1, 10000)
                self.h_thread_handle = threading.Thread(target=CameraOperation.work_thread, args=(self, win_handle))
                self.h_thread_handle.start()
                self.b_thread_closed = True
            finally:
                pass
            return MV_OK

        return MV_E_CALLORDER

    # 停止取图
    def stop_grabbing(self):
        if self.b_start_grabbing and self.b_open_device:
            # 退出线程
            if self.b_thread_closed:
                stop_thread(self.h_thread_handle)
                self.b_thread_closed = False
            ret = self.obj_cam.MV_CC_StopGrabbing()
            if ret != 0:
                return ret
            print("stop grabbing successfully!")
            self.b_start_grabbing = False
            self.b_exit = True
            return MV_OK
        else:
            return MV_E_CALLORDER

    # 关闭相机
    def close_device(self):
        if self.b_open_device:
            # 退出线程
            if self.b_thread_closed:
                stop_thread(self.h_thread_handle)
                self.b_thread_closed = False
            ret = self.obj_cam.MV_CC_CloseDevice()
            if ret != 0:
                return ret

        # ch:销毁句柄 | Destroy handle
        self.obj_cam.MV_CC_DestroyHandle()
        self.b_open_device = False
        self.b_start_grabbing = False
        self.b_exit = True
        print("close device successfully!")

        return MV_OK

    # 设置触发模式
    def set_trigger_mode(self, is_trigger_mode):
        if not self.b_open_device:
            return MV_E_CALLORDER

        if not is_trigger_mode:
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerMode", 0)
            if ret != 0:
                return ret
        else:
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerMode", 1)
            if ret != 0:
                return ret
            ret = self.obj_cam.MV_CC_SetEnumValue("TriggerSource", 7)
            if ret != 0:
                return ret

        return MV_OK

    # 软触发一次
    def trigger_once(self):
        if self.b_open_device:
            return self.obj_cam.MV_CC_SetCommandValue("TriggerSoftware")

    # 获取参数
    def get_parameter(self):
        if self.b_open_device:
            st_float_param_frame_rate = MVCC_FLOATVALUE()
            memset(byref(st_float_param_frame_rate), 0, sizeof(MVCC_FLOATVALUE))
            st_float_param_exposure_time = MVCC_FLOATVALUE()
            memset(byref(st_float_param_exposure_time), 0, sizeof(MVCC_FLOATVALUE))
            st_float_param_gain = MVCC_FLOATVALUE()
            memset(byref(st_float_param_gain), 0, sizeof(MVCC_FLOATVALUE))
            ret = self.obj_cam.MV_CC_GetFloatValue("AcquisitionFrameRate", st_float_param_frame_rate)
            if ret != 0:
                return ret
            self.frame_rate = st_float_param_frame_rate.fCurValue

            ret = self.obj_cam.MV_CC_GetFloatValue("ExposureTime", st_float_param_exposure_time)
            if ret != 0:
                return ret
            self.exposure_time = st_float_param_exposure_time.fCurValue

            ret = self.obj_cam.MV_CC_GetFloatValue("Gain", st_float_param_gain)
            if ret != 0:
                return ret
            self.gain = st_float_param_gain.fCurValue

            return MV_OK

    # 设置参数
    def set_parameter(self, frame_rate, exposure_time, gain):
        if '' == frame_rate or '' == exposure_time or '' == gain:
            print('show info', 'please type in the text box !')
            return MV_E_PARAMETER
        if self.b_open_device:
            self.obj_cam.MV_CC_SetEnumValue("ExposureAuto", 0)
            # ch: 系统进程睡眠0.2s，保证可以看到效果改变 | en: System processes sleep for 0.2 seconds to ensure that changes in performance are visible
            time.sleep(0.2)
            ret = self.obj_cam.MV_CC_SetFloatValue("ExposureTime", float(exposure_time))
            if ret != 0:
                print('show error', 'set exposure time fail! ret = ' + to_hex_str(ret))
                return ret

            ret = self.obj_cam.MV_CC_SetFloatValue("Gain", float(gain))
            if ret != 0:
                print('show error', 'set gain fail! ret = ' + to_hex_str(ret))
                return ret

            ret = self.obj_cam.MV_CC_SetFloatValue("AcquisitionFrameRate", float(frame_rate))
            if ret != 0:
                print('show error', 'set acquistion frame rate fail! ret = ' + to_hex_str(ret))
                return ret

            print('show info', 'set parameter success!')

            return MV_OK

    # 取图线程函数
    def work_thread(self, win_handle):
        st_out_frame = MV_FRAME_OUT()
        memset(byref(st_out_frame), 0, sizeof(st_out_frame))

        while True:
            ret = self.obj_cam.MV_CC_GetImageBuffer(st_out_frame, 1000)
            if 0 == ret:
                # 拷贝图像和图像信息
                if self.buf_save_image is None:
                    self.buf_save_image = (c_ubyte * st_out_frame.stFrameInfo.nFrameLen)()
                self.st_frame_info = st_out_frame.stFrameInfo

                # 获取缓存锁
                self.buf_lock.acquire()
                cdll.msvcrt.memcpy(byref(self.buf_save_image), st_out_frame.pBufAddr, self.st_frame_info.nFrameLen)
                self.buf_lock.release()

                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]"
                      % (self.st_frame_info.nWidth, self.st_frame_info.nHeight, self.st_frame_info.nFrameNum))
                # 释放缓存
                self.obj_cam.MV_CC_FreeImageBuffer(st_out_frame)
            else:
                print("no data, ret = " + to_hex_str(ret))
                continue

            # 使用Display接口显示图像
            st_display_param = MV_DISPLAY_FRAME_INFO()
            memset(byref(st_display_param), 0, sizeof(st_display_param))
            st_display_param.hWnd = int(win_handle)
            st_display_param.nWidth = self.st_frame_info.nWidth
            st_display_param.nHeight = self.st_frame_info.nHeight
            st_display_param.enPixelType = self.st_frame_info.enPixelType
            st_display_param.pData = self.buf_save_image
            st_display_param.nDataLen = self.st_frame_info.nFrameLen
            self.obj_cam.MV_CC_DisplayOneFrame(st_display_param)

            # 是否退出
            if self.b_exit:
                if self.buf_save_image is not None:
                    del self.buf_save_image
                break

    # 存jpg图像
    def save_jpg(self):
        if self.buf_save_image is None:
            return

        # 获取缓存锁
        self.buf_lock.acquire()

        file_path = str(self.st_frame_info.nFrameNum) + ".jpg"
        c_file_path = file_path.encode('ascii')
        st_save_param = MV_SAVE_IMAGE_TO_FILE_PARAM_EX()
        st_save_param.enPixelType = self.st_frame_info.enPixelType  # ch:相机对应的像素格式 | en:Camera pixel type
        st_save_param.nWidth = self.st_frame_info.nWidth  # ch:相机对应的宽 | en:Width
        st_save_param.nHeight = self.st_frame_info.nHeight  # ch:相机对应的高 | en:Height
        st_save_param.nDataLen = self.st_frame_info.nFrameLen
        st_save_param.pData = cast(self.buf_save_image, POINTER(c_ubyte))
        st_save_param.enImageType = MV_Image_Jpeg  # ch:需要保存的图像类型 | en:Image format to save
        st_save_param.nQuality = 80
        st_save_param.pcImagePath = ctypes.create_string_buffer(c_file_path)
        st_save_param.iMethodValue = 2
        ret = self.obj_cam.MV_CC_SaveImageToFileEx(st_save_param)

        self.buf_lock.release()
        return ret

    # 存BMP图像
    def save_bmp(self):
        if 0 == self.buf_save_image:
            return

        # 获取缓存锁
        self.buf_lock.acquire()

        file_path = str(self.st_frame_info.nFrameNum) + ".bmp"
        c_file_path = file_path.encode('ascii')

        st_save_param = MV_SAVE_IMAGE_TO_FILE_PARAM_EX()
        st_save_param.enPixelType = self.st_frame_info.enPixelType  # ch:相机对应的像素格式 | en:Camera pixel type
        st_save_param.nWidth = self.st_frame_info.nWidth  # ch:相机对应的宽 | en:Width
        st_save_param.nHeight = self.st_frame_info.nHeight  # ch:相机对应的高 | en:Height
        st_save_param.nDataLen = self.st_frame_info.nFrameLen
        st_save_param.pData = cast(self.buf_save_image, POINTER(c_ubyte))
        st_save_param.enImageType = MV_Image_Bmp  # ch:需要保存的图像类型 | en:Image format to save
        st_save_param.nQuality = 8
        st_save_param.pcImagePath = ctypes.create_string_buffer(c_file_path)
        st_save_param.iMethodValue = 2
        ret = self.obj_cam.MV_CC_SaveImageToFileEx(st_save_param)

        self.buf_lock.release()

        return ret
