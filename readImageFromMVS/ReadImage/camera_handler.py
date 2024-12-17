from readImageFromMVS.MVSImport.camera_params_header import *
from readImageFromMVS.MVSImport.mv_camera_control_class import *
from readImageFromMVS.ReadImage.camera_operation_class import CameraOperation
from func_utils.index import to_hex_str, decoding_char


class CameraHandler:
    def __init__(self):
        self.obj_cam_operation = None
        self.is_open = False  # 设备是否已打开
        self.is_grabbing = False  # 是否正在采集图像
        self.current_device = None  # 当前打开的设备
        self.trigger_mode = False  # False: 连续模式, True: 触发模式
        self.device_list = MV_CC_DEVICE_INFO_LIST()
        self.cam = MvCamera()

        # 参数设置（初始值）
        self.params = {
            "exposure_time": 1000.0,  # 曝光时间（单位: ms）
            "gain": 1.0,  # 增益
            "frame_rate": 30.0  # 帧率
        }

    def enum_devices(self):
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, self.device_list)
        if ret != 0:
            return [], f"Enum devices failed: {to_hex_str(ret)}"

        dev_list = []
        for i in range(self.device_list.nDeviceNum):
            mvcc_dev_info = cast(self.device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                user_name = decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName)
                ip = self._get_ip_address(mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp)
                dev_list.append(f"[{i}]GigE: {user_name} {model_name}({ip})")
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                user_name = decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName)
                model_name = decoding_char(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName)
                serial_number = self._get_serial_number(mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber)
                dev_list.append(f"[{i}]USB: {user_name} {model_name}({serial_number})")
        return dev_list, None

    def open_device(self, index):
        """打开指定设备"""
        self.obj_cam_operation = CameraOperation(self.cam, self.device_list, index)
        ret = self.obj_cam_operation.open_device()
        self.is_open = (ret == 0)
        return ret

    def close_device(self):
        """
        关闭当前设备
        """
        if self.is_open:
            self.is_open = False
            print(f"Device '{self.current_device}' closed.")
            self.obj_cam_operation.close_device()
        self.is_grabbing = False

    def set_continute_mode(self, is_continuous):
        ret = self.obj_cam_operation.set_trigger_mode(is_continuous)
        return ret

    def start_grabbing(self, window_id):
        """
        开始图像采集
        :param window_id: 显示图像的窗口句柄（模拟参数）
        :return: 0 表示成功, 其他值表示失败
        """
        if not self.is_open:
            return -1  # 设备未打开
        ret = self.obj_cam_operation.start_grabbing(window_id)
        if ret != 0:
            print("Error starting grab")
        else:
            self.is_grabbing = True
            print(f"Started grabbing on window {window_id}.")
        return ret

    def stop_grabbing(self):
        """
        停止图像采集
        """
        ret = self.obj_cam_operation.stop_grabbing()
        if ret != 0:
            return -1
        else:
            self.is_grabbing = False
            print("Image grabbing stopped.")
            return 0

    def set_trigger_mode(self, is_software_trigger_mode):
        """
        设置触发模式
        :param is_software_trigger_mode: True: 软件触发模式, False: 连续模式
        :return: 0 表示成功, 其他值表示失败
        """
        if not self.is_open:
            return -1  # 设备未打开

        ret = self.obj_cam_operation.set_trigger_mode(is_software_trigger_mode)
        mode = "Software Trigger Mode" if is_software_trigger_mode else "Continuous Mode"
        print(f"Trigger mode set to: {mode}")
        return ret

    def trigger_once(self):
        """
        触发一次图像采集（仅软件触发模式下有效）
        :return: 0 表示成功, 其他值表示失败
        """
        if not self.is_open or not self.trigger_mode:
            return -1  # 设备未打开或未处于触发模式

        print("Software trigger executed: Image captured.")
        ret = self.obj_cam_operation.trigger_once()
        return ret

    def save_bmp(self):
        """
        保存当前图像格式为bmp
        """
        if not self.is_grabbing:
            return -1  # 未在采集状态

        ret = self.obj_cam_operation.save_bmp()
        return ret

    def save_jpg(self):
        """
        保存当前图像格式为jpg
        """
        if not self.is_grabbing:
            return -1  # 未在采集状态

        ret = self.obj_cam_operation.save_jpg()
        return ret

    def get_parameter(self):
        """
        获取当前参数设置
        :return: 参数字典, 错误消息 (None 表示无错误)
        """

        if not self.is_open:
            return None, "Device not opened."

        return self.params, None

    def set_parameter(self, frame_rate, exposure_time, gain):
        """
        设置参数
        :param frame_rate: 帧率
        :param exposure_time: 曝光时间
        :param gain: 增益
        :return: 0 表示成功, 其他值表示失败
        """
        if not self.is_open:
            return -1  # 设备未打开

        ret = self.obj_cam_operation.set_parameter(frame_rate, exposure_time, gain)
        if ret == 0:
            self.params["frame_rate"] = frame_rate
            self.params["exposure_time"] = exposure_time
            self.params["gain"] = gain
            print(f"Parameters updated: Frame Rate={frame_rate}, Exposure={exposure_time}, Gain={gain}")
        return ret

    @staticmethod
    def _get_ip_address(ip):
        return ".".join([str((ip >> (8 * i)) & 0xFF) for i in range(4)][::-1])

    @staticmethod
    def _get_serial_number(chSerialNumber):
        return "".join(chr(b) for b in chSerialNumber if b != 0)
