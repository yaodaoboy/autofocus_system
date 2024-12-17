from PyQt5.QtWidgets import QMainWindow, QMessageBox, QApplication
from .py_uic_basic_demo import UiMainWindow
from .camera_handler import CameraHandler
import sys


class CameraApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = UiMainWindow()
        self.ui.setupUi(self)
        self.camera_handler = CameraHandler()

        # 初始化 UI 和状态
        self.init_ui()
        self.update_ui_controls()

    def init_ui(self):
        # 设备操作
        self.ui.bnEnum.clicked.connect(self.enum_devices)
        self.ui.bnOpen.clicked.connect(self.open_device)
        self.ui.bnClose.clicked.connect(self.close_device)

        # 图像采集操作
        self.ui.bnStart.clicked.connect(self.start_grabbing)
        self.ui.bnStop.clicked.connect(self.stop_grabbing)
        self.ui.bnSaveImage.clicked.connect(self.save_image)

        # 参数设置
        self.ui.bnGetParam.clicked.connect(self.get_param)
        self.ui.bnSetParam.clicked.connect(self.set_param)

        # 触发模式
        self.ui.radioTriggerMode.clicked.connect(self.set_software_trigger_mode)
        self.ui.radioContinueMode.clicked.connect(self.set_continue_mode)
        self.ui.bnSoftwareTrigger.clicked.connect(self.trigger_once)

    def enum_devices(self):
        """枚举设备列表"""
        dev_list, error = self.camera_handler.enum_devices()
        if error:
            QMessageBox.warning(self, "Error", error)
        else:
            self.ui.ComboDevices.clear()
            self.ui.ComboDevices.addItems(dev_list)
            self.ui.ComboDevices.setCurrentIndex(0)

    def open_device(self):
        """打开所选设备"""
        index = self.ui.ComboDevices.currentIndex()
        ret_code = self.camera_handler.open_device(index)
        if ret_code != 0:
            QMessageBox.warning(self, "Error", f"Failed to open device, ret_code is {ret_code}")
        else:
            self.set_continue_mode()
            self.get_param()
            self.camera_handler.is_open = True
            self.update_ui_controls()

    def close_device(self):
        """关闭设备"""
        self.camera_handler.close_device()
        self.camera_handler.is_open = False
        self.camera_handler.is_grabbing = False
        self.update_ui_controls()

    def start_grabbing(self):
        """开始取流"""
        ret = self.camera_handler.start_grabbing(self.ui.widgetDisplay.winId())
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Failed to start grabbing, ret_code is {ret}")
        else:
            self.camera_handler.is_grabbing = True
            self.update_ui_controls()

    def stop_grabbing(self):
        """停止取流"""
        ret = self.camera_handler.stop_grabbing()
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Failed to stop grabbing, ret_code is {ret}")
        else:
            self.camera_handler.is_grabbing = False
            self.update_ui_controls()

    def set_continue_mode(self):
        """设置连续触发模式"""
        ret = self.camera_handler.set_trigger_mode(False)
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Failed to set continue mode, ret_code is {ret}")
        else:
            self.ui.radioContinueMode.setChecked(True)
            self.ui.bnSoftwareTrigger.setEnabled(False)
            self.ui.radioTriggerMode.setChecked(False)

    def set_software_trigger_mode(self):
        """设置软件触发模式"""
        ret = self.camera_handler.set_trigger_mode(True)
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Failed to set trigger mode, ret_code is {ret}")
        else:
            self.ui.radioTriggerMode.setChecked(True)
            self.ui.bnSoftwareTrigger.setEnabled(True)
            self.ui.bnSoftwareTrigger.setEnabled(self.camera_handler.is_grabbing)
        self.update_ui_controls()

    def trigger_once(self):
        """触发一次"""
        ret = self.camera_handler.trigger_once()
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Software trigger failed, ret_code is {ret}")

    def save_image(self):
        """保存图像"""
        ret = self.camera_handler.save_jpg()
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Failed to save image, ret_code is {ret}")
        else:
            QMessageBox.information(self, "Info", "Image saved successfully!")

    def get_param(self):
        """获取当前参数"""
        params, error = self.camera_handler.get_parameter()
        if error:
            QMessageBox.warning(self, "Error", f"Failed to get parameters: {error}")
        else:
            self.ui.edtExposureTime.setText(f"{params['exposure_time']:.2f}")
            self.ui.edtGain.setText(f"{params['gain']:.2f}")
            self.ui.edtFrameRate.setText(f"{params['frame_rate']:.2f}")

    def set_param(self):
        """设置参数"""
        exposure = float(self.ui.edtExposureTime.text())
        gain = float(self.ui.edtGain.text())
        frame_rate = float(self.ui.edtFrameRate.text())
        ret = self.camera_handler.set_parameter(frame_rate, exposure, gain)
        print("set parameters successfully!")
        if ret != 0:
            QMessageBox.warning(self, "Error", f"Failed to set parameters, ret_code is {ret}")

    def update_ui_controls(self):
        """根据设备状态启用/禁用控件"""
        is_open = self.camera_handler.is_open
        is_grabbing = self.camera_handler.is_grabbing

        self.ui.groupParam.setEnabled(is_open)
        self.ui.groupGrab.setEnabled(is_open)

        self.ui.bnOpen.setEnabled(not is_open)
        self.ui.bnClose.setEnabled(is_grabbing and is_open)

        self.ui.bnStart.setEnabled(is_open and not is_grabbing)
        self.ui.bnStop.setEnabled(is_grabbing)
        self.ui.bnSaveImage.setEnabled(is_open and is_grabbing)

        # 软件触发按钮
        self.ui.bnSoftwareTrigger.setEnabled(is_grabbing and self.ui.radioTriggerMode.isChecked())


