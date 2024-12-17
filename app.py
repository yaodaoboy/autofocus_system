import sys
from PyQt5.QtWidgets import QApplication
from readImageFromMVS.ReadImage.camera_app import CameraApp

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CameraApp()
    window.show()
    sys.exit(app.exec_())