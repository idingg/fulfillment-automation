from PyQt5 import QtCore, QtGui, QtWidgets
import smtplib
from email.mime.text import MIMEText
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from std_msgs.msg import String
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QDialog,
    QLineEdit,
    QVBoxLayout,
    QLabel,
    QPushButton,
    QMessageBox,
)


class LoginDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("관리자 로그인")
        self.setGeometry(300, 300, 300, 200)

        # Predefined admin credentials (you might want to use a more secure method)
        self.ADMIN_USERNAME = "idingg"
        self.ADMIN_PASSWORD = "1234"

        layout = QVBoxLayout()

        self.username_label = QLabel("사용자명:")
        self.username_input = QLineEdit()

        self.password_label = QLabel("비밀번호:")
        self.password_input = QLineEdit()
        self.password_input.setEchoMode(QLineEdit.Password)

        self.login_button = QPushButton("로그인")
        self.login_button.clicked.connect(self.validate_login)

        layout.addWidget(self.username_label)
        layout.addWidget(self.username_input)
        layout.addWidget(self.password_label)
        layout.addWidget(self.password_input)
        layout.addWidget(self.login_button)

        self.setLayout(layout)

    def validate_login(self):
        username = self.username_input.text()
        password = self.password_input.text()

        if username == self.ADMIN_USERNAME and password == self.ADMIN_PASSWORD:
            self.accept()  # Close dialog and return accepted status
        else:
            QMessageBox.warning(
                self, "로그인 실패", "잘못된 사용자명 또는 비밀번호입니다."
            )


class fulfillmentGUI(Node):
    def __init__(self):
        super().__init__("main_window")

        # Main Window Setup
        self.MainWindow = QtWidgets.QMainWindow()
        self.ui = Ui_MainWindow(self)
        self.ui.setupUi(self.MainWindow)

        # ROS2 Setup
        self.bridge = CvBridge()
        self.camera_subscriber = self.create_subscription(
            Image, "/image_raw", self.camera_callback, 10
        )

        # Create a publisher for job selection
        self.job_publisher = self.create_publisher(String, "/job_selection", 10)

        # ROS2 Spin Thread
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

    def publish_job_selection(self, job):
        # Create a ROS message
        msg = String()
        msg.data = f"Job: {job['name']}, Red: {job['red']}, Blue: {job['blue']}, Goal: {job['goal']}"

        # Publish the message
        self.job_publisher.publish(msg)

        # Log the message to terminal
        self.get_logger().info(f"Published job selection: {msg.data}")

    def ros_spin(self):
        rclpy.spin(self)

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.resize(cv_image, (360, 260))

            # OpenCV image to QImage
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(
                cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888
            ).rgbSwapped()

            # QImage to QPixmap
            pixmap = QPixmap.fromImage(q_image)

            # camera label 업데이트 (main thread)
            self.ui.update_camera_label(pixmap)
        except Exception as e:
            print(f"Error processing camera image: {e}")

    def show(self):
        self.MainWindow.show()


class Ui_MainWindow(object):
    def __init__(self, node):
        self.node = node
        self.elapsed_time = 0
        self.timer = QtCore.QTimer()

    def setupUi(self, MainWindow):
        MainWindow.resize(950, 700)
        MainWindow.setWindowTitle("Task List")

        # Central Widget
        self.centralwidget = QtWidgets.QWidget(MainWindow)

        # Job Buttons Section
        self.job_buttons_layout = QtWidgets.QVBoxLayout()
        self.job_buttons_widget = QtWidgets.QWidget(self.centralwidget)
        self.job_buttons_widget.setGeometry(QtCore.QRect(20, 470, 520, 150))

        # Define job details
        jobs = [
            {"name": "Job1", "red": "red*2", "blue": "blue*1", "goal": "goto goal 1"},
            {"name": "Job2", "red": "red*1", "blue": "blue*2", "goal": "goto goal 2"},
            {"name": "Job3", "red": "red*1", "blue": "", "goal": "goto goal 3"},
        ]

        # Create buttons for each job
        for job in jobs:
            job_button = QtWidgets.QPushButton(job["name"])
            job_button.setToolTip(
                f"Red: {job['red']}, Blue: {job['blue']}, Goal: {job['goal']}"
            )
            self.job_buttons_layout.addWidget(job_button)

            # Optional: Connect button to a method that handles job selection
            job_button.clicked.connect(
                lambda checked, j=job: self.handle_job_selection(j)
            )

        self.job_buttons_widget.setLayout(self.job_buttons_layout)

        # Add QTextBrowser to display selected job details with scrolling
        self.job_details_browser = QtWidgets.QTextBrowser(self.centralwidget)
        self.job_details_browser.setGeometry(QtCore.QRect(30, 620, 500, 50))
        self.job_details_browser.setStyleSheet("font-size: 14px; color: black;")
        self.job_details_browser.setText("선택된 작업 정보가 여기에 누적 표시됩니다.")

        # Time Label and LCD
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(230, 10, 150, 17))
        self.label_2.setText("작업 소요 시간")

        self.time = QtWidgets.QLCDNumber(self.centralwidget)
        self.time.setGeometry(QtCore.QRect(230, 30, 150, 31))

        # Conveyor Move Slider
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(600, 20, 150, 17))
        self.label_3.setText("Conveyor Move")

        self.cvMove_button = QtWidgets.QSlider(self.centralwidget)
        self.cvMove_button.setGeometry(QtCore.QRect(600, 40, 150, 21))
        self.cvMove_button.setOrientation(QtCore.Qt.Horizontal)

        # YOLO GroupBoxes
        self.groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox.setGeometry(QtCore.QRect(560, 140, 171, 200))
        self.groupBox.setTitle("YOLO 인식 시작")

        self.groupBox_2 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_2.setGeometry(QtCore.QRect(740, 140, 171, 200))
        self.groupBox_2.setTitle("YOLO 인식 끝")

        # Camera Display Label
        self.camera_label = QtWidgets.QLabel(self.centralwidget)
        self.camera_label.setGeometry(
            QtCore.QRect(20, 130, 411, 311)
        )  # 카메라 나오는 위치
        self.camera_label.setText("")  # 이미지만 표시

        # Camera Display Label for Text
        self.text_label = QtWidgets.QLabel(self.centralwidget)
        self.text_label.setGeometry(QtCore.QRect(20, 70, 171, 100))  # 텍스트 표시
        self.text_label.setText("실시간 카메라 화면")
        self.text_label.setAlignment(QtCore.Qt.AlignCenter)

        # Text Browser for Real-time Screen
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(20, 10, 181, 51))
        self.textBrowser.setHtml(
            '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" '
            '"http://www.w3.org/TR/REC-html40/strict.dtd">\n'
            '<html><head><meta name="qrichtext" content="1" />'
            '<style type="text/css">\n'
            "p, li { white-space: pre-wrap; }\n"
            "</style></head><body style=\" font-family:'Ubuntu'; "
            'font-size:11pt; font-weight:400; font-style:normal;">\n'
            '<p align="center" style=" margin-top:0px; margin-bottom:0px; '
            "margin-left:0px; margin-right:0px; -qt-block-indent:0; "
            'text-indent:0px;"><span style=" font-size:20pt; font-weight:600; '
            'color:#000000;">실시간 화면</span></p></body></html>'
        )

        # Status Text Browser
        self.textBrowser_4 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_4.setGeometry(QtCore.QRect(400, 30, 100, 31))
        self.textBrowser_4.setText("운행멈춤")

        # Control Buttons
        self.play_button = QtWidgets.QPushButton(self.centralwidget)
        self.play_button.setGeometry(QtCore.QRect(560, 350, 81, 31))
        self.play_button.setText("Play")

        self.stop_button = QtWidgets.QPushButton(self.centralwidget)
        self.stop_button.setGeometry(QtCore.QRect(650, 350, 81, 31))
        self.stop_button.setText("Stop")

        self.reset_button = QtWidgets.QPushButton(self.centralwidget)
        self.reset_button.setGeometry(QtCore.QRect(740, 350, 81, 31))
        self.reset_button.setText("Reset")

        self.resume_button = QtWidgets.QPushButton(self.centralwidget)
        self.resume_button.setGeometry(QtCore.QRect(830, 350, 81, 31))
        self.resume_button.setText("Resume")

        self.allstop_button = QtWidgets.QPushButton(self.centralwidget)
        self.allstop_button.setGeometry(QtCore.QRect(560, 390, 351, 31))
        self.allstop_button.setText("All Stop")

        # Email Section
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(560, 470, 100, 17))
        self.label.setText("관리자 이메일 입력!!!")

        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(560, 490, 350, 31))

        self.emailgo_button = QtWidgets.QPushButton(self.centralwidget)
        self.emailgo_button.setGeometry(QtCore.QRect(900, 490, 41, 31))
        self.emailgo_button.setText("전송")

        # Connect buttons
        self.emailgo_button.clicked.connect(self.send_email)
        self.play_button.clicked.connect(self.start_operation)
        self.stop_button.clicked.connect(self.stop_operation)
        self.reset_button.clicked.connect(self.reset_operation)
        self.resume_button.clicked.connect(self.resume_operation)
        self.allstop_button.clicked.connect(self.all_stop_operation)

        # Set Central Widget
        MainWindow.setCentralWidget(self.centralwidget)

        # Menubar and Statusbar
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1000, 28))
        MainWindow.setMenuBar(self.menubar)

        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.statusbar)

        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def handle_job_selection(self, job):
        # QTextBrowser에 작업 정보 추가
        current_text = self.job_details_browser.toHtml()
        new_entry = f"<p>Selected Job: <b>{job['name']}</b>, Red: {job['red']}, Blue: {job['blue']}, Goal: {job['goal']}</p>"
        self.job_details_browser.setHtml(current_text + new_entry)

        # 스크롤을 최신 작업 정보로 자동 이동
        self.job_details_browser.verticalScrollBar().setValue(
            self.job_details_browser.verticalScrollBar().maximum()
        )

        # ROS2 메시지로 작업 정보를 퍼블리시
        self.node.publish_job_selection(job)

    def update_camera_label(self, pixmap):
        self.camera_label.setPixmap(
            pixmap.scaled(self.camera_label.size(), QtCore.Qt.KeepAspectRatio)
        )

    def send_email(self):
        recipient = self.lineEdit.text()
        if not recipient:
            QtWidgets.QMessageBox.warning(
                None, "Error", "Please enter a valid email address."
            )
            return

        # Email settings
        smtp_server = "smtp.gmail.com"
        smtp_port = 587
        sender_email = "pigi0420@gmail.com"
        sender_password = "jhdk vafn bpdp usus"

        # Email content
        subject = "[E-2] 오류 발생"
        body = f"작업 중 오류가 발생하였습니다 확인 부탁드립니다.\n오류 발생 시간: {self.elapsed_time}초"
        msg = MIMEText(body, _charset="utf-8")
        msg["Subject"] = subject
        msg["From"] = sender_email
        msg["To"] = recipient

        try:
            # Send email
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, sender_password)
            server.sendmail(sender_email, recipient, msg.as_string())
            server.quit()
            QtWidgets.QMessageBox.information(None, "Success", "이메일 전송 완료")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error", f"이메일 전송 불가: {str(e)}")

    def start_operation(self):
        self.textBrowser_4.setText("운행중")
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)  # Update every 1 second

    def stop_operation(self):
        self.textBrowser_4.setText("운행멈춤")
        self.timer.stop()

    def reset_operation(self):
        self.textBrowser_4.setText("운행멈춤")
        self.timer.stop()
        self.elapsed_time = 0
        self.time.display(self.elapsed_time)

    def resume_operation(self):
        self.textBrowser_4.setText("운행중")
        self.timer.start(1000)

    def all_stop_operation(self):
        self.textBrowser_4.setText("전체정지")
        self.timer.stop()
        # Additional all-stop logic can be implemented here

    def update_time(self):
        self.elapsed_time += 1
        self.time.display(self.elapsed_time)


def main(args=None):
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv)

    # Show login dialog first
    login_dialog = LoginDialog()

    if login_dialog.exec_() == QDialog.Accepted:  # If login is successful
        node = fulfillmentGUI()
        node.show()
        sys.exit(app.exec_())
    else:
        sys.exit()


if __name__ == "__main__":
    main()
