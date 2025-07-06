import os
import rospy
import rospkg
import datetime
import time
#import json

import subprocess
import signal
import psutil
import os




#from inertiallabs_msgs.msg import ins_data
 
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot, QTimer, QRegExp
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QScrollArea, QVBoxLayout
from python_qt_binding.QtGui import QIntValidator, QRegExpValidator, QDoubleValidator

LIDAR_CODE = 1
RADAR_CODE = 2
CAMERA_CODE = 3
CAMERA_COMP_CODE = 4
INS_CODE = 5
IR_CAM_CODE = 6
VEL_CODE = 7

SYNC_CODE = 100
LIDAR_SYNC_CODE = SYNC_CODE + LIDAR_CODE
RADAR_SYNC_CODE = SYNC_CODE + RADAR_CODE
CAMERA_SYNC_CODE = SYNC_CODE + CAMERA_CODE
CAMERA_COMP_SYNC_CODE = SYNC_CODE +CAMERA_COMP_CODE
INS_SYNC_CODE = SYNC_CODE + INS_CODE
IR_CAM_SYNC_CODE = SYNC_CODE + IR_CAM_CODE
VEL_SYNC_CODE = SYNC_CODE + VEL_CODE

code2sensor_dict = {
  LIDAR_SYNC_CODE: "lidar",
  VEL_SYNC_CODE: "vel_lidar",
  RADAR_SYNC_CODE: "radar",
  CAMERA_SYNC_CODE: "camera",
  CAMERA_COMP_SYNC_CODE: "camera_comp",
  INS_SYNC_CODE: "ins",
  IR_CAM_SYNC_CODE: "flircam"
}

code2topic_dict = {
  LIDAR_CODE: "/invz_reflection_0",
  RADAR_CODE: "/esr/markers_pos",
  CAMERA_CODE: "/image_raw",
  CAMERA_COMP_CODE: "/image_raw/compressed",
  INS_CODE: "/Inertial_Labs/sensor_data /Inertial_Labs/ins_data /Inertial_Labs/gps_data",
  IR_CAM_CODE: "/flir_boson/image_raw",
  VEL_CODE: "/velodyne_points"
}

SYNC_PREFIX = "/sensor_sync"

code2topic_sync_dict = {
  LIDAR_SYNC_CODE: SYNC_PREFIX+"/invz_reflection_0",
  RADAR_SYNC_CODE: SYNC_PREFIX+"/esr/markers_pos",
  CAMERA_SYNC_CODE: SYNC_PREFIX+"/image_raw",
  CAMERA_COMP_SYNC_CODE: SYNC_PREFIX+"/image_raw/compressed",
  INS_SYNC_CODE: SYNC_PREFIX+"/Inertial_Labs/sensor_data "+SYNC_PREFIX+"/Inertial_Labs/ins_data "+SYNC_PREFIX+"/Inertial_Labs/gps_data",
  IR_CAM_SYNC_CODE: SYNC_PREFIX+"/flir_boson/image_raw",
  VEL_SYNC_CODE: SYNC_PREFIX+"/velodyne_points"
}

SYNC_LAUNCH_FILE = "roslaunch sensors_synchronizer sensors_synchronizer.launch sensors_topics:="

DEFAULT_DIR = os.path.abspath(".")

 
class CarSensorsRec(Plugin):

    def __init__(self, context):
        super(CarSensorsRec, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CarSensorsRec')
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        #args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #    print 'arguments: ', args
        #    print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_car_sensors_rec'), 'resource', 'CarSensorsRec.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CarSensorsRecUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        
        ###Ignore meanwhile (old plugin code ahead:)
        """
        ## Use me on a later label, maybe timer for recording..
        self._widget.message.setText("Hello ROS!")
        self._widget.message.adjustSize()
        
        rospy.Subscriber("/Inertial_Labs/ins_data", ins_data, self.get_ins_data)
        
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        """
        # important global params:
        self.is_recoding_on = False
        self.is_recording_paused = False
        self.sync_node_running = False
        self.pause_len =  datetime.timedelta(0)
        self.rec_topic_list = []
        self.rec_topic_sync_list = []
        self.rec_topic_sensors_sync_list = []
        self.save_dir_bagfile = DEFAULT_DIR # TODO: check how to avoid abs-path
        self._widget.dir_lineEdit.setText(self.save_dir_bagfile)


        # more gui settings
        self._widget.pauseButton.setEnabled(False)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_timer)
        self._widget.buffer_lineEdit.setValidator(QIntValidator(self._widget))
        self._widget.duration_lineEdit.setValidator(QDoubleValidator(self._widget))
        self._widget.filename_lineEdit.setValidator(QRegExpValidator(QRegExp("^[\w\-. ]+$"),self._widget))


        # setup rec topic list:
        for topic in code2topic_dict.values():
            self.rec_topic_list.append(topic)
        
        # buttons/checkboxes
        self._widget.recButton.clicked.connect(self.recClicked)
        self._widget.pauseButton.clicked.connect(self.pause_resume_recodring)
        self._widget.dir_button.clicked.connect(self.update_save_dir)

        #self._widget.Radar_sync_button.setEnabled(False)
        #self._widget.Radar_sync_checkBox.setEnabled(False)


        self._widget.Lidar_button.clicked.connect(lambda: self.sensorClicked(LIDAR_CODE))
        self._widget.Lidar_checkBox.clicked.connect(lambda:self.sensorClicked(LIDAR_CODE))
        self._widget.Radar_button.clicked.connect(lambda: self.sensorClicked(RADAR_CODE))
        self._widget.Radar_checkBox.clicked.connect(lambda:self.sensorClicked(RADAR_CODE))
        self._widget.Camera_button.clicked.connect(lambda: self.sensorClicked(CAMERA_CODE))
        self._widget.Camera_checkBox.clicked.connect(lambda:self.sensorClicked(CAMERA_CODE))
        self._widget.Camera_comp_button.clicked.connect(lambda: self.sensorClicked(CAMERA_COMP_CODE))
        self._widget.Camera_comp_checkBox.clicked.connect(lambda:self.sensorClicked(CAMERA_COMP_CODE))
        self._widget.Ins_button.clicked.connect(lambda: self.sensorClicked(INS_CODE))
        self._widget.Ins_checkBox.clicked.connect(lambda:self.sensorClicked(INS_CODE))
        self._widget.FLIR_cam_pushButton.clicked.connect(lambda: self.sensorClicked(IR_CAM_CODE))
        self._widget.FLIR_cam_checkBox.clicked.connect(lambda:self.sensorClicked(IR_CAM_CODE))
        self._widget.VelButton.clicked.connect(lambda: self.sensorClicked(VEL_CODE))
        self._widget.VelCheckBox.clicked.connect(lambda:self.sensorClicked(VEL_CODE))

        self._widget.Lidar_sync_button.clicked.connect(lambda: self.sensorClicked(LIDAR_SYNC_CODE))
        self._widget.Lidar_sync_checkBox.clicked.connect(lambda:self.sensorClicked(LIDAR_SYNC_CODE))
        #self._widget.Radar_sync_button.clicked.connect(lambda: self.sensorClicked(RADAR_SYNC_CODE))
        #self._widget.Radar_sync_checkBox.clicked.connect(lambda:self.sensorClicked(RADAR_SYNC_CODE))
        self._widget.Camera_sync_button.clicked.connect(lambda: self.sensorClicked(CAMERA_SYNC_CODE))
        self._widget.Camera_sync_checkBox.clicked.connect(lambda:self.sensorClicked(CAMERA_SYNC_CODE))
        self._widget.Camera_comp_sync_button.clicked.connect(lambda: self.sensorClicked(CAMERA_COMP_SYNC_CODE))
        self._widget.Camera_comp_sync_checkBox.clicked.connect(lambda:self.sensorClicked(CAMERA_COMP_SYNC_CODE))
        self._widget.Ins_sync_button.clicked.connect(lambda: self.sensorClicked(INS_SYNC_CODE))
        self._widget.Ins_sync_checkBox.clicked.connect(lambda:self.sensorClicked(INS_SYNC_CODE))
        self._widget.FLIR_sync_button.clicked.connect(lambda: self.sensorClicked(IR_CAM_SYNC_CODE))
        self._widget.FLIR_sync_checkBox.clicked.connect(lambda:self.sensorClicked(IR_CAM_SYNC_CODE))
        self._widget.vel_sync_button.clicked.connect(lambda: self.sensorClicked(VEL_SYNC_CODE))
        self._widget.vel_sync_checkBox.clicked.connect(lambda:self.sensorClicked(VEL_SYNC_CODE))

        self._widget.rec_All_checkBox.toggled.connect(self.selecet_All)
        self._widget.rec_All_sync_checkBox.toggled.connect(self.selecet_All_sync)

        
        # Add widget to the user interface
        context.add_widget(self._widget)
        

    @Slot()

    def selecet_All(self):
        if self._widget.rec_All_checkBox.isChecked():
            rec_topic_list_all = []
            for topic in code2topic_dict.values():
                rec_topic_list_all.append(topic)
            self.rec_topic_list = rec_topic_list_all
        else:
            self.rec_topic_list = []

        
    def selecet_All_sync(self):
        if self._widget.rec_All_sync_checkBox.isChecked():
            rec_topic_sync_list_all = []
            for topic_sync in code2topic_sync_dict.values():
                rec_topic_sync_list_all.append(topic_sync)
            self.rec_topic_sync_list = rec_topic_sync_list_all
            rec_sensor_sync_list_all = []
            for sensor in code2sensor_dict.values():
                if sensor == 'radar':    ## 
                    continue
                rec_sensor_sync_list_all.append(sensor)
            self.rec_topic_sensors_sync_list = rec_sensor_sync_list_all
        else:
            self.rec_topic_sync_list = []
            self.rec_topic_sensors_sync_list = []

    def is_none_sensors_toggled(self):
        lidar = self._widget.Lidar_checkBox.isChecked()
        radar = self._widget.Radar_checkBox.isChecked()
        ins = self._widget.Ins_checkBox.isChecked()
        camera = self._widget.Camera_checkBox.isChecked()
        camera_comp = self._widget.Camera_comp_checkBox.isChecked()
        flir = self._widget.FLIR_cam_checkBox.isChecked()
        vel = self._widget.VelCheckBox.isChecked()

        lidar_sync = self._widget.Lidar_sync_checkBox.isChecked()
        #radar_sync = self._widget.Radar_sync_checkBox.isChecked()
        ins_sync = self._widget.Ins_sync_checkBox.isChecked()
        camera_sync = self._widget.Camera_sync_checkBox.isChecked()
        camera_comp_sync = self._widget.Camera_comp_sync_checkBox.isChecked()
        vel_sync = self._widget.vel_sync_checkBox.isChecked()
        flir_sync = self._widget.FLIR_sync_checkBox.isChecked()
        
        return  not(lidar or radar or ins or camera or camera_comp or lidar_sync or ins_sync or camera_sync or camera_comp_sync or flir or vel or vel_sync or flir_sync) # or radar_sync


    def recClicked(self):
        if self.is_recoding_on == False:
            # check is there is a checked sensor
            if self.is_none_sensors_toggled():
                self._widget.car_rec_timer.setText("No sensors selected, Please select a sensor to record.")
                self._widget.recButton.toggle()
                return None

            # setting up logic and button
            self.is_recoding_on = True
            self.set_sensor_button_availability(False)
            self.set_advance_settings_availability(False)
            self._widget.recButton.setText("Stop recording")


            self.start_recording()
            self.set_pause_button_availability(True)

            print(" ".join(self.rec_topic_list))

            #staring timer
            self.timer_tik = datetime.datetime.now().time()
            self.timer.start(200)  # every 200 milliseconds
        
        else:
            self.is_recoding_on = False
            self._widget.recButton.setText("Start recording")
            # timer stops inside update_timer(self).
            if self.is_recording_paused:
                self.pause_resume_recodring()
            
            self.set_pause_button_availability(False)
            self.set_sensor_button_availability(True)
            self.set_advance_settings_availability(True)
            self.stop_recording()

    def pause_resume_recodring(self):
        
        rosbag_process = psutil.Process(self.rosbag_handle.pid)
        
        for rosbag_child in rosbag_process.children(recursive=True):
            if self.is_recording_paused:
                rosbag_child.resume()
            else:
                rosbag_child.suspend()
                
        if self.is_recording_paused:
            # handling timer
            self.pause_tok = datetime.datetime.now().time()
            self.pause_len += datetime.datetime.combine(datetime.date.min, self.pause_tok) - datetime.datetime.combine(datetime.date.min, self.pause_tik)
            print("total pause len: " + str(self.pause_len))
            self.timer.start(200)

            rospy.loginfo("Recording resumed")
        else:
            # handling timer
            self.timer.stop()
            self.pause_tik = datetime.datetime.now().time()

            rospy.loginfo("Recording paused")

        self.is_recording_paused = self.is_recording_paused ^ True

        print("recording_paused = " + str(self.is_recording_paused))

    def start_recording(self):
        
        rospy.loginfo('now the sensors recording should start')

        command = "rosbag record"

        # buffer settings
        # buffer_size = str(self._widget.buffer_lineEdit.text()).translate(None, ',') # - this is outdated
        buffer_size = str(self._widget.buffer_lineEdit.text()).replace(',', '')
        if buffer_size == "":
            buffer_size = str(self._widget.buffer_lineEdit.placeholderText())
        command += " -b " + buffer_size

        # self.duration = str(self._widget.duration_lineEdit.text()).translate(None, ',') # - this is outdated
        self.duration = str(self._widget.duration_lineEdit.text()).replace(',', '')
        if self.duration != "":
            command += " --duration=" + self.duration + "m"


        # filename settings
        filename = str(self._widget.filename_lineEdit.text())
        if self._widget.filename_td_checkBox.isChecked():
            filename_arg = " -o "
        else:
            filename_arg = " -O "
        
        if filename != "":
            command += filename_arg + filename

        # dir settings are defined via dir_button and get function
         
        self.save_dir_bagfile = self._widget.dir_lineEdit.text()
        if self.save_dir_bagfile == "":
            self.save_dir_bagfile = DEFAULT_DIR
        
        command += " " + " ".join(self.rec_topic_list)
        command += " " + " ".join(self.rec_topic_sync_list)

        if self.rec_topic_sensors_sync_list !=[]:
            arg = " ".join(self.rec_topic_sensors_sync_list)
            self.ros_sync_node_handle = subprocess.Popen(SYNC_LAUNCH_FILE + "\"" + arg + "\"", stdin=subprocess.PIPE, shell=True)
            self.sync_node_running = True
            print("start: sync node pid is " + str(self.ros_sync_node_handle.pid))

        self.rosbag_handle = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.save_dir_bagfile)

        print("start: rosbag pid is " + str(self.rosbag_handle.pid))

    def stop_recording(self):

        rospy.loginfo("Stopping recording..")
        rosbag_process = psutil.Process(self.rosbag_handle.pid)
                
        for rosbag_child in rosbag_process.children(recursive=True):
            rosbag_child.send_signal(signal.SIGINT)
                
        try:                
            print("stop: pid is " + str(self.rosbag_handle.pid))
        except:
            print("stop: pid is missing!")

        if self.rec_topic_sensors_sync_list !=[]:
            sync_node_process = psutil.Process(self.ros_sync_node_handle.pid)
            for sync_node_child in sync_node_process.children(recursive=True):
                sync_node_child.send_signal(signal.SIGINT)
            
            try:                
                print("stop: pid is " + str(self.ros_sync_node_handle.pid))
            except:
                print("stop: pid is missing!")

        rospy.loginfo("Recording is done.")


    def sensorClicked(self, code):
        if code < SYNC_CODE:
            sensors_topic = code2topic_dict[code]
            if sensors_topic in self.rec_topic_list:
                self.rec_topic_list.remove(sensors_topic)
                print("removed "+sensors_topic)
            else:
                self.rec_topic_list.append(sensors_topic)
                print("added "+sensors_topic)
        else:
            print(code2topic_sync_dict)
            sensors_topic = code2topic_sync_dict[code]
            if sensors_topic in self.rec_topic_sync_list:
                self.rec_topic_sync_list.remove(sensors_topic)
                self.rec_topic_sensors_sync_list.remove(code2sensor_dict[code])
                print("removed "+sensors_topic)
            else:
                self.rec_topic_sync_list.append(sensors_topic)
                self.rec_topic_sensors_sync_list.append(code2sensor_dict[code])
                print("added "+sensors_topic)

    def update_timer(self):
        if self.is_recoding_on:
                     
            self.timer_tok = datetime.datetime.now().time()
            self.rec_timer = - self.pause_len + datetime.datetime.combine(datetime.date.min, self.timer_tok) - datetime.datetime.combine(datetime.date.min, self.timer_tik)
            if self.duration != "":
                if self.rec_timer.total_seconds() >= float(self.duration)*60:
                    time.sleep(0.5)
                    self._widget.recButton.toggle()
                    self.recClicked()

                
            self._widget.car_rec_timer.setText("recording in process.. " + str(self.rec_timer).split('.')[0])

        
        else:
            self.timer.stop()
            self._widget.car_rec_timer.setText("recoding is over. " + str(self.rec_timer).split('.')[0])

    def update_save_dir(self):
        self.save_dir_bagfile = QFileDialog.getExistingDirectory(
            self._widget,
            caption='Select a folder to save the bag file'
        )
        self._widget.dir_lineEdit.setText(self.save_dir_bagfile)

    def set_sensor_button_availability(self,status):
        self._widget.Lidar_button.setEnabled(status)
        self._widget.Lidar_checkBox.setEnabled(status)
        self._widget.Radar_button.setEnabled(status)
        self._widget.Radar_checkBox.setEnabled(status)
        self._widget.Camera_button.setEnabled(status)
        self._widget.Camera_checkBox.setEnabled(status)
        self._widget.Camera_comp_button.setEnabled(status)
        self._widget.Camera_comp_checkBox.setEnabled(status)        
        self._widget.Ins_button.setEnabled(status)
        self._widget.Ins_checkBox.setEnabled(status)
        self._widget.FLIR_cam_pushButton.setEnabled(status)
        self._widget.FLIR_cam_checkBox.setEnabled(status)
        self._widget.VelButton.setEnabled(status)
        self._widget.VelCheckBox.setEnabled(status)

    def set_advance_settings_availability(self,status):
        self._widget.buffer_lineEdit.setEnabled(status)
        self._widget.filename_lineEdit.setEnabled(status)
        self._widget.dir_lineEdit.setEnabled(status)
        self._widget.dir_button.setEnabled(status)
        self._widget.filename_td_checkBox.setEnabled(status)

    def set_pause_button_availability(self,status):
        self._widget.pauseButton.setEnabled(status)
        

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass
 
    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass
 
    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
 
    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
