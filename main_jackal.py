#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
Qt GUI for launching the 3dnav system.
Author: Luigi Freda
"""

import os
import sys
from PyQt5.QtWidgets import (
    QWidget, QToolTip, QDesktopWidget, QPushButton, QApplication)
from PyQt5.QtWidgets import (
    QHBoxLayout, QGroupBox, QDialog, QVBoxLayout, QGridLayout, QFileDialog)
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QLabel)
from PyQt5 import QtCore
from PyQt5.QtGui import QFont
import subprocess
from pathlib import Path


# --------------------------------------------
# small lib for getting value in a config file 
def lines_that_equal(line_to_match, fp):
    return [line for line in fp if line == line_to_match]
def lines_that_contain(string, fp):
    return [line for line in fp if string in line]
def lines_that_start_with(string, fp):
    return [line for line in fp if line.startswith(string)]
def lines_that_end_with(string, fp):
    return [line for line in fp if line.endswith(string)]

def get_config_var_value_from_config_file(filename, varname):
    start_code="=\"" # ="
    end_code="\"" # "  
    res = ''
    with open(filename, "r") as fp:
        for line in lines_that_contain(varname, fp):
            start_index = line.find(start_code) + len(start_code)
            end_index = start_index + line[start_index:].find(end_code)
            res = line[start_index:end_index]
            #print(f'line: {line}')
    #print(f'res: {res}')
    return res
# --------------------------------------------

# get the location of this file!
__location__ = os.path.realpath(os.path.join(
    os.getcwd(), os.path.dirname(__file__)))

SCRIPT_DIRECTORY = Path(__file__).resolve().parent

kDefaultGazeboWorldFolder = __location__ + '/jackal_ws/src/gazebo/jackal/jackal_3dmain/worlds'
print(f'kDefaultGazeboWorldFolder: {kDefaultGazeboWorldFolder}')
kDefaultGazeboWorldName = 'jackal_baylands.world'
kDefaultGazeboExplWorldName = 'jackal_race.world'

# --------------------------------------------
# run-command utils


def run_command(command, debug=False):
    """ runs command and returns the output."""
    if debug:
        print("$ {}".format(command))
    #p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True, executable='/bin/bash')
    p = subprocess.Popen(command, shell=True, executable='/bin/bash')
    # return iter(p.stdout.readline, b'')


def executeCommand(command, stderr=None, stdout=None, debug=False):
    """ Raises an error when a command fails. """
    if debug:
        print("$ {}".format(command))
    subprocess.check_call(command, stderr=stderr,
                          stdout=stdout, env=dict(os.environ), shell=True)


def executeCommands(commands, stderr=None, stdout=None, debug=False):
    for command in commands:
        executeCommand(command, stderr=stderr, stdout=stdout, debug=debug)


def executeCommandNoCheck(command, stderr=None, stdout=None, debug=False):
    """ Does not raise an error when a command fails. """
    if debug:
        print("$ {}".format(command))
    subprocess.call(command, stderr=stderr, stdout=stdout,
                    env=dict(os.environ), shell=True)


def executeCommandsNoCheck(commands, stderr=None, stdout=None, debug=False):
    for command in commands:
        executeCommandNoCheck(command, stderr=stderr,
                              stdout=stdout, debug=debug)

# --------------------------------------------
# Utils


def getBaseFileNameNoExt(filename):
    base = os.path.basename(filename)
    os.path.splitext(base)
    return os.path.splitext(base)[0]

# --------------------------------------------
# Main Widget


class MainWidget(QWidget):

    def __init__(self):
        super().__init__()

        self.cmdDebug = False

        self.mainWidgetWidth = 400
        self.mainWidgetHeight = 400

        self.gazebo_worlds_folder = kDefaultGazeboWorldFolder 
        self.gazebo_world_name = kDefaultGazeboWorldName
        self.is_gazebo_launched = 0 
        self.gazebo_mode = 0 # normal:1 launch Gazebo in normal mode, nheadless:0 launch VREP in headless mode (hidden) 
        
        self.exploration_world_name = kDefaultGazeboExplWorldName
        self.exploration_worlds_folder = kDefaultGazeboWorldFolder

        self.exploration_enable = True
        self.exploration_auto_launch = True        
        
        self.teb_enable = 0
        self.voxblox_enable = 0
        self.elevation_mapping_enable = 0        
        
        self.sourceCmd = 'source source_all.bash && '

        self.initUI()
        self.center()

    def initUI(self):

        QToolTip.setFont(QFont('SansSerif', 10))

        self.setGeometry(0, 0, self.mainWidgetWidth, self.mainWidgetHeight)
        self.setWindowTitle('3DMR  -  JACKAL')

        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        #
        self.btn_gazebo = QPushButton('Launch gazebo', self)
        self.btn_gazebo.setToolTip('Test separately gazebo simulator.')

        self.btn_gazebo_world = QPushButton('Select gazebo world', self)
        self.btn_gazebo_world.setToolTip('Select a gazebo world. A default world is already set.')

        #
        self.btn_nav = QPushButton('Launch navigation', self)
        self.btn_nav.setToolTip('Launch navigation system: segmentation, trav analysis, path planner, trajectory control')
        
        #
        # self.btn_teb_nav = QPushButton('Launch TEB navigation', self)
        # self.btn_teb_nav.setToolTip('Launch reduced navigation system for TEB + TEB local planner')

        #
        self.checkbtn_voxblox_enable = QCheckBox(
            'Use voxblox mapping', self)
        self.checkbtn_voxblox_enable.setToolTip(
            'Enable/disable voxblox for mapping instead of using octomap.')
        self.checkbtn_voxblox_enable.toggled.connect(
            self.slot_voxblox_enable)
        self.checkbtn_voxblox_enable.setChecked(self.voxblox_enable)
        
        #
        self.checkbtn_teb_enable = QCheckBox(
            'Use TEB local planner', self)
        self.checkbtn_teb_enable.setToolTip(
            'Enable/disable TEB as local planner and trajectory control.')
        self.checkbtn_teb_enable.toggled.connect(
            self.slot_teb_enable)
        self.checkbtn_teb_enable.setChecked(self.teb_enable)    
        
        #
        self.checkbtn_elevation_mapping_enable = QCheckBox(
            'Use elevation mapping', self)
        self.checkbtn_elevation_mapping_enable.setToolTip(
            'Enable/disable elevation mapping.')
        self.checkbtn_elevation_mapping_enable.toggled.connect(
            self.slot_elevation_mapping_enable)
        self.checkbtn_elevation_mapping_enable.setChecked(self.elevation_mapping_enable)            
          
          
        #
        self.btn_exploration = QPushButton('Launch exploration', self)
        self.btn_exploration.setToolTip('Launch the exploration system')

        self.btn_exploration_world = QPushButton('Select exploration world', self)
        self.btn_exploration_world.setToolTip('Select an exploration world in one of the subfolder of maps. A default world is already set in the script sim_launcher_exploration.')

        self.checkbtn_exploration_enable = QCheckBox(
            'Enable exploration', self)
        self.checkbtn_exploration_enable.setToolTip(
            'Enable/disable the exploration agent. Disabling can be useful for building and saving a map in a selected world (please, read the documentation).')
        self.checkbtn_exploration_enable.toggled.connect(
            self.slot_exploration_enable)
        self.checkbtn_exploration_enable.setChecked(self.exploration_enable)

        self.checkbtn_exploration_auto_launch = QCheckBox(
            'Auto launch exploration', self)
        self.checkbtn_exploration_auto_launch.setToolTip(
            'Auto launch exploration.')
        self.checkbtn_exploration_auto_launch.toggled.connect(
            self.slot_exploration_auto_launch)
        self.checkbtn_exploration_auto_launch.setChecked(
            self.exploration_auto_launch)
        
                        
        #
        self.hbox_gazebo = QGroupBox()
        #hbox_layout_gazebo = QHBoxLayout()
        #self.hbox_gazebo.setLayout(hbox_layout_gazebo)
        gazebo_mode_help_string = 'normal:1 launch Gazebo in normal mode \nheadless:0 launch VREP in headless mode (hidden) '
        self.label_gazebo_mode = QLabel()
        self.label_gazebo_mode.setText("Mode")
        self.label_gazebo_mode.setToolTip(gazebo_mode_help_string)
        self.cb_gazebo_mode = QComboBox()
        self.cb_gazebo_mode.setToolTip(gazebo_mode_help_string)
        self.cb_gazebo_mode.addItems(["headless", "normal"])
        self.cb_gazebo_mode.setCurrentIndex(self.gazebo_mode)
        self.cb_gazebo_mode.currentIndexChanged.connect(
            self.slot_gazebo_mode_change)
        #hbox_layout_gazebo.addWidget(self.label_gazebo_mode)
        #hbox_layout_gazebo.addWidget(self.cb_gazebo_mode)
        
        #
        self.btn_save_map = QPushButton('Save nav map', self)
        self.btn_save_map.setToolTip('Save the built navigation map')

        #
        self.btn_load_map = QPushButton('Load nav map', self)
        self.btn_load_map.setToolTip('Load the built navigation map')

        #
        self.btn_kill = QPushButton('Kill all', self)
        self.btn_kill.setToolTip('Kill all nodes of the system')

        # set layouts
        self.create_exploration_layout()          
        self.create_nav_layout()  
        self.create_gazebo_layout()        
        #self.mainLayout.addWidget(self.hbox_gazebo)

        self.mainLayout.addWidget(self.btn_save_map)
        self.mainLayout.addWidget(self.btn_load_map)
        self.mainLayout.addWidget(self.btn_kill)

        # connections
        self.btn_gazebo.clicked.connect(self.slot_gazebo)
        self.btn_gazebo_world.clicked.connect(self.slot_gazebo_world)
        
        self.btn_nav.clicked.connect(self.slot_nav)
     
        self.btn_exploration.clicked.connect(self.slot_exploration)
        self.btn_exploration_world.clicked.connect(self.slot_exploration_world)
     
        self.btn_save_map.clicked.connect(self.slot_save_map)
        self.btn_load_map.clicked.connect(self.slot_load_map)

        self.btn_kill.clicked.connect(self.slot_kill)

        # show the widget
        self.show()

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def create_gazebo_layout(self):
        self.horizontalGroupBox_gazebo = QGroupBox("Gazebo")
        self.gridLayout_gazebo = QGridLayout()
        #self.gridLayout_gazebo.setColumnStretch(1, 3)
        #self.gridLayout_gazebo.setColumnStretch(2, 3)

        self.gridLayout_gazebo.addWidget(self.btn_gazebo, 0, 0)
        self.gridLayout_gazebo.addWidget(self.btn_gazebo_world, 0, 1)     

        self.gridLayout_gazebo.addWidget(self.label_gazebo_mode, 1, 0)
        self.gridLayout_gazebo.addWidget(self.cb_gazebo_mode, 1, 1)        
        
        self.horizontalGroupBox_gazebo.setLayout(
            self.gridLayout_gazebo)
                
        self.mainLayout.addWidget(self.horizontalGroupBox_gazebo)
        
    def create_nav_layout(self):
        self.horizontalGroupBox_nav = QGroupBox("Navigation")
        self.gridLayout_nav = QGridLayout()
        #self.gridLayout_nav.setColumnStretch(1, 3)
        #self.gridLayout_nav.setColumnStretch(2, 3)

        self.gridLayout_nav.addWidget(self.btn_nav, 1, 0)
        self.gridLayout_nav.addWidget(self.checkbtn_voxblox_enable, 2, 0)  
        self.gridLayout_nav.addWidget(self.checkbtn_teb_enable, 3, 0)        
        self.gridLayout_nav.addWidget(self.checkbtn_elevation_mapping_enable, 4, 0)                 
        
        self.horizontalGroupBox_nav.setLayout(self.gridLayout_nav)
        self.mainLayout.addWidget(self.horizontalGroupBox_nav)
        
    def create_exploration_layout(self):
        self.horizontalGroupBox_expl = QGroupBox("Exploration")
        self.gridLayout_expl = QGridLayout()
        #self.gridLayout_expl.setColumnStretch(1, 3)
        #self.gridLayout_expl.setColumnStretch(2, 3)

        self.gridLayout_expl.addWidget(self.btn_exploration, 0, 0)
        self.gridLayout_expl.addWidget(self.btn_exploration_world, 0, 1)
        self.gridLayout_expl.addWidget(self.checkbtn_exploration_enable, 1, 1)
        self.gridLayout_expl.addWidget(self.checkbtn_exploration_auto_launch, 2, 1)

        self.horizontalGroupBox_expl.setLayout(self.gridLayout_expl)
        self.mainLayout.addWidget(self.horizontalGroupBox_expl)
                
# slots

    def slot_gazebo(self):
        print('launch gazebo')
        cmd_envs = ''
        cmd_envs += 'export LAUNCH_GAZEBO_MODE_ENV=' + str(self.gazebo_mode) + '; '
        cmd = self.sourceCmd + cmd_envs + \
            'rosrun jackal_3dmain jackal_ugvs.sh ' + self.gazebo_world_name
        print(cmd)
        run_command(cmd)
        self.is_gazebo_launched = 1

    def slot_gazebo_world(self):
        print(f'select gazebo world in {self.gazebo_worlds_folder}')
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Select gazebo world", self.gazebo_worlds_folder, "Gazebo worlds (*.world)", options=options)
        if fileName:
            self.gazebo_world_name = os.path.basename(fileName) # getBaseFileNameNoExt(fileName)
            print('world: ', self.gazebo_world_name)
        self.slot_gazebo()
        
    def slot_nav(self):
        print('launch path planner system')
        cmd_envs = ''
        if self.is_gazebo_launched: 
            cmd_envs += 'export LAUNCH_GAZEBO_ENV=0; '          
        cmd_envs += 'export LAUNCH_GAZEBO_MODE_ENV=' + str(self.gazebo_mode) + '; '          
        if self.teb_enable: 
            cmd_envs += 'export ENABLE_TEB_LOCAL_PLANNER=1; '
        if self.voxblox_enable: 
            cmd_envs += 'export ENABLE_VOXBLOX=1; '    
        if self.elevation_mapping_enable:
            cmd_envs += 'export ENABLE_ELEVATION_MAPPING=1; '                                    
        cmd = self.sourceCmd + cmd_envs + 'rosrun jackal_3dnav sim_launcher_navigation'
        print(cmd)
        run_command(cmd)
        

    def slot_exploration(self):
        print('launch exploration')
        cmd_envs = ''
        if not self.exploration_enable:
            cmd_envs += 'export ENABLE_EXPLORATION_ENV=0; '  # must be integer
        if not self.exploration_auto_launch:
            cmd_envs += 'export EXPL_AUTO_LAUNCH_ENV=0; '    # must be integer
        cmd_envs += 'export LAUNCH_GAZEBO_MODE_ENV=' + str(self.gazebo_mode) + '; '
        cmd = self.sourceCmd + cmd_envs + \
            'rosrun jackal_3dexplorer sim_launcher_exploration ' + self.exploration_world_name
        print(cmd)
        run_command(cmd)
                
    def slot_exploration_world(self):
        print('select expl. world')
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Select exploration world", self.exploration_worlds_folder, "Gazebo worlds (*.world)", options=options)
        if fileName:
            self.exploration_world_name = getBaseFileNameNoExt(fileName)
            print('world: ', self.exploration_world_name)
        self.slot_exploration()


    def slot_exploration_enable(self):
        self.exploration_enable = self.checkbtn_exploration_enable.isChecked()
        print('exploration_enable: ', self.exploration_enable)

    def slot_exploration_auto_launch(self):
        self.exploration_auto_launch = self.checkbtn_exploration_auto_launch.isChecked()
        print('exploration_auto_launch: ', self.exploration_auto_launch)

                
    def slot_gazebo_mode_change(self, i):
        print('change gazebo mode')
        self.gazebo_mode = i
        print('Items in the list are :')
        for count in range(self.cb_gazebo_mode.count()):
            print(self.cb_gazebo_mode.itemText(count))
        print('Current index', self.gazebo_mode,
              'selection changed ', self.cb_gazebo_mode.currentText())        
        
    def slot_voxblox_enable(self):
        self.voxblox_enable = self.checkbtn_voxblox_enable.isChecked()
        print('voxblox_enable: ', self.voxblox_enable)
        
    def slot_teb_enable(self):
        self.teb_enable = self.checkbtn_teb_enable.isChecked()
        print('teb_enable: ', self.teb_enable)        
        
    def slot_elevation_mapping_enable(self):
        self.elevation_mapping_enable = self.checkbtn_elevation_mapping_enable.isChecked()
        print('elevation_mapping_enable: ', self.elevation_mapping_enable)
                
    #
    def slot_save_map(self):
        print('save_map')
        cmd = self.sourceCmd + 'rosrun path_planner save_map'
        print(cmd)
        run_command(cmd)

    def slot_load_map(self):
        print('load_map')
        cmd = self.sourceCmd + 'rosrun path_planner load_map'
        print(cmd)
        run_command(cmd)

    def slot_kill(self):
        print('kill all nodes')
        cmd = self.sourceCmd + 'rosrun jackal_3dnav kill_gazebo_sim'
        print(cmd)
        run_command(cmd)
        self.is_gazebo_launched = 0 

if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = MainWidget()
    sys.exit(app.exec_())
