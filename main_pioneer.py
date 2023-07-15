#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# This file is part of the framework 3dpatrolling.
#
# Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>
# For more information see <https://github.com/luigifreda/3dmr>
#
# 3dpatrolling is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# 3dpatrolling is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with 3DMR. If not, see <http://www.gnu.org/licenses/>.

"""
Qt GUI for launching the patrolling system or the path planning system.
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

# get the location of this file!
__location__ = os.path.realpath(os.path.join(
    os.getcwd(), os.path.dirname(__file__)))

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
        self.mainWidgetHeight = 350

        self.exploration_pioneer_WorldName = ''
        self.exploration_pioneer_WorldsFolder = __location__ + \
            '/pioneer_ws/src/pioneer_nav/vrep_pioneer_simulation/data'
        
        self.exploration_pioneer_enable = True
        self.exploration_auto_launch = True           

        self.teb_enable = 0
        self.voxblox_enable = 0
        
        self.sourceCmd = 'source source_all.bash; '

        self.initUI()
        self.center()

    def initUI(self):

        QToolTip.setFont(QFont('SansSerif', 10))

        self.setGeometry(0, 0, self.mainWidgetWidth, self.mainWidgetHeight)
        self.setWindowTitle('3DMR - PIONEER')

        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)


        #           PIONEER
        self.btn_exploration_pioneer = QPushButton('Launch exploration', self)
        self.btn_exploration_pioneer.setToolTip('Launch the exploration system')
        #self.btn_rviz_pioneer = QPushButton('Open RVIZ', self)
        #self.btn_rviz_pioneer.setToolTip('Launch RVIZ')

        self.checkbtn_expl_pioneer_enable = QCheckBox('Enable exploration', self)
        self.checkbtn_expl_pioneer_enable.setToolTip('Enable/disable exploration')
        self.checkbtn_expl_pioneer_enable.toggled.connect(self.slot_exploration_pioneer_enable)
        self.checkbtn_expl_pioneer_enable.setChecked(self.exploration_pioneer_enable)

        self.checkbtn_exploration_auto_launch = QCheckBox(
            'Auto launch exploration', self)
        self.checkbtn_exploration_auto_launch.setToolTip(
            'Auto launch exploration.')
        self.checkbtn_exploration_auto_launch.toggled.connect(
            self.slot_exploration_auto_launch)
        self.checkbtn_exploration_auto_launch.setChecked(
            self.exploration_auto_launch)
        
        self.btn_exploration_pioneer_world = QPushButton('Select exploration world', self)
        self.btn_exploration_pioneer_world.setToolTip('Select an exploration world in one of the subfolder of maps. A default world is already set in the script sim_launcher_exploration.')

        self.btn_path_planner = QPushButton('Launch navigation', self)
        self.btn_path_planner.setToolTip('Launch vrep_pioneer_simulation system')

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
        
        
        # common commands 
        self.btn_save_map = QPushButton('Save map', self)
        self.btn_save_map.setToolTip('Save the built map')

        self.btn_load_map = QPushButton('Load map', self)
        self.btn_load_map.setToolTip('Load the built map')
        
        self.btn_kill = QPushButton('Kill all', self)
        self.btn_kill.setToolTip('Kill all nodes of the system')

        # set layouts

        self.create_exploration_layout()
        self.create_nav_layout()
        self.mainLayout.addWidget(self.btn_save_map)
        self.mainLayout.addWidget(self.btn_load_map)
        self.mainLayout.addWidget(self.btn_kill)

        # connections
        self.btn_exploration_pioneer.clicked.connect(self.slot_exploration_pioneer)
        self.btn_exploration_pioneer_world.clicked.connect(self.slot_expl_pioneer_world)
        #self.btn_rviz_pioneer.clicked.connect(self.slot_rviz_pioneer)

        self.btn_path_planner.clicked.connect(self.slot_path_planner)
        
        self.btn_save_map.clicked.connect(self.slot_save_map)
        self.btn_load_map.clicked.connect(self.slot_load_map)
        self.btn_kill.clicked.connect(self.slot_kill)

        # show the widget
        self.show()

    def create_exploration_layout(self):
        self.horizontalGroupBox_expl_pioneer = QGroupBox(
            "Exploration - Pioneer")
        self.gridLayout_expl_pioneer = QGridLayout()
        #self.gridLayout_expl.setColumnStretch(1, 3)
        #self.gridLayout_expl.setColumnStretch(2, 3)

        self.gridLayout_expl_pioneer.addWidget(self.btn_exploration_pioneer, 0, 0)
        self.gridLayout_expl_pioneer.addWidget(self.btn_exploration_pioneer_world, 0, 1)
        #self.gridLayout_expl_pioneer.addWidget(self.btn_rviz_pioneer, 1, 0)
        self.gridLayout_expl_pioneer.addWidget(self.checkbtn_expl_pioneer_enable, 1, 1)
        self.gridLayout_expl_pioneer.addWidget(self.checkbtn_exploration_auto_launch, 2, 1)
        
        self.horizontalGroupBox_expl_pioneer.setLayout(
            self.gridLayout_expl_pioneer)
        self.mainLayout.addWidget(self.horizontalGroupBox_expl_pioneer)

    def create_nav_layout(self):
        self.horizontalGroupBox_pp = QGroupBox("Navigation")
        self.gridLayout_pp = QGridLayout()
        #self.gridLayout_pp.setColumnStretch(1, 3)
        #self.gridLayout_pp.setColumnStretch(2, 3)

        self.gridLayout_pp.addWidget(self.btn_path_planner, 1, 0)

        self.gridLayout_pp.addWidget(self.checkbtn_voxblox_enable, 2, 0)  
        self.gridLayout_pp.addWidget(self.checkbtn_teb_enable, 3, 0)  
        
        self.horizontalGroupBox_pp.setLayout(self.gridLayout_pp)
        self.mainLayout.addWidget(self.horizontalGroupBox_pp)

# slots

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def slot_exploration_pioneer(self):
        print('launch exploration')
        cmd_env = ''
        if not self.exploration_pioneer_enable:
            cmd_envs += 'export ENABLE_EXPLORATION_ENV=0; '   # must be integer 
        if not self.exploration_auto_launch:
            cmd_envs += 'export EXPL_AUTO_LAUNCH_ENV=0; '     # must be integer        
        cmd = self.sourceCmd + cmd_env + \
            'rosrun pioneer_3dexplorer sim_launcher_exploration ' + self.exploration_pioneer_WorldName
        print(cmd)
        run_command(cmd)  

    def slot_expl_pioneer_world(self):
        print('select expl. world')
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Select exploration world", self.exploration_pioneer_WorldsFolder, "V-REP scenes (*.ttt)", options=options)
        if fileName:
            self.exploration_pioneer_WorldName = getBaseFileNameNoExt(fileName)
            print('world: ', self.exploration_pioneer_WorldName)
        self.slot_exploration_pioneer()

    def slot_exploration_pioneer_enable(self):
        self.exploration_pioneer_enable = self.checkbtn_expl_pioneer_enable.isChecked()
        print('exploration_pioneer_enable: ', self.exploration_pioneer_enable)

    def slot_exploration_auto_launch(self):
        self.exploration_auto_launch = self.checkbtn_exploration_auto_launch.isChecked()
        print('exploration_auto_launch: ', self.exploration_auto_launch)
        
    def slot_path_planner(self):
        print('launch path planner system')
        cmd_envs = ''
        #cmd_envs += 'export LAUNCH_VREP_MODE_ENV=' + str(self.vrep_mode) + '; '
        if self.teb_enable: 
            cmd_envs += 'export ENABLE_TEB_LOCAL_PLANNER=1; '
        if self.voxblox_enable: 
            cmd_envs += 'export ENABLE_VOXBLOX=1; '         
        cmd = self.sourceCmd + cmd_envs + 'rosrun pioneer_3dnav sim_launcher_navigation'
        print(cmd)
        run_command(cmd)

    def slot_rviz_pioneer(self):
        print('Launching RVIZ')
        cmd = self.sourceCmd + 'roslaunch pioneer_3dnav rviz_sim_path_planning.launch'
        print(cmd)
        run_command(cmd)

    def slot_voxblox_enable(self):
        self.voxblox_enable = self.checkbtn_voxblox_enable.isChecked()
        print('voxblox_enable: ', self.voxblox_enable)
        
    def slot_teb_enable(self):
        self.teb_enable = self.checkbtn_teb_enable.isChecked()
        print('teb_enable: ', self.teb_enable)    
        
    def slot_save_map(self):
        print('save_map')
        cmd = self.sourceCmd + 'rosrun patrolling3d_sim save_map'
        print(cmd)
        run_command(cmd)

    def slot_load_map(self):
        print('load_map')
        cmd = self.sourceCmd + 'rosrun patrolling3d_sim load_map'
        print(cmd)
        run_command(cmd)

    def slot_kill(self):
        print('kill all nodes')
        cmd = self.sourceCmd + 'rosrun patrolling3d_sim kill_vrep_sim'
        print(cmd)
        run_command(cmd)


if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = MainWidget()
    sys.exit(app.exec_())
