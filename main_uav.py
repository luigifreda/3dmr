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

        self.exploration_WorldName = 'flat.world'
        self.exploration_WorldsFolder = __location__ + \
            '/exploration_ws/src/interface_nbvp_rotors/worlds'
        
        self.enable_multi_exploration = True      
        
        self.sourceCmd = 'source source_all.bash; '

        self.initUI()
        self.center()

    def initUI(self):

        QToolTip.setFont(QFont('SansSerif', 10))

        self.setGeometry(0, 0, self.mainWidgetWidth, self.mainWidgetHeight)
        self.setWindowTitle('3DMR - UAV')

        self.mainLayout = QVBoxLayout()
        self.setLayout(self.mainLayout)

        #  
        self.btn_exploration = QPushButton('Launch exploration', self)
        self.btn_exploration.setToolTip('Launch the exploration system')

        self.checkbtn_expl_enable = QCheckBox('Enable multi', self)
        self.checkbtn_expl_enable.setToolTip('Enable/disable multi-robot exploration')
        self.checkbtn_expl_enable.toggled.connect(self.slot_enable_multi_exploration)
        self.checkbtn_expl_enable.setChecked(self.enable_multi_exploration)
        
        self.btn_exploration_world = QPushButton('Select exploration world', self)
        self.btn_exploration_world.setToolTip('Select an exploration world in one of the subfolder of maps. A default world is already set in the script sim_launcher_exploration.')

        
        # common commands 
        
        self.btn_kill = QPushButton('Kill all', self)
        self.btn_kill.setToolTip('Kill all nodes of the system')

        # set layouts

        self.createExplorationPioneerLayout()
        self.mainLayout.addWidget(self.btn_kill)

        # connections
        self.btn_exploration.clicked.connect(self.slot_exploration)
        self.btn_exploration_world.clicked.connect(self.slot_expl_world)
        
        self.btn_kill.clicked.connect(self.slot_kill)

        # show the widget
        self.show()

    def createExplorationPioneerLayout(self):
        self.horizontalGroupBox_expl = QGroupBox(
            "Exploration - UAV")
        self.gridLayout_expl = QGridLayout()
        #self.gridLayout_expl.setColumnStretch(1, 3)
        #self.gridLayout_expl.setColumnStretch(2, 3)

        self.gridLayout_expl.addWidget(self.btn_exploration, 0, 0)
        self.gridLayout_expl.addWidget(self.btn_exploration_world, 0, 1)
        self.gridLayout_expl.addWidget(self.checkbtn_expl_enable, 1, 1)

        
        self.horizontalGroupBox_expl.setLayout(
            self.gridLayout_expl)
        self.mainLayout.addWidget(self.horizontalGroupBox_expl)

# slots

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def slot_exploration(self):
        print('launch exploration')
        cmd_env = ''
        if self.enable_multi_exploration:    
            cmd = self.sourceCmd + cmd_env + \
                'roslaunch interface_nbvp_rotors multiagent_flat_exploration.launch world_name:=' + self.exploration_WorldName            
        else: 
            cmd = self.sourceCmd + cmd_env + \
                'roslaunch interface_nbvp_rotors flat_exploration.launch world_name:=' + self.exploration_WorldName                
        print(cmd)
        run_command(cmd)  

    def slot_expl_world(self):
        print('select expl. world')
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Select exploration world", self.exploration_WorldsFolder, "Gazebo worlds (*.world)", options=options)
        if fileName:
            self.exploration_WorldName = getBaseFileNameNoExt(fileName)
            print('world: ', self.exploration_WorldName)
        self.slot_exploration()

    def slot_enable_multi_exploration(self):
        self.enable_multi_exploration = self.checkbtn_expl_enable.isChecked()
        print('enable_multi_exploration: ', self.enable_multi_exploration)

    def slot_kill(self):
        print('kill all nodes')
        cmd = self.sourceCmd + 'rosrun gazebo_utils kill_gazebo_sim'
        print(cmd)
        run_command(cmd)


if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = MainWidget()
    sys.exit(app.exec_())
