/**
* This file is part of the ROS package path_planner which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <sstream>
#include <fstream>

#include <QMessageBox>

#include "PriorityDialogUI.h"


using namespace std;

namespace path_planner_rviz_wp_plugin
{

PriorityDialogUI::PriorityDialogUI(QWidget *parent): QDialog(parent)
{
    ui_.setupUi(this);
    
    x = y = z = 0; 

    boxSliderConnector_.Init(ui_.doubleSpinBox, ui_.horizontalSlider);

    connect(&boxSliderConnector_, SIGNAL(ValueChanged(double)), this, SIGNAL(ValueChanged(double)));
}

PriorityDialogUI::~PriorityDialogUI()
{
    std::cout << "PriorityDialogUI::~PriorityDialogUI()" << std::endl;

}

void PriorityDialogUI::SetPriorityValue(double val)
{
    boxSliderConnector_.SetValue(val);
}


}