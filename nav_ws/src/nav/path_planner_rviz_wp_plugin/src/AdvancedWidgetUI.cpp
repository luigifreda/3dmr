/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Alcor Lab (La Sapienza University)
* For more information see <https://github.com/luigifreda/3dmr>
*
* 3DMR is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DMR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DMR. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <sstream>
#include <fstream>

#include <QMessageBox>

#include "AdvancedWidgetUI.h"
#include "NavigationDialogUI.h"
#include "WaypointsTool.h"


using namespace std;

namespace path_planner_rviz_wp_plugin
{

AdvancedWidgetUI::AdvancedWidgetUI(QWidget *parent) : QWidget(parent)
{
    ui_.setupUi(this);

    bAddedToPane_ = false;

    pWpTool_ = 0;
    pNavigationDialogUI_ = 0;

    connect(ui_.navigationPushButton, SIGNAL(pressed()), this, SLOT(OpenNavigationDialog()));
}

AdvancedWidgetUI::~AdvancedWidgetUI()
{
    std::cout << "AdvancedWidgetUI::~AdvancedWidgetUI()" << std::endl;

}

void AdvancedWidgetUI::SetWpTool(WaypointsTool* pWpTool)
{
    ROS_INFO_STREAM("AdvancedWidgetUI::SetWpTool()");
    pWpTool_ = pWpTool;
    SetConnectionsWithWpTool();
}

void AdvancedWidgetUI::SetConnectionsWithWpTool()
{
    if (pWpTool_)
    {
        ROS_INFO_STREAM("AdvancedWidgetUI::SetConnections()");
        connect(ui_.playButton, SIGNAL(clicked(bool)), this, SLOT(SetPlay(bool)));
    }
}

void AdvancedWidgetUI::SetPlay(bool val)
{
    if(!pWpTool_) return; 
    
    if (val)
    {
        ROS_INFO_STREAM("AdvancedWidgetUI::SetPlay() - patrolling restart");
        pWpTool_->patrollingTaskRestart();
        pWpTool_->explorationTaskRestart();
        pWpTool_->markerAddRobotTextMsg("Patrolling/Exploration Restart");
    }
    else
    {
        ROS_INFO_STREAM("AdvancedWidgetUI::SetPlay() - patrolling pause");
        pWpTool_->patrollingTaskPause();
        pWpTool_->explorationTaskPause();
        pWpTool_->markerAddRobotTextMsg("Patrolling/Exploration Pause");
    }
}


void AdvancedWidgetUI::OpenNavigationDialog()
{
    if(!pNavigationDialogUI_)
    {
        pNavigationDialogUI_ = new NavigationDialogUI(this);
        if(pWpTool_)    pNavigationDialogUI_->SetWpTool(pWpTool_);
        pNavigationDialogUI_->show();
    }
    else
    {
        if (pNavigationDialogUI_->isVisible())
        {
            pNavigationDialogUI_->close();
        }
        else
        {
            pNavigationDialogUI_->show();
        }
    }
}

}