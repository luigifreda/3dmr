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

#ifndef ADVANCED_WIDGET_UI_H
#define ADVANCED_WIDGET_UI_H

#include <iostream>
#include <QWidget>
#include "ui_AdvancedWidget.h"


namespace path_planner_rviz_wp_plugin
{

class NavigationDialogUI;

class WaypointsTool;

///	\class AdvancedWidgetUI
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning

class AdvancedWidgetUI : public QWidget
{
    Q_OBJECT

public:
    AdvancedWidgetUI(QWidget *parent = 0);

    ~AdvancedWidgetUI();
    
public: /// < getters

    bool IsAddedToPane() const {return bAddedToPane_;}
    
public: /// < setters     
    
    void SetAddedToPane(bool val) {bAddedToPane_ = val;}
    
    void SetWpTool(WaypointsTool* pWpTool);
    
    void SetConnectionsWithWpTool();
    
public Q_SLOTS:

    void SetPlay(bool val);    
    
    void OpenNavigationDialog();    
    
protected:

    Ui::AdvancedWidget ui_;


protected:

    bool bAddedToPane_;

    WaypointsTool* pWpTool_;
    
    NavigationDialogUI* pNavigationDialogUI_;
};


} // namespace 

#endif