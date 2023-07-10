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

#ifndef NAVIGATION_DIALOG_UI_H
#define NAVIGATION_DIALOG_UI_H

#include <iostream>
#include <QWidget>
#include "ui_NavigationDialog.h"

#include "BoxSliderConnector.h"

namespace path_planner_rviz_wp_plugin
{

class WaypointsTool;

///	\class NavigationDialogUI
///	\author Luigi Freda
///	\brief 
///	\note
/// 	\todo 
///	\date
///	\warning
class NavigationDialogUI: public QDialog
{
    
//https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/    
    
    Q_OBJECT
    
    
    static const double kNormalRadiusStep; 
    static const double kNormalRadiusMin;
    static const double kNormalRadiusMax;
    static const double kNormalRadiusDefault; 
    static const double kNormalRadiusModeNormal;
    static const double kNormalRadiusModeStairs;
    
    static const double kRssMinValueStep; 
    static const double kRssMinValueMin;
    static const double kRssMinValueMax;
    static const double kRssMinValueDefault;     
    
    static const double kDynamicReconfigureTimeout;
    

public:
    
    NavigationDialogUI(QWidget *parent = 0);

    ~NavigationDialogUI();

public Q_SLOTS:

    void SetNormalRadiusValue(double val);
 
    void SendNormalRadiusValue(double val);
    
    void SendNormalRadiusNormalMode();    
    void SendNormalRadiusStairsMode();
    
    void SetClosestObstVelReductionEnable();    
    void SetClosestObstVelReductionDisable();  
    
    void SetRssEnable();    
    void SetRssDisable();      
    
    void RssLoad();
    void RssSave();
           
    void SetMinRssValueEnable(double val);
    
Q_SIGNALS:

    double ValueChanged(double);


public: /// < getters
    
    
public: /// < setters    
    
    void SetRadiusValue(double val) { boxSliderNormalRadiusConnector_.SetValue(val); }    
    
    void SetWpTool(WaypointsTool* pWpTool);    
    
    void SetConnectionsWithWpTool();    

protected:

    Ui::NavigationDialog ui_;

    WaypointsTool* pWpTool_;

protected:

    BoxSliderConnector boxSliderNormalRadiusConnector_;
    BoxSliderConnector boxSliderMinRssValueConnector_;
    
};


} // namespace 

#endif