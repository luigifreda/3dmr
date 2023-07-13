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

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */
#include <future>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <sstream>      // std::stringstrea

#include <QtCore>
#include <QMessageBox>
#include <QFuture>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtConcurrent/QtConcurrent>
#include <QButtonGroup>
#endif

#include "NavigationDialogUI.h"
#include "WaypointsTool.h"


using namespace std;

namespace path_planner_rviz_wp_plugin
{


const double NavigationDialogUI::kNormalRadiusStep    = 0.05; 
const double NavigationDialogUI::kNormalRadiusMin     = 0.1;
const double NavigationDialogUI::kNormalRadiusMax     = 1.0;

const double NavigationDialogUI::kNormalRadiusDefault = 0.2; 
const double NavigationDialogUI::kNormalRadiusModeNormal = 0.2;
const double NavigationDialogUI::kNormalRadiusModeStairs = 0.4;

const double NavigationDialogUI::kRssMinValueStep = 1; 
const double NavigationDialogUI::kRssMinValueMin  = -100;
const double NavigationDialogUI::kRssMinValueMax  = 0;
const double NavigationDialogUI::kRssMinValueDefault = -80;     

const double NavigationDialogUI::kDynamicReconfigureTimeout = 2;

NavigationDialogUI::NavigationDialogUI(QWidget *parent): QDialog(parent)
{
    ui_.setupUi(this);

    pWpTool_ = 0;
            
    boxSliderNormalRadiusConnector_.Init(ui_.radiusDoubleSpinBox, ui_.radiusSlider);
    boxSliderNormalRadiusConnector_.InitBox(kNormalRadiusStep,kNormalRadiusMin,kNormalRadiusMax);
    //connect(&boxSliderConnector_, SIGNAL(ValueChanged(double)), this, SIGNAL(ValueChanged(double)));
    connect(&boxSliderNormalRadiusConnector_, SIGNAL(ValueChanged(double)), this, SLOT(SendNormalRadiusValue(double)));
        
    QButtonGroup* buttonGroupNormalRadiusMode = new QButtonGroup(this);
    buttonGroupNormalRadiusMode->addButton(ui_.normalRadioButton);
    buttonGroupNormalRadiusMode->addButton(ui_.stairsRadioButton);    
    
    connect(ui_.normalRadioButton, SIGNAL(pressed()), this, SLOT(SendNormalRadiusNormalMode()));
    connect(ui_.stairsRadioButton, SIGNAL(pressed()), this, SLOT(SendNormalRadiusStairsMode()));
    
    
    boxSliderMinRssValueConnector_.Init(ui_.minRssValueDoubleSpinBox, ui_.minRssValueSlider);
    boxSliderMinRssValueConnector_.InitBox(kRssMinValueStep,kRssMinValueMin,kRssMinValueMax);
    //connect(&boxSliderConnector_, SIGNAL(ValueChanged(double)), this, SIGNAL(ValueChanged(double)));
    connect(&boxSliderMinRssValueConnector_, SIGNAL(ValueChanged(double)), this, SLOT(SetMinRssValueEnable(double)));
    boxSliderMinRssValueConnector_.SetValue(kRssMinValueDefault);
    
    
    
    QButtonGroup* buttonGroupLpEnable = new QButtonGroup(this);
    buttonGroupLpEnable->addButton(ui_.lpEnableRadioButton);
    buttonGroupLpEnable->addButton(ui_.lpDisableRadioButton); 
    
    connect(ui_.lpEnableRadioButton,   SIGNAL(pressed()), this, SLOT(SetClosestObstVelReductionEnable()));
    connect(ui_.lpDisableRadioButton, SIGNAL(pressed()), this, SLOT(SetClosestObstVelReductionDisable()));
    
    
    QButtonGroup* buttonGroupRssEnable = new QButtonGroup(this);
    buttonGroupRssEnable->addButton(ui_.rssEnableRadioButton);
    buttonGroupRssEnable->addButton(ui_.rssDisableRadioButton); 
    
    connect(ui_.rssEnableRadioButton,   SIGNAL(pressed()), this, SLOT(SetRssEnable()));
    connect(ui_.rssDisableRadioButton, SIGNAL(pressed()), this, SLOT(SetRssDisable()));

    connect(ui_.rssLoadPushButton, SIGNAL(pressed()), this, SLOT(RssLoad()));
    connect(ui_.rssSavePushButton, SIGNAL(pressed()), this, SLOT(RssSave()));      
    
    
    //boxSliderConnector_.SetValue(kNormalRadiusDefault); // after the connection 
    ui_.normalRadioButton->click();
    ui_.lpEnableRadioButton->click();
    ui_.rssDisableRadioButton->click();
    
    
    ROS_INFO_STREAM("NavigationDialogUI::NavigationDialogUI() - setting normal radius default " << kNormalRadiusModeNormal);    
}

NavigationDialogUI::~NavigationDialogUI()
{
    std::cout << "NavigationDialogUI::~NavigationDialogUI()" << std::endl;

}

void NavigationDialogUI::SetNormalRadiusValue(double val)
{
    boxSliderNormalRadiusConnector_.SetValue(val);
}


void NavigationDialogUI::SetWpTool(WaypointsTool* pWpTool)
{
    ROS_INFO_STREAM("NavigationDialogUI::SetWpTool()");
    pWpTool_ = pWpTool;
    SetConnectionsWithWpTool();
}

void NavigationDialogUI::SetConnectionsWithWpTool()
{
    if (pWpTool_)
    {
        ROS_INFO_STREAM("NavigationDialogUI::SetConnections()");
        //connect(ui_.lpEnableRadioButton, SIGNAL(clicked(bool)), pWpTool_, SLOT(SetClosestObstVelReductionEnable(bool)));
    }
}
 
static void SendNormalRadiusValueParallel(double val)
{
    // from http://ros-robotics.blogspot.it/2016/03/controling-dynamic-reconfiguration-from.html
    // and
    // from https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/
    
    static std::string suffix("/set_parameters"); // this is the standard suffix for the dynamic reconfigure service 
    static std::string ss(std::string(NAME_COMPUTE_NORMAL_ESTIMATION_DYNRECONF) + suffix);

    // NOTE: keep "/" as a prefix since these behave like absolute topic names 
    static std::string ss_ugv1(std::string("/ugv1") + std::string(NAME_COMPUTE_NORMAL_ESTIMATION_DYNRECONF) + suffix);
    static std::string ss_ugv2(std::string("/ugv2") + std::string(NAME_COMPUTE_NORMAL_ESTIMATION_DYNRECONF) + suffix);
    static std::string ss_ugv3(std::string("/ugv3") + std::string(NAME_COMPUTE_NORMAL_ESTIMATION_DYNRECONF) + suffix);

    static std::string ss_sim_ugv1(std::string(NAME_COMPUTE_NORMAL_ESTIMATION_UGV1_DYNRECONF) + suffix);
    static std::string ss_sim_ugv2(std::string(NAME_COMPUTE_NORMAL_ESTIMATION_UGV2_DYNRECONF) + suffix);
    static std::string ss_sim_ugv3(std::string(NAME_COMPUTE_NORMAL_ESTIMATION_UGV3_DYNRECONF) + suffix);
        
    static std::vector<std::string> ss_array = {ss, ss_ugv1, ss_ugv2, ss_ugv3, ss_sim_ugv1,ss_sim_ugv2,ss_sim_ugv3};
    
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "radius";
    double_param.value = val;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

//    if(ros::service::exists("/compute_normals_ugv1/NormalEstimationPcl/set_parameters",true))
//    {
//        if(ros::service::call("/compute_normals_ugv1/NormalEstimationPcl/set_parameters", srv_req, srv_resp))
//        {
//            ROS_INFO_STREAM("NavigationDialogUI::SendNormalRadiusValue() - sent radius " << val );
//        }
//    }

    for(size_t ii=0; ii<ss_array.size(); ii++)
    {
        if(ros::service::exists(ss_array[ii],true))
        {
            if(ros::service::call(ss_array[ii], srv_req, srv_resp))
            {
                ROS_INFO_STREAM("SendNormalRadiusValueParallel() - sent radius " << val << " on " << ss_array[ii] );
            }
        }  
    }
}


void NavigationDialogUI::SendNormalRadiusValue(double val)
{
    // rosrun dynamic_reconfigure dynparam set /compute_normals_ugv1/NormalEstimationPcl radius 0.4
    
//    std::stringstream ss,ss_ugv1,ss_ugv2,ss_ugv3;
//    ss      << "rosrun dynamic_reconfigure dynparam set " << NAME_COMPUTE_NORMAL_ESTIMATION_DYNRECONF      << " radius "  << val << " -t" << kDynamicReconfigureTimeout;
//    ss_ugv1 << "rosrun dynamic_reconfigure dynparam set " << NAME_COMPUTE_NORMAL_ESTIMATION_UGV1_DYNRECONF << " radius "  << val << " -t" << kDynamicReconfigureTimeout;
//    ss_ugv2 << "rosrun dynamic_reconfigure dynparam set " << NAME_COMPUTE_NORMAL_ESTIMATION_UGV2_DYNRECONF << " radius "  << val << " -t" << kDynamicReconfigureTimeout;
//    ss_ugv3 << "rosrun dynamic_reconfigure dynparam set " << NAME_COMPUTE_NORMAL_ESTIMATION_UGV3_DYNRECONF << " radius "  << val << " -t" << kDynamicReconfigureTimeout;
//    
//    
//    ROS_INFO_STREAM("NavigationDialogUI::SendNormalRadiusValue() - sending radius " << val);    
//    
//    // not working
//    //auto handle = std::async(std::launch::async, system, ss_ugv1.str().c_str()); 
//    
//    QFuture<int> future  = QtConcurrent::run(system,ss.str().c_str());
//    QFuture<int> future1 = QtConcurrent::run(system,ss_ugv1.str().c_str());
//    QFuture<int> future2 = QtConcurrent::run(system,ss_ugv2.str().c_str());
//    QFuture<int> future3 = QtConcurrent::run(system,ss_ugv3.str().c_str());  
//    int result1 = future1.result();
    
    
    QFuture<void> future  = QtConcurrent::run(SendNormalRadiusValueParallel,val);
    
    //ROS_INFO_STREAM("NavigationDialogUI::SendNormalRadiusValue() - command: " << ss_ugv1.str().c_str() );    
    //ROS_INFO_STREAM("NavigationDialogUI::SendNormalRadiusValue() - " << result1);    
    //ROS_INFO_STREAM("NavigationDialogUI::SendNormalRadiusValue() - sent radius " << val );
}


void NavigationDialogUI::SendNormalRadiusNormalMode()
{
    SetNormalRadiusValue(kNormalRadiusModeNormal);  
}


void NavigationDialogUI::SendNormalRadiusStairsMode()
{
    SetNormalRadiusValue(kNormalRadiusModeStairs);      
}


void NavigationDialogUI::SetClosestObstVelReductionEnable()
{
    if(pWpTool_) pWpTool_->SetClosestObstVelReductionEnable(true);
}


void NavigationDialogUI::SetClosestObstVelReductionDisable()
{
    if(pWpTool_) pWpTool_->SetClosestObstVelReductionEnable(false);    
}

void NavigationDialogUI::SetRssEnable()
{
    if(pWpTool_) pWpTool_->SetRssEnable(true);
}

void NavigationDialogUI::SetRssDisable()
{
    if(pWpTool_) pWpTool_->SetRssEnable(false);    
}
 
void NavigationDialogUI::RssLoad()
{
    if(pWpTool_) pWpTool_->RssLoad();
}

void NavigationDialogUI::RssSave()
{
    if(pWpTool_) pWpTool_->RssSave();    
}

void NavigationDialogUI::SetMinRssValueEnable(double val)
{
    if(pWpTool_) pWpTool_->SetMinRssValueEnable((int)val);        
}
    
}
