// This file is part of the ROS PLUGIN for V-REP
// 
// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// A big thanks to Svetlin Penkov for his precious help!
// 
// The ROS PLUGIN is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The ROS PLUGIN is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// THE ROS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the ROS PLUGIN.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

#include "sensor_msgs/distortion_models.h"
#include <boost/algorithm/string/replace.hpp>
#include <map>
#include "../include/vrep_plugin/ROS_server.h"
#include "../include/v_repLib.h"

std::map<std::string,std::string> renamedFrames;

// Static variables:
ros::NodeHandle* ROS_server::node = NULL;
tf::TransformBroadcaster* ROS_server::tf_broadcaster = NULL;
tf::TransformBroadcasterCustom* ROS_server::tf_old_broadcaster = NULL;

image_transport::ImageTransport* ROS_server::images_streamer=NULL;
int ROS_server::imgStreamerCnt=0;

std::vector<std::string> ROS_server::_last50Errors;

bool ROS_server::_waitTriggerEnable=false;
bool ROS_server::_waitForTrigger=true;

std::vector<SPublisherData> ROS_server::publishers;
ros::Publisher ROS_server::infoPublisher; // special publisher that is active also when simulation is not running!

std::vector<CSubscriberData*> ROS_server::subscribers;
int ROS_server::lastSubscriberID=-1;

int ROS_server::_simulationFrameID=-1;

static std::string objNameToFrameId(const std::string & name) 
{
   // First check if there is a remapping (courtesy of Federico Ferri)
   std::map<std::string, std::string>::iterator it = renamedFrames.find(name);
   if(it != renamedFrames.end())
      return(renamedFrames[name]);
	  
	std::string slash("/");
	std::string out;
#if 0	
	// melodic and before
	if (name[0]=='/')
		out=name;
	else
		out=slash+name;
#else
    // noetic
	if (name[0]=='/')
		out=name.substr(1,name.length()-1);
	else
		out=name;
#endif

	// Following changes courtesy of Cedric Pradalier:
	boost::replace_all(out,"#","_HASHMARK_");
	boost::replace_all(out,"-","_MINUSMARK_");
	boost::replace_all(out,"(","_OPARENTHESISMARK_");
	boost::replace_all(out,")","_CPARENTHESISMARK_");

	return(out);
}

// Remapping (courtesy of Federico Ferri):
static std::string objIdToFrameId(int objId)
{
	simChar* pname=simGetObjectName(objId);
	if (pname!=NULL)
	{
		std::string objName(pname);
		std::string ret = objNameToFrameId(objName);
		simReleaseBuffer(pname);
		return ret;
	}
	else
		return "";
}

struct SPointCloudPublisherData : public SSpecificPublisherData
{
	sensor_msgs::PointCloud2 pointcloud;
	SPointCloudPublisherData(const std::string & obj_name)
	{
		pointcloud.header.frame_id = objNameToFrameId(obj_name);
		pointcloud.fields.resize(3);
		pointcloud.fields[0].name="x";
		pointcloud.fields[0].offset=0; 
		pointcloud.fields[0].datatype=sensor_msgs::PointField::FLOAT32;
		pointcloud.fields[0].count=1;
		pointcloud.fields[1].name="y";
		pointcloud.fields[1].offset=4; 
		pointcloud.fields[1].datatype=sensor_msgs::PointField::FLOAT32;
		pointcloud.fields[1].count=1;
		pointcloud.fields[2].name="z";
		pointcloud.fields[2].offset=8; 
		pointcloud.fields[2].datatype=sensor_msgs::PointField::FLOAT32;
		pointcloud.fields[2].count=1;
		pointcloud.point_step = 12;
		pointcloud.height = 1;
		pointcloud.width = 0; // To be updated
		pointcloud.row_step = 0; // To be updated
		pointcloud.is_bigendian = false;
		pointcloud.is_dense = true;
	}
};

struct SDepthCloudPublisherData : public SPointCloudPublisherData
{
    std::vector<float> x_scale;
    std::vector<float> y_scale;
	SDepthCloudPublisherData(const std::string & obj_name) : SPointCloudPublisherData(obj_name)
	{
        pointcloud.fields.resize(4);
        pointcloud.fields[3].name="rgb";
        pointcloud.fields[3].offset=12; 
        pointcloud.fields[3].datatype=sensor_msgs::PointField::UINT32;
        pointcloud.fields[3].count=1;
        pointcloud.point_step = 16;
	}
};

SSpecificPublisherData::~SSpecificPublisherData() {}

ros::ServiceServer ROS_server::simRosAddStatusbarMessageServer;
ros::ServiceServer ROS_server::simRosAuxiliaryConsoleCloseServer;
ros::ServiceServer ROS_server::simRosAuxiliaryConsoleOpenServer;
ros::ServiceServer ROS_server::simRosAuxiliaryConsolePrintServer;
ros::ServiceServer ROS_server::simRosAuxiliaryConsoleShowServer;
ros::ServiceServer ROS_server::simRosBreakForceSensorServer;
ros::ServiceServer ROS_server::simRosClearFloatSignalServer;
ros::ServiceServer ROS_server::simRosClearIntegerSignalServer;
ros::ServiceServer ROS_server::simRosClearStringSignalServer;
ros::ServiceServer ROS_server::simRosCloseSceneServer;
ros::ServiceServer ROS_server::simRosCopyPasteObjectsServer;
ros::ServiceServer ROS_server::simRosDisplayDialogServer;
ros::ServiceServer ROS_server::simRosEndDialogServer;
ros::ServiceServer ROS_server::simRosEraseFileServer;
ros::ServiceServer ROS_server::simRosGetArrayParameterServer;
ros::ServiceServer ROS_server::simRosGetBooleanParameterServer;
ros::ServiceServer ROS_server::simRosGetCollisionHandleServer;
ros::ServiceServer ROS_server::simRosGetDialogInputServer;
ros::ServiceServer ROS_server::simRosGetDialogResultServer;
ros::ServiceServer ROS_server::simRosGetDistanceHandleServer;
ros::ServiceServer ROS_server::simRosGetFloatingParameterServer;
ros::ServiceServer ROS_server::simRosGetFloatSignalServer;
ros::ServiceServer ROS_server::simRosGetIntegerParameterServer;
ros::ServiceServer ROS_server::simRosGetIntegerSignalServer;
ros::ServiceServer ROS_server::simRosGetJointMatrixServer;
ros::ServiceServer ROS_server::simRosGetJointStateServer;
ros::ServiceServer ROS_server::simRosGetLastErrorsServer;
ros::ServiceServer ROS_server::simRosGetModelPropertyServer;
ros::ServiceServer ROS_server::simRosGetObjectChildServer;
ros::ServiceServer ROS_server::simRosGetObjectFloatParameterServer;
ros::ServiceServer ROS_server::simRosGetObjectHandleServer;
ros::ServiceServer ROS_server::simRosGetObjectIntParameterServer;
//ros::ServiceServer ROS_server::simRosGetObjectOrientationServer;
ros::ServiceServer ROS_server::simRosGetObjectParentServer;
//ros::ServiceServer ROS_server::simRosGetObjectPositionServer;
ros::ServiceServer ROS_server::simRosGetObjectPoseServer;
ros::ServiceServer ROS_server::simRosGetObjectsServer;
ros::ServiceServer ROS_server::simRosGetObjectSelectionServer;
ros::ServiceServer ROS_server::simRosGetStringParameterServer;
ros::ServiceServer ROS_server::simRosGetStringSignalServer;
ros::ServiceServer ROS_server::simRosGetUIButtonPropertyServer;
ros::ServiceServer ROS_server::simRosGetUIEventButtonServer;
ros::ServiceServer ROS_server::simRosGetUIHandleServer;
ros::ServiceServer ROS_server::simRosGetUISliderServer;
ros::ServiceServer ROS_server::simRosGetVisionSensorDepthBufferServer;
ros::ServiceServer ROS_server::simRosGetVisionSensorImageServer;
ros::ServiceServer ROS_server::simRosLoadModelServer;
ros::ServiceServer ROS_server::simRosLoadSceneServer;
ros::ServiceServer ROS_server::simRosLoadUIServer;
ros::ServiceServer ROS_server::simRosPauseSimulationServer;
ros::ServiceServer ROS_server::simRosReadCollisionServer;
ros::ServiceServer ROS_server::simRosReadDistanceServer;
ros::ServiceServer ROS_server::simRosReadForceSensorServer;
ros::ServiceServer ROS_server::simRosReadProximitySensorServer;
ros::ServiceServer ROS_server::simRosReadVisionSensorServer;
ros::ServiceServer ROS_server::simRosRemoveObjectServer;
ros::ServiceServer ROS_server::simRosRemoveModelServer;
ros::ServiceServer ROS_server::simRosRemoveUIServer;
ros::ServiceServer ROS_server::simRosSetArrayParameterServer;
ros::ServiceServer ROS_server::simRosSetBooleanParameterServer;
ros::ServiceServer ROS_server::simRosSetFloatingParameterServer;
ros::ServiceServer ROS_server::simRosSetFloatSignalServer;
ros::ServiceServer ROS_server::simRosSetIntegerParameterServer;
ros::ServiceServer ROS_server::simRosSetIntegerSignalServer;
ros::ServiceServer ROS_server::simRosSetJointForceServer;
ros::ServiceServer ROS_server::simRosSetJointPositionServer;
ros::ServiceServer ROS_server::simRosSetJointTargetPositionServer;
ros::ServiceServer ROS_server::simRosSetJointTargetVelocityServer;
ros::ServiceServer ROS_server::simRosSetModelPropertyServer;
ros::ServiceServer ROS_server::simRosSetObjectFloatParameterServer;
ros::ServiceServer ROS_server::simRosSetObjectIntParameterServer;
ros::ServiceServer ROS_server::simRosSetObjectPoseServer;
ros::ServiceServer ROS_server::simRosSetObjectParentServer;
ros::ServiceServer ROS_server::simRosSetObjectPositionServer;
ros::ServiceServer ROS_server::simRosSetObjectSelectionServer;
ros::ServiceServer ROS_server::simRosGetInfoServer;
ros::ServiceServer ROS_server::simRosSetSphericalJointMatrixServer;
ros::ServiceServer ROS_server::simRosSetStringSignalServer;
ros::ServiceServer ROS_server::simRosSetUIButtonLabelServer;
ros::ServiceServer ROS_server::simRosSetUIButtonPropertyServer;
ros::ServiceServer ROS_server::simRosSetUISliderServer;
ros::ServiceServer ROS_server::simRosSetVisionSensorImageServer;
ros::ServiceServer ROS_server::simRosStartSimulationServer;
ros::ServiceServer ROS_server::simRosStopSimulationServer;
ros::ServiceServer ROS_server::simRosSynchronousServer;
ros::ServiceServer ROS_server::simRosSynchronousTriggerServer;
ros::ServiceServer ROS_server::simRosTransferFileServer;
ros::ServiceServer ROS_server::simRosEnablePublisherServer;
ros::ServiceServer ROS_server::simRosDisablePublisherServer;
//ros::ServiceServer ROS_server::simRosGetObjectQuaternionServer;
ros::ServiceServer ROS_server::simRosSetObjectQuaternionServer;
ros::ServiceServer ROS_server::simRosEnableSubscriberServer;
ros::ServiceServer ROS_server::simRosDisableSubscriberServer;
ros::ServiceServer ROS_server::simRosSetJointStateServer;
ros::ServiceServer ROS_server::simRosCreateDummyServer;
ros::ServiceServer ROS_server::simRosGetAndClearStringSignalServer;
ros::ServiceServer ROS_server::simRosGetObjectGroupDataServer;

bool ROS_server::initialize()
{
	int argc = 0;
	char** argv = NULL;
	ros::init(argc,argv,"vrep");

	if(!ros::master::check())
		return(false);
	
	node=new ros::NodeHandle("~");
        
        tf_broadcaster     = new tf::TransformBroadcaster();
        tf_old_broadcaster = new tf::TransformBroadcasterCustom("tf_old");

	enableAPIServices();
	infoPublisher=node->advertise<vrep_common::VrepInfo>("info",1); // special case! This is the only publisher always active!

	return(true);
}

void ROS_server::shutDown()
{
	infoPublisher.shutdown(); // special case! This is the only publisher always active!
	disableAPIServices();
	ros::shutdown();
}

void ROS_server::instancePass()
{ // When simulation is not running, we "spinOnce" here:
	int simState=simGetSimulationState();
	if ((simState&sim_simulation_advancing)==0)
		spinOnce();
}

void ROS_server::simulationAboutToStart()
{
	_simulationFrameID=0;
}

bool ROS_server::mainScriptAboutToBeCalled()
{ // When simulation is running, we "spinOnce" here:

	spinOnce();

	// If we return false, then a new simulation pass will be executed. Otherwise V-REP waits. This allows
	// to have a ROS client and V-REP run synchronously (i.e. the client will trigger each next smulation pass)
	_simulationFrameID++;
	if (_waitTriggerEnable)
	{
		if (_waitForTrigger)
		{ // We stay in this simulation frame
			_simulationFrameID--;
			return(true);
		}
		_waitForTrigger=true;
		return(false); // trigger was activated!
	}
	return(false);
}

void ROS_server::simulationEnded()
{
	removeAllPublishers();
	removeAllSubscribers();
	_simulationFrameID=-1;
	renamedFrames.clear();
}

void ROS_server::spinOnce()
{
	// Disable error reporting (it is enabled in the service processing part, but we don't want error reporting for publishers/subscribers)
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	char* err=simGetLastError(); // just clear the last error
	if (err!=NULL)
		simReleaseBuffer(err);

	//Handle all streaming (publishers). Those are normally not modifying the scene (sensing phase).
	streamAllData();

	//Process all requested services and topic subscriptions. Those can be modifying the scene (actuation phase).
	ros::spinOnce();

	// Restore previous error report mode:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); 
}


std::string ROS_server::addPublisher(const char* topicName,int queueSize,int streamCmd,int auxInt1,int auxInt2,const char* auxString,int publishCnt)
{
	// 0. Check if the streamCmd is valid:
	if (streamCmd==simros_strmcmdnull_start)
		return("");
	if ( (streamCmd>=simros_strmcmdnull_subscriber_start)&&(streamCmd<=simros_strmcmdint_start) )
		return("");
	if ( (streamCmd>=simros_strmcmdint_subscriber_start)&&(streamCmd<=simros_strmcmdintint_start) )
		return("");
	if ( (streamCmd>=simros_strmcmdintint_subscriber_start)&&(streamCmd<=simros_strmcmdstring_start) )
		return("");
	if ( (streamCmd>=simros_strmcmdstring_subscriber_start)&&(streamCmd<=simros_strmcmdintstring_start) )
		return("");
	if (streamCmd>=simros_strmcmdintstring_subscriber_start)
		return("");

	// 1. Check if we have already such a publisher:
	int ind=getPublisherIndexFromCmd(streamCmd,auxInt1,auxInt2,auxString);
	if (ind>=0)
	{ // Yes, there is already such a publisher. We simply return the topic name of that publisher:
		publishers[ind].dependencyCnt++;
		return(publishers[ind].topicName);
	}
	// 2. Check if we have already such a topic name. If yes, generate a new name:
	std::string nm(topicName);

	// Following changes courtesy of Cedric Pradalier:
	boost::replace_all(nm,"#","_HASHMARK_");
	boost::replace_all(nm,"-","_MINUSMARK_");
	boost::replace_all(nm,"(","_OPARENTHESISMARK_");
	boost::replace_all(nm,")","_CPARENTHESISMARK_");
	
	ind=getPublisherIndexFromTopicName(nm.c_str());
	if ( (ind==-1)&&(nm.compare("info")==0) )
	{ // special case: the info stream is handled differently (always active when V-REP is running, not only during simulations)
		ind=0; // will trigger a generation of a new name
	}
	while (ind!=-1)
	{
		nm+="_2"; // we simply append "_2" to the name!
		ind=getPublisherIndexFromTopicName(nm.c_str());
	}
	// 3. Try to add a new publisher with the given topic name:
	SPublisherData pub;
	pub.cmdID=streamCmd;
	pub.auxInt1=auxInt1;
	pub.auxInt2=auxInt2;
	pub.auxStr=auxString;
	pub.publishCnt=publishCnt;
	pub.topicName=nm;
	pub.dependencyCnt=0;
	pub.specificPublisherData=NULL;
	if (launchPublisher(pub,queueSize))
	{ // We launched the publisher!
		publishers.push_back(pub);
		return(pub.topicName);
	}
        else
        {
            std::cout << "failed to launch the publisher: " << pub.topicName << std::endl; 
        }
	return(""); // We failed launching the publisher!
}


int ROS_server::getPublisherIndexFromCmd(int streamCmd,int auxInt1,int auxInt2,const char* auxString)
{
	for (unsigned int i=0;i<publishers.size();i++)
	{
		if (publishers[i].cmdID==streamCmd)
		{ // we have a similar command here. Check if the aux. data is same too:
			if (streamCmd<simros_strmcmdint_start)
				return(i); // Yes! (these command types don't have any aux data)
			if ((streamCmd<simros_strmcmdintint_start)&&(streamCmd>simros_strmcmdint_start))
			{ // these command types have an integer as aux data!
				if (publishers[i].auxInt1==auxInt1)
					return(i); // Yes!
			}
			if ((streamCmd<simros_strmcmdstring_start)&&(streamCmd>simros_strmcmdintint_start))
			{ // these command types have 2 integers as aux data!
				if ( (publishers[i].auxInt1==auxInt1)&&(publishers[i].auxInt2==auxInt2) )
					return(i); // Yes!
			}
			if ((streamCmd<simros_strmcmdintstring_start)&&(streamCmd>simros_strmcmdstring_start))
			{ // these command types have a string as aux data!
				if (publishers[i].auxStr.compare(auxString)==0)
					return(i); // Yes!
			}
			if ((streamCmd<simros_strmcmdreserved_start)&&(streamCmd>simros_strmcmdintstring_start))
			{ // these command types have an integer and a string as aux data!
				if ( (publishers[i].auxInt1==auxInt1)&&(publishers[i].auxStr.compare(auxString)==0) )
					return(i); // Yes!
			}
                        /*if (streamCmd>=simros_strmcmdreserved_start)
                        { // these command types have an integer and a string as aux data!
				if ( (publishers[i].auxInt1==auxInt1)&&(publishers[i].auxStr.compare(auxString)==0) )
					return(i); // Yes!
                        }*/
		}
	}
	return(-1);
}

int ROS_server::getPublisherIndexFromTopicName(const char* topicName)
{
	for (unsigned int i=0;i<publishers.size();i++)
	{
		if (publishers[i].topicName.compare(topicName)==0)
			return(i);
	}
	return(-1);
}

int ROS_server::removePublisher(const char* topicName,bool ignoreReferenceCounter)
{
	for (unsigned int i=0;i<publishers.size();i++)
	{
		if (publishers[i].topicName.compare(topicName)==0)
		{
			publishers[i].dependencyCnt--;
			if ((publishers[i].dependencyCnt==0)||ignoreReferenceCounter)
			{
				shutDownPublisher(publishers[i]);
				publishers.erase(publishers.begin()+i);
				return(0);
			}
			else
				return(publishers[i].dependencyCnt);
		}
	}
	return(-1);
}

int ROS_server::wakePublisher(const char* topicName,int publishCnt)
{
	for (unsigned int i=0;i<publishers.size();i++)
	{
		if (publishers[i].topicName.compare(topicName)==0)
		{
			if (publishCnt<-1)
				return(publishers[i].publishCnt);
			publishers[i].publishCnt=publishCnt;
			return(1);
		}
	}
	return(-2);
}

void ROS_server::removeAllPublishers()
{
	while (publishers.size()!=0)
		removePublisher(publishers[0].topicName.c_str(),true);
}

bool ROS_server::launchPublisher(SPublisherData& pub,int queueSize)
{
	if (pub.cmdID==simros_strmcmd_get_odom_data)
	{ // courtesy of George Moustris
	      pub.generalPublisher=node->advertise<nav_msgs::Odometry>(pub.topicName,queueSize);
	      pub.dependencyCnt++;
	      return(true);
	}
	if (pub.cmdID==simros_strmcmd_get_laser_scanner_data)
	{ // courtesy of George Moustris
	      pub.generalPublisher=node->advertise<sensor_msgs::LaserScan>(pub.topicName,queueSize);
	      pub.dependencyCnt++;
	      return(true);
	}
	if (pub.cmdID==simros_strmcmd_get_vision_sensor_image)
	{
		if (simGetObjectType(pub.auxInt1)!=sim_object_visionsensor_type)
			return(false); // invalid data!
		if (imgStreamerCnt==0)
			images_streamer = new image_transport::ImageTransport(*ROS_server::node);
		pub.imagePublisher=images_streamer->advertise(pub.topicName,queueSize);
		imgStreamerCnt++;
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_joint_state)
	{
		if ( (pub.auxInt1!=sim_handle_all)&&(simGetObjectType(pub.auxInt1)!=sim_object_joint_type) )
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<sensor_msgs::JointState>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	
	if (pub.cmdID==simros_strmcmd_get_array_parameter)
	{
		float dummy[3];
		if (simGetArrayParameter(pub.auxInt1,dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<geometry_msgs::Point32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	if (pub.cmdID==simros_strmcmd_get_boolean_parameter)
	{
		if (simGetBooleanParameter(pub.auxInt1)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::UInt8>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	if (pub.cmdID==simros_strmcmd_get_dialog_result)
	{
		if (simGetDialogResult(pub.auxInt1)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	
	if (pub.cmdID==simros_strmcmd_get_floating_parameter)
	{
		float dummy;
		if (simGetFloatingParameter(pub.auxInt1,&dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Float32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	
	if (pub.cmdID==simros_strmcmd_get_integer_parameter)
	{
		int dummy;
		if (simGetIntegerParameter(pub.auxInt1,&dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_float_signal)
	{
//		float dummy;
//		if (simGetFloatSignal(pub.auxStr.c_str(),&dummy)==-1)
//			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Float32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_integer_signal)
	{
//		int dummy;
//		if (simGetIntegerSignal(pub.auxStr.c_str(),&dummy)==-1)
//			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_string_signal)
	{
		pub.generalPublisher=node->advertise<std_msgs::String>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	
	if (pub.cmdID==simros_strmcmd_get_and_clear_string_signal)
	{
		pub.generalPublisher=node->advertise<std_msgs::String>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_string_parameter)
	{
		char* str=simGetStringParameter(pub.auxInt1);
		if (str==NULL)
			return(false); // invalid data!
		simReleaseBuffer(str);
		pub.generalPublisher=node->advertise<std_msgs::String>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_object_float_parameter)
	{
		float dummy;
		if (simGetObjectFloatParameter(pub.auxInt1,pub.auxInt2,&dummy)<=0)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Float32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	
	if (pub.cmdID==simros_strmcmd_get_object_group_data)
	{
		pub.generalPublisher=node->advertise<vrep_common::ObjectGroupData>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true); // we remove it later if invalid!
	}


	if (pub.cmdID==simros_strmcmd_get_object_int_parameter)
	{
		int dummy;
		if (simGetObjectIntParameter(pub.auxInt1,pub.auxInt2,&dummy)<=0)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
/*
	if (pub.cmdID==simros_strmcmd_get_object_orientation)
	{
		float dummy[3];
		if (simGetObjectOrientation(pub.auxInt1,pub.auxInt2,dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<geometry_msgs::Point32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_object_quaternion)
	{
		float dummy[4];
		if (simGetObjectQuaternion(pub.auxInt1,pub.auxInt2,dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<geometry_msgs::Quaternion>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	
	if (pub.cmdID==simros_strmcmd_get_object_position)
	{
		float dummy[3];
		if (simGetObjectPosition(pub.auxInt1,pub.auxInt2,dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<geometry_msgs::Point32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	*/
	if (pub.cmdID==simros_strmcmd_get_object_pose)
	{
		float dummy[3];
		if (simGetObjectPosition(pub.auxInt1,pub.auxInt2,dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<geometry_msgs::PoseStamped>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	if (pub.cmdID==simros_strmcmd_get_object_parent)
	{
		if (simGetObjectType(pub.auxInt1)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_objects)
	{
		pub.generalPublisher=node->advertise<std_msgs::Int32MultiArray>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_object_selection)
	{
		pub.generalPublisher=node->advertise<std_msgs::Int32MultiArray>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_ui_button_property)
	{
		if (simGetUIButtonProperty(pub.auxInt1,pub.auxInt2)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_ui_event_button)
	{
		if (simGetUIProperty(pub.auxInt1)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32MultiArray>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_ui_slider)
	{
		if (simGetUIButtonProperty(pub.auxInt1,pub.auxInt2)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_vision_sensor_depth_buffer)
	{
		if (simGetObjectType(pub.auxInt1)!=sim_object_visionsensor_type)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<vrep_common::VisionSensorDepthBuff>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_depth_sensor_data) {
		if (simGetObjectType(pub.auxInt1)!=sim_object_visionsensor_type)
			return(false); // invalid data!
		// We might set-up the signal AFTER setting up the publisher, so always return true
		pub.generalPublisher=node->advertise<sensor_msgs::PointCloud2>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		char* objName=simGetObjectName(pub.auxInt1);
		pub.specificPublisherData=new SDepthCloudPublisherData(objName);
		simReleaseBuffer(objName);
		return(true);
    }







	if (pub.cmdID==simros_strmcmd_get_range_finder_data)
	{
		// We might set-up the signal AFTER setting up the publisher, so always return true
		pub.generalPublisher=node->advertise<sensor_msgs::PointCloud2>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		char* objName=simGetObjectName(pub.auxInt1);
		pub.specificPublisherData=new SPointCloudPublisherData(objName);
		simReleaseBuffer(objName);
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_vision_sensor_info)
	{
		if (simGetObjectType(pub.auxInt1)!=sim_object_visionsensor_type)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<sensor_msgs::CameraInfo>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_read_collision)
	{
		if (simReadCollision(pub.auxInt1)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Int32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_read_distance)
	{
		float dummy;
		if (simReadDistance(pub.auxInt1,&dummy)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<std_msgs::Float32>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_read_force_sensor)
	{
		if (simReadForceSensor(pub.auxInt1,NULL,NULL)==-1)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<vrep_common::ForceSensorData>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_read_proximity_sensor)
	{
		if (simGetObjectType(pub.auxInt1)!=sim_object_proximitysensor_type)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<vrep_common::ProximitySensorData>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_read_vision_sensor)
	{
		if (simGetObjectType(pub.auxInt1)!=sim_object_visionsensor_type)
			return(false); // invalid data!
		pub.generalPublisher=node->advertise<vrep_common::VisionSensorData>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	
	if (pub.cmdID==simros_strmcmd_get_twist_status)
	{
		pub.generalPublisher=node->advertise<geometry_msgs::TwistStamped>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}

	if (pub.cmdID==simros_strmcmd_get_transform)
	{
            std::cout << "transform cmID: " << simros_strmcmd_get_transform << std::endl; 
		float dummy[3];
		if (simGetObjectPosition(pub.auxInt1,pub.auxInt2,dummy)==-1)
			return(false); // invalid data!
		pub.dependencyCnt++;
		return(true);
	}
        
        if (pub.cmdID==simros_strmcmd_get_transform_remapped)
	{
            std::cout << "transform remapped cmID: " << simros_strmcmd_get_transform_remapped << std::endl; 
		float dummy[3];
		if (simGetObjectPosition(pub.auxInt1,pub.auxInt2,dummy)==-1)
			return(false); // invalid data!
		pub.dependencyCnt++;
		return(true);
	}
	
	if(pub.cmdID==simros_strmcmd_get_tracks_vel)
	{
		pub.generalPublisher=node->advertise<nifti_robot_driver_msgs::TracksStamped>(pub.topicName,queueSize);
		pub.dependencyCnt++;
		return(true);
	}
	return(false); // we failed at launching the publisher
}

void ROS_server::shutDownPublisher(SPublisherData& pub)
{
	if (pub.specificPublisherData!=NULL)
	{
		delete pub.specificPublisherData;
		pub.specificPublisherData=NULL;
	}
	if (pub.cmdID==simros_strmcmd_get_vision_sensor_image)
	{
		pub.imagePublisher.shutdown();
		imgStreamerCnt--;
		if (imgStreamerCnt==0)
		{
			delete images_streamer;
			images_streamer=NULL;
		}
	}
	else
		pub.generalPublisher.shutdown();
}


//==============================================================================================================
//========================================= STREAMING ==========================================================
//==============================================================================================================


void ROS_server::streamAllData()
{
	// 1. First we stream the data that is always streamed (also when simulation is not running): the info
	vrep_common::VrepInfo inf;
	int simState=simGetSimulationState();
	inf.simulatorState.data=0;
	if (simState!=sim_simulation_stopped)
	{
		inf.simulatorState.data|=1; // simulation is not stopped
		if (simState==sim_simulation_paused)
			inf.simulatorState.data|=2; // simulation is paused
	}
	if (simGetRealTimeSimulation()>0)
		inf.simulatorState.data|=4;
	int editModeType;
	simGetIntegerParameter(sim_intparam_edit_mode_type,&editModeType);
	inf.simulatorState.data|=(editModeType<<3);
	inf.simulationTime.data=simGetSimulationTime();
	inf.timeStep.data=simGetSimulationTimeStep();
	inf.headerInfo.seq=_simulationFrameID;
	inf.headerInfo.stamp=ros::Time::now();
	infoPublisher.publish(inf);

	// 2. Now handle all "regular" streams (those that can only run while simulation is running):
	for (int pubI=0;pubI<int(publishers.size());pubI++)
	{

		if (publishers[pubI].publishCnt!=-1)
		{ // we handle only the publishers that are not asleep!
			bool removeThisPublisher=false;
			bool publishedSomething=false;

			if (publishers[pubI].cmdID==simros_strmcmd_get_odom_data)
			{ // courtesy of George Moustris
				int child_Handle = publishers[pubI].auxInt1;
				simChar* childn=simGetObjectName(child_Handle);
				if (childn!=NULL)
				{ // ok, the object exists
					std::string child_frame_id=(childn);
					simReleaseBuffer(childn);
					child_frame_id=objNameToFrameId(child_frame_id);
					nav_msgs::Odometry fl;
					int header_Handle = publishers[pubI].auxInt2;
					if (header_Handle==sim_handle_parent)
						header_Handle=simGetObjectParent(child_Handle);
					std::string header_frame_id="world";
					if (header_Handle!=-1)
					{
						simChar* pname=simGetObjectName(header_Handle);
						if (pname!=NULL)
						{
							header_frame_id=pname;
							simReleaseBuffer(pname);
						}
						else
							header_Handle=-1;
					}
					header_frame_id = objNameToFrameId(header_frame_id);

					//construct header
					fl.header.seq=inf.headerInfo.seq;
					fl.header.stamp=inf.headerInfo.stamp;
					fl.header.frame_id=header_frame_id;
					fl.child_frame_id=child_frame_id;

					//get pose and orientation
					float val[4];
					if (simGetObjectPosition(child_Handle,header_Handle,val)!=-1)
					{ // should always pass (verified further up
						fl.pose.pose.position.x=(double)val[0];
						fl.pose.pose.position.y=(double)val[1];
						fl.pose.pose.position.z=(double)val[2];
						simGetObjectQuaternion(child_Handle,header_Handle,val);
						fl.pose.pose.orientation.x=(double)val[0];
						fl.pose.pose.orientation.y=(double)val[1];
						fl.pose.pose.orientation.z=(double)val[2];
						fl.pose.pose.orientation.w=(double)val[3];
					}

					float linVel[3];
					float angVel[3];
					if (simGetObjectVelocity(child_Handle,linVel,angVel)!=-1)
					{ // should always pass
						if (header_Handle!=-1)
						{ // following portion was forgotten (thanks to Andreas Kuhner for noticing this)
							float mi[12];
							simGetObjectMatrix(header_Handle,-1,mi);
							simInvertMatrix(mi);
							simTransformVector(mi,linVel);
							float em[12];
							float _pos[3]={0.0f,0.0f,0.0f};
							float _rot[3]={angVel[0]/1000.0f,angVel[1]/1000.0f,angVel[2]/1000.0f};
							simBuildMatrix(_pos,_rot,em);
							float mout[12];
							simMultiplyMatrices(mi,em,mout);
							simGetEulerAnglesFromMatrix(mout,angVel);
							angVel[0]*=1000.0f;
							angVel[1]*=1000.0f;
							angVel[2]*=1000.0f;
						}
						fl.twist.twist.linear.x=linVel[0];
						fl.twist.twist.linear.y=linVel[1];
						fl.twist.twist.linear.z=linVel[2];
						fl.twist.twist.angular.x=angVel[0];
						fl.twist.twist.angular.y=angVel[1];
						fl.twist.twist.angular.z=angVel[2];
					}

					simInt Strlength;
					simChar* s =simGetStringSignal(publishers[pubI].auxStr.c_str(),&Strlength);
					if(s != NULL)
					{
						simFloat* sfloat=(simFloat*)s;
						simInt FLlength=(int)Strlength/sizeof(simFloat);
						if (FLlength!=72)
						{
							std::cout<<"Error passing covariance matrices: elements should be 36+36=72";
							break;
						}
						else
						{
							boost::array<double,36> covarPose;
							boost::array<double,36> covarTwist;
							std::copy(sfloat, sfloat + 36, &covarPose[0]);
							std::copy(sfloat + 36, sfloat + 72, &covarTwist[0]);
							fl.pose.covariance = covarPose;
							fl.twist.covariance= covarTwist;
						}
						simReleaseBuffer(s);
					}
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_laser_scanner_data)
			{ // courtesy of George Moustris
				int offspring = publishers[pubI].auxInt1;
				simChar* offn=simGetObjectName(offspring);
				if (offn!=NULL)
				{ // ok, the object exists
					std::string offspring_name(offn);
					simReleaseBuffer(offn);
					simInt Strlength;
					simChar* s=simGetStringSignal(publishers[pubI]. auxStr.c_str(),&Strlength);
					if (s!=NULL)
					{ // string signal exists
						simInt FLlength=(int)Strlength/sizeof(simFloat);
						if (FLlength>3)
						{ // string signal is large enough
							sensor_msgs::LaserScan fl;
							simFloat* sfloat=(simFloat*)s;
							std::vector<float> val(sfloat,sfloat+FLlength-3);
							std::vector<float> anglemm(sfloat+FLlength-3,sfloat+FLlength);

							offspring_name = objNameToFrameId(offspring_name);
							fl.header.seq=inf.headerInfo.seq;
							fl.header.stamp=inf.headerInfo.stamp;
							fl.header.frame_id=offspring_name;
							fl.angle_min=anglemm.at(0);
							fl.angle_max=anglemm.at(1);
							fl.angle_increment= anglemm.at(2); //0.006135923;
							fl.time_increment=0.0000173611115315;
							fl.scan_time=0.025;
							fl.range_min=0.023;
							fl.range_max=60;
							fl.ranges=val;
							publishedSomething=true;
							publishers[pubI].generalPublisher.publish(fl);
						}
						simReleaseBuffer(s);
					}
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_vision_sensor_image)
				removeThisPublisher=streamVisionSensorImage(publishers[pubI],inf.headerInfo.stamp,publishedSomething);

			if (publishers[pubI].cmdID==simros_strmcmd_get_range_finder_data)
				removeThisPublisher=streamLaserCloud(publishers[pubI],inf.headerInfo.stamp,publishedSomething);

			if (publishers[pubI].cmdID==simros_strmcmd_get_depth_sensor_data)
				removeThisPublisher=streamDepthSensorCloud(publishers[pubI],inf.headerInfo.stamp,publishedSomething);

			if (publishers[pubI].cmdID==simros_strmcmd_get_joint_state)
			{
				int handle=publishers[pubI].auxInt1;
				float val;
				if ( (handle==sim_handle_all)||(simGetJointPosition(handle,&val)!=-1) )
				{
					sensor_msgs::JointState fl;
					fl.header.seq=_simulationFrameID;
					fl.header.stamp=inf.headerInfo.stamp;

					if (handle==sim_handle_all)
					{
						int index=0;
						while (true)
						{
							int h=simGetObjects(index++,sim_object_joint_type);
							if (h!=-1)
							{
								char* nm=simGetObjectName(h);
								fl.name.push_back(nm);
								simReleaseBuffer(nm);
								float v;
								simGetJointPosition(h,&v);
								fl.position.push_back((double)v);
								simGetObjectFloatParameter(h,2012,&v);
								fl.velocity.push_back((double)v);
								simJointGetForce(h,&v);
								fl.effort.push_back((double)v);
							}
							else
								break;
						}
					}
					else
					{
						float v;
						char* nm=simGetObjectName(handle);
						fl.name.push_back(nm);
						simReleaseBuffer(nm);
						simGetJointPosition(handle,&v);
						fl.position.push_back((double)v);
						simGetObjectFloatParameter(handle,2012,&v);
						fl.velocity.push_back((double)v);
						simJointGetForce(handle,&v);
						fl.effort.push_back((double)v);
					}
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_array_parameter)
			{
				float val[3];
				if (simGetArrayParameter(publishers[pubI].auxInt1,val)!=-1)
				{
					geometry_msgs::Point32 fl;
					fl.x=val[0];
					fl.y=val[1];
					fl.z=val[2];
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_boolean_parameter)
			{
				int r=simGetBooleanParameter(publishers[pubI].auxInt1);
				if (r!=-1)
				{
					std_msgs::UInt8 fl;
					fl.data=r;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_dialog_result)
			{
				int r=simGetDialogResult(publishers[pubI].auxInt1);
				if (r!=-1)
				{
					std_msgs::Int32 fl;
					fl.data=r;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}


			if (publishers[pubI].cmdID==simros_strmcmd_get_floating_parameter)
			{
				float val;
				if (simGetFloatingParameter(publishers[pubI].auxInt1,&val)!=-1)
				{
					std_msgs::Float32 fl;
					fl.data=val;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_integer_parameter)
			{
				int val;
				if (simGetIntegerParameter(publishers[pubI].auxInt1,&val)!=-1)
				{
					std_msgs::Int32 fl;
					fl.data=val;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_float_signal)
			{
				float val;
				int r=simGetFloatSignal(publishers[pubI].auxStr.c_str(),&val);
				if (r!=-1)
				{
					if (r>0)
					{ // publish only if the signal is present
						std_msgs::Float32 fl;
						fl.data=val;
						publishedSomething=true;
						publishers[pubI].generalPublisher.publish(fl);
					}
				}
			//	else
			//		removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_integer_signal)
			{
				int val;
				int r=simGetIntegerSignal(publishers[pubI].auxStr.c_str(),&val);
				if (r!=-1)
				{
					if (r>0)
					{ // publish only if the signal is present
						std_msgs::Int32 fl;
						fl.data=val;
						publishedSomething=true;
						publishers[pubI].generalPublisher.publish(fl);
					}
				}
			//	else
			//		removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_string_signal)
			{
				int signalLength;
				char* str=simGetStringSignal(publishers[pubI].auxStr.c_str(),&signalLength);
				if (str!=NULL)
				{ // publish only if the signal is present
					std_msgs::String fl;
					for (int j=0;j<signalLength;j++)
						fl.data+=str[j];
					simReleaseBuffer(str);
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_and_clear_string_signal)
			{
				int signalLength;
				char* str=simGetStringSignal(publishers[pubI].auxStr.c_str(),&signalLength);
				if (str!=NULL)
				{ // publish only if the signal is present
					std_msgs::String fl;
					for (int j=0;j<signalLength;j++)
						fl.data+=str[j];
					simReleaseBuffer(str);
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
					simClearStringSignal(publishers[pubI].auxStr.c_str());
				}
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_string_parameter)
			{
				char* str=simGetStringParameter(publishers[pubI].auxInt1);
				if (str!=NULL)
				{
					std_msgs::String fl;
					fl.data=str;
					simReleaseBuffer(str);
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_object_float_parameter)
			{
				float val;
				if (simGetObjectFloatParameter(publishers[pubI].auxInt1,publishers[pubI].auxInt2,&val)>0)
				{
					std_msgs::Float32 fl;
					fl.data=val;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_object_group_data)
			{

				std::vector<int> handles;
				std::vector<int> intData;
				std::vector<float> floatData;
				std::vector<std::string> stringData;
				if (getObjectGroupData(publishers[pubI].auxInt1,publishers[pubI].auxInt2,handles,intData,floatData,stringData))
				{
					vrep_common::ObjectGroupData fl;
					for (int i=0;i<int(handles.size());i++)
						fl.handles.data.push_back(handles[i]);
					for (int i=0;i<int(intData.size());i++)
						fl.intData.data.push_back(intData[i]);
					for (int i=0;i<int(floatData.size());i++)
						fl.floatData.data.push_back(floatData[i]);

					for (int i=0;i<int(stringData.size());i++)
					{
						fl.stringData.data+=stringData[i];
						fl.stringData.data+='\0';
					}
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_object_int_parameter)
			{
				int val;
				if (simGetObjectIntParameter(publishers[pubI].auxInt1,publishers[pubI].auxInt2,&val)>0)
				{
					std_msgs::Int32 fl;
					fl.data=val;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_object_pose)
			{
				float val[4];
				if (simGetObjectPosition(publishers[pubI].auxInt1,publishers[pubI].auxInt2,val)!=-1)
				{
					geometry_msgs::PoseStamped fl;
					fl.header.seq=_simulationFrameID;
					fl.header.stamp=inf.headerInfo.stamp;
					fl.pose.position.x=(double)val[0];
					fl.pose.position.y=(double)val[1];
					fl.pose.position.z=(double)val[2];
					simGetObjectQuaternion(publishers[pubI].auxInt1,publishers[pubI].auxInt2,val);
					fl.pose.orientation.x=(double)val[0];
					fl.pose.orientation.y=(double)val[1];
					fl.pose.orientation.z=(double)val[2];
					fl.pose.orientation.w=(double)val[3];
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}
			if (publishers[pubI].cmdID==simros_strmcmd_get_object_parent)
			{
				if (simGetObjectType(publishers[pubI].auxInt1)!=-1)
				{
					int r=simGetObjectParent(publishers[pubI].auxInt1);
					std_msgs::Int32 fl;
					fl.data=r;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_objects)
			{
				int index=0;
				std_msgs::Int32MultiArray l;
				while (true)
				{
					int h=simGetObjects(index++,publishers[pubI].auxInt1);
					if (h!=-1)
						l.data.push_back(h);
					else
						break;
				}
				publishedSomething=true;
				publishers[pubI].generalPublisher.publish(l);
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_object_selection)
			{
				std_msgs::Int32MultiArray l;
				int s=simGetObjectSelectionSize();
				l.data.resize(s);
				if (s>0)
					simGetObjectSelection(&l.data[0]);
				publishedSomething=true;
				publishers[pubI].generalPublisher.publish(l);
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_ui_button_property)
			{
				int p=simGetUIButtonProperty(publishers[pubI].auxInt1,publishers[pubI].auxInt2);
				if (p!=-1)
				{
					std_msgs::Int32 fl;
					fl.data=p;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}
			if (publishers[pubI].cmdID==simros_strmcmd_get_ui_event_button)
			{
				if (simGetUIProperty(publishers[pubI].auxInt1)!=-1)
				{
					int auxDat[2];
					int r=simGetUIEventButton(publishers[pubI].auxInt1,auxDat);
					if (r!=-1)
					{ // publish only if there was an event!
						std_msgs::Int32MultiArray l;
						l.data.push_back(r);
						l.data.push_back(auxDat[0]);
						l.data.push_back(auxDat[1]);
						publishedSomething=true;
						publishers[pubI].generalPublisher.publish(l);
					}
				}
				else
					removeThisPublisher=true;
			}
			if (publishers[pubI].cmdID==simros_strmcmd_get_ui_slider)
			{
				int p=simGetUISlider(publishers[pubI].auxInt1,publishers[pubI].auxInt2);
				if (p!=-1)
				{
					std_msgs::Int32 fl;
					fl.data=p;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_vision_sensor_info)
			{
				int resol[2];
				int handle = publishers[pubI].auxInt1;
				if (simGetVisionSensorResolution(handle,resol)!=-1)
				{
					sensor_msgs::CameraInfo info;
					info.header.stamp = inf.headerInfo.stamp;
					char* objName=simGetObjectName(handle);
					info.header.frame_id = objNameToFrameId(objName);
					simReleaseBuffer(objName);
					info.width = resol[0];
					info.height = resol[1];
					simFloat view_angle = M_PI/4;
					const unsigned int viewing_angle_id = 1004;
					simGetObjectFloatParameter(handle, viewing_angle_id, &view_angle);
					double f_x = (info.width/2.) / tan(view_angle/2.);
					double f_y = f_x;

					info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
					info.D.resize(5);
					info.D[0] = 0;
					info.D[1] = 0;
					info.D[2] = 0;
					info.D[3] = 0;
					info.D[4] = 0;

					info.K[0] = f_x; info.K[1] =   0; info.K[2] = info.width/2;
					info.K[3] =   0; info.K[4] = f_y; info.K[5] = info.height/2;
					info.K[6] =   0; info.K[7] =   0; info.K[8] = 1;

					info.R[0] = 1; info.R[1] = 0; info.R[2] = 0;
					info.R[3] = 0; info.R[4] = 1; info.R[5] = 0;
					info.R[6] = 0; info.R[7] = 0; info.R[8] = 1;

					info.P[0] = info.K[0]; info.P[1] = 0;         info.P[2] = info.K[2]; info.P[3] = 0;
					info.P[4] = 0;         info.P[5] = info.K[4]; info.P[6] = info.K[5]; info.P[7] = 0;
					info.P[8] = 0;         info.P[9] = 0;         info.P[10] = 1;        info.P[11] = 0;

					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(info);
				}
				else
					removeThisPublisher = true;
			}
			if (publishers[pubI].cmdID==simros_strmcmd_get_vision_sensor_depth_buffer)
			{
				int resol[2];
				if (simGetVisionSensorResolution(publishers[pubI].auxInt1,resol)!=-1)
				{
					float* buff=simGetVisionSensorDepthBuffer(publishers[pubI].auxInt1);
					if (buff!=NULL)
					{
						vrep_common::VisionSensorDepthBuff fl;
						fl.x.data=resol[0];
						fl.y.data=resol[1];
						for (int j=0;j<resol[0]*resol[1];j++)
							fl.data.data.push_back(buff[j]);
						simReleaseBuffer((char*)buff);
						publishedSomething=true;
						publishers[pubI].generalPublisher.publish(fl);
					}
					else
						removeThisPublisher=true;
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_read_collision)
			{
				int r=simReadCollision(publishers[pubI].auxInt1);
				if (r!=-1)
				{
					std_msgs::Int32 fl;
					fl.data=r;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_read_distance)
			{
				float dist;
				int r=simReadDistance(publishers[pubI].auxInt1,&dist);
				if (r!=-1)
				{
					std_msgs::Float32 fl;
					fl.data=dist;
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_read_force_sensor)
			{
				float force[3];
				float torque[3];
				int r=simReadForceSensor(publishers[pubI].auxInt1,force,torque);
				if (r!=-1)
				{
					vrep_common::ForceSensorData fl;
					fl.sensorState.data=r;
					fl.force.x=(double)force[0];
					fl.force.y=(double)force[1];
					fl.force.z=(double)force[2];
					fl.torque.x=(double)torque[0];
					fl.torque.y=(double)torque[1];
					fl.torque.z=(double)torque[2];
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_read_proximity_sensor)
			{
				float detectedPt[4];
				int detectedObject;
				float detectedN[3];
				int r=simReadProximitySensor(publishers[pubI].auxInt1,detectedPt,&detectedObject,detectedN);
				if (r!=-1)
				{
					if (r>0)
					{ // we publish only of we detected something
						vrep_common::ProximitySensorData fl;
						fl.detectedPoint.x=detectedPt[0];
						fl.detectedPoint.y=detectedPt[1];
						fl.detectedPoint.z=detectedPt[2];

						fl.detectedObject.data=detectedObject;

						fl.normalVector.x=detectedN[0];
						fl.normalVector.y=detectedN[1];
						fl.normalVector.z=detectedN[2];

						publishedSomething=true;
						publishers[pubI].generalPublisher.publish(fl);
					}
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_read_vision_sensor)
			{
				float* auxValues;
				int* auxValuesCount;
				int r=simReadVisionSensor(publishers[pubI].auxInt1,&auxValues,&auxValuesCount);
				if (r>=0)
				{
					vrep_common::VisionSensorData fl;
					int packetCnt=auxValuesCount[0];
					fl.packetSizes.data.resize(packetCnt);
					fl.packetData.data.clear();
					int cnt=0;
					for (int j=0;j<packetCnt;j++)
					{
						fl.packetSizes.data[j]=auxValuesCount[1+j];
						for (int k=0;k<fl.packetSizes.data[j];k++)
							fl.packetData.data.push_back(auxValues[cnt++]);
					}
					fl.triggerState.data=r;
					simReleaseBuffer((char*)auxValues);
					simReleaseBuffer((char*)auxValuesCount);
					publishedSomething=true;
					publishers[pubI].generalPublisher.publish(fl);
				}
				else
					removeThisPublisher=true;
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_twist_status)
			{
				if (publishers[pubI].auxStr.length()==0)
				{ // new since 17/9/2013
					float linVel[3];
					float angVel[3];
					int res=simGetObjectVelocity(publishers[pubI].auxInt1,linVel,angVel);
					if (res==-1)
						removeThisPublisher=true;
					else
					{
						geometry_msgs::TwistStamped tw;
						char* nm=simGetObjectName(publishers[pubI].auxInt1);
						tw.header.frame_id = objNameToFrameId(nm);
						simReleaseBuffer(nm);
						tw.header.stamp = inf.headerInfo.stamp;
						tw.twist.linear.x=linVel[0];
						tw.twist.linear.y=linVel[1];
						tw.twist.linear.z=linVel[2];
						tw.twist.angular.x=angVel[0];
						tw.twist.angular.y=angVel[1];
						tw.twist.angular.z=angVel[2];
						publishedSomething=true;
						publishers[pubI].generalPublisher.publish(tw);
					}
				}
				else
				{
					simInt datalen = 0;
					simChar * data;
					data = simGetStringSignal(publishers[pubI].auxStr.c_str(),&datalen);
					if (!data ||  (datalen != 6*sizeof(simFloat)))
						removeThisPublisher = true;
					else
					{
						simFloat * dataf = (simFloat*)data;
						geometry_msgs::TwistStamped tw;
						char* nm=simGetObjectName(publishers[pubI].auxInt1);
						if (nm!=NULL)
						{
							tw.header.frame_id = objNameToFrameId(nm);
							simReleaseBuffer(nm);
							tw.header.stamp = inf.headerInfo.stamp;
							tw.twist.linear.x=dataf[0];
							tw.twist.linear.y=dataf[1];
							tw.twist.linear.z=dataf[2];
							tw.twist.angular.x=dataf[3];
							tw.twist.angular.y=dataf[4];
							tw.twist.angular.z=dataf[5];
							publishedSomething=true;
							publishers[pubI].generalPublisher.publish(tw);
						}
						else
							removeThisPublisher = true;
					}
					simReleaseBuffer(data);
				}
			}

			if (publishers[pubI].cmdID==simros_strmcmd_get_transform)
			{
				float position[3];
				float quaternion[4];
				bool ignore = false;
				int offspring = publishers[pubI].auxInt1;
				int parent = publishers[pubI].auxInt2;
				std::cout<<"int offspring = publishers[pubI].auxInt1 "<<offspring<<std::endl;
				std::cout<<"int parent = publishers[pubI].auxInt2" <<parent<<std::endl;
				if (simGetObjectPosition(offspring,parent,position)==-1)
					ignore=true;
				if (simGetObjectQuaternion(offspring,parent,quaternion)==-1)
					ignore=true;
				if (!ignore)
				{
					if (parent==sim_handle_parent)
						parent=simGetObjectParent(offspring);
					char* nm=simGetObjectName(offspring);
					std::string offspring_name(nm);
					simReleaseBuffer(nm);
					std::string parent_name="world";
					if (parent!=-1)
					{
						char* pname=simGetObjectName(parent);
						parent_name=pname;
						simReleaseBuffer(pname);
					}
					
					std::string old_offspring_name = offspring_name;
					std::string old_parent_name = parent_name;
					if (publishers[pubI].auxStr.size()>0)
					{
						// auxstr can be used to specify the names, in case the handles
						// do not correspond to the exact names we want to use in
						// frame_ids...
						// Syntax "offspring_name%parent_name"
						size_t pos = publishers[pubI].auxStr.find_first_of("%");
						if ((pos == std::string::npos) || (pos > 0)) {
							offspring_name = publishers[pubI].auxStr.substr(0,pos);
						}
						if ((pos != std::string::npos) && (pos != publishers[pubI].auxStr.size()-1)) {
							parent_name = publishers[pubI].auxStr.substr(pos+1,std::string::npos);
						}
					}
					offspring_name = objNameToFrameId(offspring_name);
					parent_name = objNameToFrameId(parent_name);

					// Mapping change courtesy of Federico Ferri:
					if (publishers[pubI].auxStr.size()>0)
					{
						renamedFrames[old_offspring_name] = offspring_name;
						renamedFrames[old_parent_name] = parent_name;
					}					

					tf::Transform transform;
					tf::Quaternion q(quaternion[0],quaternion[1],quaternion[2],quaternion[3]);
					transform.setRotation( q );
					transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
					tf_broadcaster->sendTransform(tf::StampedTransform(transform, inf.headerInfo.stamp,
								parent_name, offspring_name));
					publishedSomething=true;
				}
			}
                        
                        if (publishers[pubI].cmdID==simros_strmcmd_get_transform_remapped)
			{
				float position[3];
				float quaternion[4];
				bool ignore = false;
				int offspring = publishers[pubI].auxInt1;
				int parent = publishers[pubI].auxInt2;
				std::cout<<"int offspring = publishers[pubI].auxInt1 "<<offspring<<std::endl;
				std::cout<<"int parent = publishers[pubI].auxInt2" <<parent<<std::endl;
				if (simGetObjectPosition(offspring,parent,position)==-1)
					ignore=true;
				if (simGetObjectQuaternion(offspring,parent,quaternion)==-1)
					ignore=true;
				if (!ignore)
				{
					if (parent==sim_handle_parent)
						parent=simGetObjectParent(offspring);
					char* nm=simGetObjectName(offspring);
					std::string offspring_name(nm);
					simReleaseBuffer(nm);
					std::string parent_name="world";
					if (parent!=-1)
					{
						char* pname=simGetObjectName(parent);
						parent_name=pname;
						simReleaseBuffer(pname);
					}
					
					std::string old_offspring_name = offspring_name;
					std::string old_parent_name = parent_name;
					if (publishers[pubI].auxStr.size()>0)
					{
						// auxstr can be used to specify the names, in case the handles
						// do not correspond to the exact names we want to use in
						// frame_ids...
						// Syntax "offspring_name%parent_name"
						size_t pos = publishers[pubI].auxStr.find_first_of("%");
						if ((pos == std::string::npos) || (pos > 0)) {
							offspring_name = publishers[pubI].auxStr.substr(0,pos);
						}
						if ((pos != std::string::npos) && (pos != publishers[pubI].auxStr.size()-1)) {
							parent_name = publishers[pubI].auxStr.substr(pos+1,std::string::npos);
						}
					}
					offspring_name = objNameToFrameId(offspring_name);
					parent_name = objNameToFrameId(parent_name);

					// Mapping change courtesy of Federico Ferri:
					if (publishers[pubI].auxStr.size()>0)
					{
						renamedFrames[old_offspring_name] = offspring_name;
						renamedFrames[old_parent_name] = parent_name;
					}					

					tf::Transform transform;
					tf::Quaternion q(quaternion[0],quaternion[1],quaternion[2],quaternion[3]);
					transform.setRotation( q );
					transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
					tf_old_broadcaster->sendTransform(tf::StampedTransform(transform, inf.headerInfo.stamp,
								parent_name, offspring_name));
					publishedSomething=true;
				}
			}
			
			if(publishers[pubI].cmdID==simros_strmcmd_get_tracks_vel)
			{
				int left_track_handle = publishers[pubI].auxInt1;
				int right_track_handle = publishers[pubI].auxInt2;
				float v_l;
				float v_r;
				simGetJointPosition(left_track_handle,&v_l);
				simGetJointPosition(right_track_handle,&v_r);
				simGetObjectFloatParameter(left_track_handle,2012,&v_l);
				simGetObjectFloatParameter(right_track_handle,2012,&v_r);
				
				nifti_robot_driver_msgs::TracksStamped tracks_vel;
				tracks_vel.header.seq = _simulationFrameID;
				tracks_vel.header.stamp = inf.headerInfo.stamp;
				tracks_vel.left = (double)v_l;
				tracks_vel.right = -(double)v_r;
				publishedSomething=true;
				publishers[pubI].generalPublisher.publish(tracks_vel);
				
			}
			
			if (removeThisPublisher)
			{
				removePublisher(publishers[pubI].topicName.c_str(),true);
				pubI--; // we have to reprocess this position
			}
			else
			{
				if ((publishers[pubI].publishCnt!=0)&&publishedSomething)
				{
					publishers[pubI].publishCnt--;
					if (publishers[pubI].publishCnt==0)
						publishers[pubI].publishCnt=-1; // we put this publisher to sleep
				}
			}
		}
	}
}

bool ROS_server::getObjectGroupData(int objectType,int dataType,std::vector<int>& handles,std::vector<int>& intData,std::vector<float>& floatData,std::vector<std::string>& stringData)
{
	handles.clear();
	intData.clear();
	floatData.clear();
	stringData.clear();
	int i=0;
	if ((objectType==sim_appobj_object_type)||((objectType>=sim_object_shape_type)&&(objectType<sim_object_type_end)) )
	{
		if (objectType==sim_appobj_object_type)
			objectType=sim_handle_all;
		while (true)
		{
			int handle=simGetObjects(i++,objectType);
			if (handle<0)
				break;
			handles.push_back(handle);
			if (dataType==0)
			{ // object name
				char* name=simGetObjectName(handle);
				stringData.push_back(name);
				simReleaseBuffer(name);
			}
			if (dataType==1)
			{ // object type
				intData.push_back(simGetObjectType(handle));
			}
			if (dataType==2)
			{ // object parent
				intData.push_back(simGetObjectParent(handle));
			}
			if ((dataType==3)||(dataType==4))
			{ // object position
				int w=-1; // abs
				if (dataType==4)
					w=sim_handle_parent; // rel
				float p[3];
				simGetObjectPosition(handle,w,p);
				floatData.push_back(p[0]);
				floatData.push_back(p[1]);
				floatData.push_back(p[2]);
			}
			if ((dataType==5)||(dataType==6))
			{ // object orientation (Euler angles)
				int w=-1; // abs
				if (dataType==6)
					w=sim_handle_parent; // rel
				float o[3];
				simGetObjectOrientation(handle,w,o);
				floatData.push_back(o[0]);
				floatData.push_back(o[1]);
				floatData.push_back(o[2]);
			}
			if ((dataType==7)||(dataType==8))
			{ // object orientation (Quaternions)
				int w=-1; // abs
				if (dataType==8)
					w=sim_handle_parent; // rel
				float q[4];
				simGetObjectQuaternion(handle,w,q);
				floatData.push_back(q[0]);
				floatData.push_back(q[1]);
				floatData.push_back(q[2]);
				floatData.push_back(q[3]);
			}
			if ((dataType==9)||(dataType==10))
			{ // object pose (position+orientation (Euler angles))
				int w=-1; // abs
				if (dataType==10)
					w=sim_handle_parent; // rel
				float p[3];
				simGetObjectPosition(handle,w,p);
				floatData.push_back(p[0]);
				floatData.push_back(p[1]);
				floatData.push_back(p[2]);
				float o[3];
				simGetObjectOrientation(handle,w,o);
				floatData.push_back(o[0]);
				floatData.push_back(o[1]);
				floatData.push_back(o[2]);
			}
			if ((dataType==11)||(dataType==12))
			{ // object pose (position+orientation (Quaternion))
				int w=-1; // abs
				if (dataType==12)
					w=sim_handle_parent; // rel
				float p[3];
				simGetObjectPosition(handle,w,p);
				floatData.push_back(p[0]);
				floatData.push_back(p[1]);
				floatData.push_back(p[2]);
				float q[4];
				simGetObjectOrientation(handle,w,q);
				floatData.push_back(q[0]);
				floatData.push_back(q[1]);
				floatData.push_back(q[2]);
				floatData.push_back(q[3]);
			}
			if (dataType==13)
			{ // prox sensor data
				if (simGetObjectType(handle)==sim_object_proximitysensor_type)
				{
					float pt[4];
					int obj;
					float normal[3];
					int res=simReadProximitySensor(handle,pt,&obj,normal);
					intData.push_back(res);
					intData.push_back(obj);
					floatData.push_back(pt[0]);
					floatData.push_back(pt[1]);
					floatData.push_back(pt[2]);
					floatData.push_back(normal[0]);
					floatData.push_back(normal[1]);
					floatData.push_back(normal[2]);
				}
				else
				{ // this is not a proximity sensor!
					intData.push_back(-1);
					intData.push_back(-1);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
				}
			}
			if (dataType==14)
			{ // force sensor data
				if (simGetObjectType(handle)==sim_object_forcesensor_type)
				{
					float force[3];
					float torque[3];
					int res=simReadForceSensor(handle,force,torque);
					intData.push_back(res);
					floatData.push_back(force[0]);
					floatData.push_back(force[1]);
					floatData.push_back(force[2]);
					floatData.push_back(torque[0]);
					floatData.push_back(torque[1]);
					floatData.push_back(torque[2]);
				}
				else
				{ // this is not a force sensor!
					intData.push_back(-1);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
				}
			}
			if (dataType==15)
			{ // joint data
				if (simGetObjectType(handle)==sim_object_joint_type)
				{
					float pos=0.0f;
					float force=0.0f;
					simGetJointPosition(handle,&pos);
					simJointGetForce(handle,&force);
					floatData.push_back(pos);
					floatData.push_back(force);
				}
				else
				{ // this is not a joint!
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
				}
			}
			if (dataType==16)
			{ // joint type, mode and limit data
				if (simGetObjectType(handle)==sim_object_joint_type)
				{
					float range[2];
					simBool cyclic;
					simGetJointInterval(handle,&cyclic,range);
					if (cyclic)
						range[1]=-1.0f;
					int jointType=simGetJointType(handle);
					int options;
					int jointMode=simGetJointMode(handle,&options);
					if (options&1)
						jointMode|=65536;
					intData.push_back(jointType);
					intData.push_back(jointMode);
					floatData.push_back(range[0]);
					floatData.push_back(range[1]);
				}
				else
				{ // this is not a joint!
					intData.push_back(-1);
					intData.push_back(-1);
					floatData.push_back(0.0f);
					floatData.push_back(0.0f);
				}
			}
			if (dataType==17)
			{ // object linear velocity
				float linVel[3];
				simGetObjectVelocity(handle,linVel,NULL);
				floatData.push_back(linVel[0]);
				floatData.push_back(linVel[1]);
				floatData.push_back(linVel[2]);
			}
			if (dataType==18)
			{ // object angular velocity
				float angVel[3];
				simGetObjectVelocity(handle,NULL,angVel);
				floatData.push_back(angVel[0]);
				floatData.push_back(angVel[1]);
				floatData.push_back(angVel[2]);
			}
			if (dataType==19)
			{ // object linear and angular velocity (twist data)
				float linVel[3];
				float angVel[3];
				simGetObjectVelocity(handle,linVel,angVel);
				floatData.push_back(linVel[0]);
				floatData.push_back(linVel[1]);
				floatData.push_back(linVel[2]);
				floatData.push_back(angVel[0]);
				floatData.push_back(angVel[1]);
				floatData.push_back(angVel[2]);
			}
		}
		return(true);
	}
	return(false);
}


bool ROS_server::streamVisionSensorImage(SPublisherData& pub, const ros::Time & now,bool& publishedSomething)
{
	int handle=pub.auxInt1;
	int resol[2];
	int result=simGetVisionSensorResolution(handle,resol);
	if (result==-1)
		return(true); // remove this topic!
	unsigned char* image_buf = simGetVisionSensorCharImage(handle,NULL,NULL);
	
	// Populate the sensor_msgs::Image message
	sensor_msgs::Image image_msg;
    image_msg.header.stamp = now;
	char* objName=simGetObjectName(handle);
    image_msg.header.frame_id = objNameToFrameId(objName);
	simReleaseBuffer(objName);
	image_msg.encoding=sensor_msgs::image_encodings::RGB8; //Set the format to be RGB with 8bits per channel
	image_msg.height=resol[1]; //Set the height of the image
	image_msg.width=resol[0]; //Set the widht of the image
	image_msg.step=image_msg.width*3; //Set the image stride in bytes
	
	int data_len=image_msg.step*image_msg.height;
	image_msg.data.resize(data_len);
	image_msg.is_bigendian=0;
			
	for(unsigned int i=0;i<image_msg.height;i++)
	{
		int msg_idx=(image_msg.height-i-1)*image_msg.step;
		int buf_idx=i*image_msg.step;
		for(unsigned int j=0;j<image_msg.step;j++)
			image_msg.data[msg_idx+j]=image_buf[buf_idx+j];
	}
	
	simReleaseBuffer((char*)image_buf);
	
	//Publish the populated message
	publishedSomething=true;
	pub.imagePublisher.publish(image_msg);
	return(false); // keep this topic
}

bool ROS_server::streamLaserCloud(SPublisherData& pub, const ros::Time & now,bool& publishedSomething)
{
	SPointCloudPublisherData* pcd=(SPointCloudPublisherData*)pub.specificPublisherData;
	if (!pcd)
		return(false);
	simInt datalen = 0;
	simChar* data=simGetStringSignal(pub.auxStr.c_str(),&datalen);
	if (!data)
		return(false);

    pcd->pointcloud.data.resize(datalen);
    std::copy(data,data+datalen,pcd->pointcloud.data.begin());

	unsigned int n = pcd->pointcloud.data.size() / 12;
	pcd->pointcloud.header.frame_id = objIdToFrameId(pub.auxInt1); // Courtesy of Federico Ferri
	pcd->pointcloud.width = n;
	pcd->pointcloud.row_step = n * pcd->pointcloud.point_step;
	pcd->pointcloud.header.stamp = now;
	publishedSomething=true;
	pub.generalPublisher.publish(pcd->pointcloud);
	pcd->pointcloud.data.clear();

	simReleaseBuffer(data);
	return(false); // keep this topic
}

bool ROS_server::streamDepthSensorCloud(SPublisherData& pub, const ros::Time & now,bool& publishedSomething)
{
    SDepthCloudPublisherData* pcd=(SDepthCloudPublisherData*)pub.specificPublisherData;
    if (!pcd)
        return(false);
    int resol_x=100, resol_y=100;
    int handle = pub.auxInt1;
    simFloat view_angle = M_PI/4, near_clip=0.0, far_clip = 5.0;
    const unsigned int near_clip_id = 1000;
    const unsigned int far_clip_id = 1001;
    const unsigned int resol_x_id = 1002;
    const unsigned int resol_y_id = 1003;
    const unsigned int viewing_angle_id = 1004;
    simGetObjectFloatParameter(handle, near_clip_id, &near_clip);
    simGetObjectFloatParameter(handle, far_clip_id, &far_clip);
    simGetObjectIntParameter(handle, resol_x_id, &resol_x);
    simGetObjectIntParameter(handle, resol_y_id, &resol_y);
    simGetObjectFloatParameter(handle, viewing_angle_id, &view_angle);

    float scale = (far_clip-near_clip)/1.0;
    float* buff=simGetVisionSensorDepthBuffer(handle);
    if (buff==NULL) {
        return true;
    }
	unsigned char * image_buf = simGetVisionSensorCharImage(handle,NULL,NULL);

    unsigned int datalen = resol_x * resol_y;
    if (pcd->x_scale.size() != datalen) {
        // First run, we need to initialise the scaling factors
        pcd->pointcloud.data.resize(datalen*pcd->pointcloud.point_step);
        pcd->x_scale.resize(datalen);
        pcd->y_scale.resize(datalen);
        float f = (resol_x/2.) / tan(view_angle/2.);
        for (int j=0;j<resol_y;j++){
            float y = (j - resol_y/2.0);
            for (int i=0;i<resol_x;i++){
                int k = j*resol_x + i;
                float x = -(i - resol_x/2.0);
                pcd->x_scale[k] = x / f;
                pcd->y_scale[k] = y / f;
            }
        }
    }
    for (unsigned int i=0;i<datalen;i++){
        float depth = near_clip + scale * buff[i];
        float xyz[3] = {depth*pcd->x_scale[i],depth*pcd->y_scale[i],depth};
        assert(sizeof(xyz)==12);
        uint8_t * raw = (uint8_t*)xyz;
        std::copy(raw,raw+12,pcd->pointcloud.data.begin()+i*pcd->pointcloud.point_step);
        unsigned int p = i*3;
        uint8_t argb[4] = {image_buf[p+2],image_buf[p+1],image_buf[p],0};
        std::copy(argb,argb+4,pcd->pointcloud.data.begin()+i*pcd->pointcloud.point_step+12);
    }
	pcd->pointcloud.header.frame_id = objIdToFrameId(pub.auxInt1); // Courtesy of Federico Ferri
    pcd->pointcloud.width = resol_x;
    pcd->pointcloud.height = resol_y;
    pcd->pointcloud.row_step = resol_x * pcd->pointcloud.point_step;
    pcd->pointcloud.header.stamp = now;
	publishedSomething=true;
    pub.generalPublisher.publish(pcd->pointcloud);
    simReleaseBuffer((char*)buff);
    simReleaseBuffer((char*)image_buf);

    return(false); // keep this topic
}

//=================================================================================================================
//========================================= API SUBSCRIPTIONS======================================================
//=================================================================================================================
int ROS_server::addSubscriber(const char* topicName,int queueSize,int streamCmd,int auxInt1,int auxInt2,const char* auxString,int callbackTag_before,int callbackTag_after)
{
	// 1. Check if the streamCmd is valid:
	if (streamCmd<=simros_strmcmdnull_subscriber_start)
		return(-1);
	if ( (streamCmd>=simros_strmcmdint_start)&&(streamCmd<=simros_strmcmdint_subscriber_start) )
		return(-1);
	if ( (streamCmd>=simros_strmcmdintint_start)&&(streamCmd<=simros_strmcmdintint_subscriber_start) )
		return(-1);
	if ( (streamCmd>=simros_strmcmdstring_start)&&(streamCmd<=simros_strmcmdstring_subscriber_start) )
		return(-1);
	if ( (streamCmd>=simros_strmcmdintstring_start)&&(streamCmd<=simros_strmcmdintstring_subscriber_start) )
		return(-1);
	if (streamCmd>=simros_strmcmdreserved_start)
		return(-1);
	// 2. Try to add a new subscriber with the given topic name:
	std::string nm(topicName);

	// Following changes courtesy of Cedric Pradalier:
	boost::replace_all(nm,"#","_HASHMARK_");
	boost::replace_all(nm,"-","_MINUSMARK_");
	boost::replace_all(nm,"(","_OPARENTHESISMARK_");
	boost::replace_all(nm,")","_CPARENTHESISMARK_");

	CSubscriberData* sub=new CSubscriberData(node,nm.c_str(),queueSize,streamCmd,auxInt1,auxInt2,auxString,callbackTag_before,callbackTag_after,&images_streamer,imgStreamerCnt);
	if (sub->getIsValid())
	{
		subscribers.push_back(sub);
		lastSubscriberID++;
		sub->setSubscriberID(lastSubscriberID);
		return(lastSubscriberID);
	}
	delete sub;
	return(-1);
}

bool ROS_server::removeSubscriber(int subscriberID)
{
	for (unsigned int i=0;i<subscribers.size();i++)
	{
		if (subscribers[i]->getSubscriberID()==subscriberID)
		{
			subscribers[i]->shutDownSubscriber();
			delete subscribers[i];
			subscribers.erase(subscribers.begin()+i);
			return(true);
		}
	}
	return(false);
}

void ROS_server::removeAllSubscribers()
{
	while (subscribers.size()!=0)
		removeSubscriber(subscribers[0]->getSubscriberID());
}


//=================================================================================================================
//========================================= API services ==========================================================
//=================================================================================================================
void ROS_server::enableAPIServices()
{
	simRosAddStatusbarMessageServer = node->advertiseService("simRosAddStatusbarMessage",ROS_server::simRosAddStatusbarMessageService);
	simRosAuxiliaryConsoleCloseServer = node->advertiseService("simRosAuxiliaryConsoleClose",ROS_server::simRosAuxiliaryConsoleCloseService);
	simRosAuxiliaryConsoleOpenServer = node->advertiseService("simRosAuxiliaryConsoleOpen",ROS_server::simRosAuxiliaryConsoleOpenService);
	simRosAuxiliaryConsolePrintServer = node->advertiseService("simRosAuxiliaryConsolePrint",ROS_server::simRosAuxiliaryConsolePrintService);
	simRosAuxiliaryConsoleShowServer = node->advertiseService("simRosAuxiliaryConsoleShow",ROS_server::simRosAuxiliaryConsoleShowService);
	simRosBreakForceSensorServer = node->advertiseService("simRosBreakForceSensor",ROS_server::simRosBreakForceSensorService);
	simRosClearFloatSignalServer = node->advertiseService("simRosClearFloatSignal",ROS_server::simRosClearFloatSignalService);
	simRosClearIntegerSignalServer = node->advertiseService("simRosClearIntegerSignal",ROS_server::simRosClearIntegerSignalService);
	simRosClearStringSignalServer = node->advertiseService("simRosClearStringSignal",ROS_server::simRosClearStringSignalService);
	simRosCloseSceneServer = node->advertiseService("simRosCloseScene",ROS_server::simRosCloseSceneService);
	simRosCopyPasteObjectsServer = node->advertiseService("simRosCopyPasteObjects",ROS_server::simRosCopyPasteObjectsService);
	simRosDisplayDialogServer = node->advertiseService("simRosDisplayDialog",ROS_server::simRosDisplayDialogService);
	simRosEndDialogServer = node->advertiseService("simRosEndDialog",ROS_server::simRosEndDialogService);
	simRosEraseFileServer = node->advertiseService("simRosEraseFile",ROS_server::simRosEraseFileService);
	simRosGetArrayParameterServer = node->advertiseService("simRosGetArrayParameter",ROS_server::simRosGetArrayParameterService);
	simRosGetBooleanParameterServer = node->advertiseService("simRosGetBooleanParameter",ROS_server::simRosGetBooleanParameterService);
	simRosGetCollisionHandleServer = node->advertiseService("simRosGetCollisionHandle",ROS_server::simRosGetCollisionHandleService);
	simRosGetDialogInputServer = node->advertiseService("simRosGetDialogInput",ROS_server::simRosGetDialogInputService);
	simRosGetDialogResultServer = node->advertiseService("simRosGetDialogResult",ROS_server::simRosGetDialogResultService);
	simRosGetDistanceHandleServer = node->advertiseService("simRosGetDistanceHandle",ROS_server::simRosGetDistanceHandleService);
	simRosGetFloatingParameterServer = node->advertiseService("simRosGetFloatingParameter",ROS_server::simRosGetFloatingParameterService);
	simRosGetFloatSignalServer = node->advertiseService("simRosGetFloatSignal",ROS_server::simRosGetFloatSignalService);
	simRosGetIntegerParameterServer = node->advertiseService("simRosGetIntegerParameter",ROS_server::simRosGetIntegerParameterService);
	simRosGetIntegerSignalServer = node->advertiseService("simRosGetIntegerSignal",ROS_server::simRosGetIntegerSignalService);
	simRosGetJointMatrixServer = node->advertiseService("simRosGetJointMatrix",ROS_server::simRosGetJointMatrixService);
	simRosGetJointStateServer = node->advertiseService("simRosGetJointState",ROS_server::simRosGetJointStateService);
	simRosGetLastErrorsServer = node->advertiseService("simRosGetLastErrors",ROS_server::simRosGetLastErrorsService);
	simRosGetModelPropertyServer = node->advertiseService("simRosGetModelProperty",ROS_server::simRosGetModelPropertyService);
	simRosGetObjectChildServer = node->advertiseService("simRosGetObjectChild",ROS_server::simRosGetObjectChildService);
	simRosGetObjectFloatParameterServer = node->advertiseService("simRosGetObjectFloatParameter",ROS_server::simRosGetObjectFloatParameterService);
	simRosGetObjectHandleServer = node->advertiseService("simRosGetObjectHandle",ROS_server::simRosGetObjectHandleService);
	simRosGetObjectIntParameterServer = node->advertiseService("simRosGetObjectIntParameter",ROS_server::simRosGetObjectIntParameterService);
//	simRosGetObjectOrientationServer = node->advertiseService("simRosGetObjectOrientation",ROS_server::simRosGetObjectOrientationService);
	simRosGetObjectParentServer = node->advertiseService("simRosGetObjectParent",ROS_server::simRosGetObjectParentService);
//	simRosGetObjectPositionServer = node->advertiseService("simRosGetObjectPosition",ROS_server::simRosGetObjectPositionService);
	simRosGetObjectPoseServer = node->advertiseService("simRosGetObjectPose",ROS_server::simRosGetObjectPoseService);
	simRosGetObjectsServer = node->advertiseService("simRosGetObjects",ROS_server::simRosGetObjectsService);
	simRosGetObjectSelectionServer = node->advertiseService("simRosGetObjectSelection",ROS_server::simRosGetObjectSelectionService);
	simRosGetStringParameterServer = node->advertiseService("simRosGetStringParameter",ROS_server::simRosGetStringParameterService);
	simRosGetStringSignalServer = node->advertiseService("simRosGetStringSignal",ROS_server::simRosGetStringSignalService);
	simRosGetUIButtonPropertyServer = node->advertiseService("simRosGetUIButtonProperty",ROS_server::simRosGetUIButtonPropertyService);
	simRosGetUIEventButtonServer = node->advertiseService("simRosGetUIEventButton",ROS_server::simRosGetUIEventButtonService);
	simRosGetUIHandleServer = node->advertiseService("simRosGetUIHandle",ROS_server::simRosGetUIHandleService);
	simRosGetUISliderServer = node->advertiseService("simRosGetUISlider",ROS_server::simRosGetUISliderService);
	simRosGetVisionSensorDepthBufferServer = node->advertiseService("simRosGetVisionSensorDepthBuffer",ROS_server::simRosGetVisionSensorDepthBufferService);
	simRosGetVisionSensorImageServer = node->advertiseService("simRosGetVisionSensorImage",ROS_server::simRosGetVisionSensorImageService);
	simRosLoadModelServer = node->advertiseService("simRosLoadModel",ROS_server::simRosLoadModelService);
	simRosLoadSceneServer = node->advertiseService("simRosLoadScene",ROS_server::simRosLoadSceneService);
	simRosLoadUIServer = node->advertiseService("simRosLoadUI",ROS_server::simRosLoadUIService);
	simRosPauseSimulationServer = node->advertiseService("simRosPauseSimulation",ROS_server::simRosPauseSimulationService);
	simRosReadCollisionServer = node->advertiseService("simRosReadCollision",ROS_server::simRosReadCollisionService);
	simRosReadDistanceServer = node->advertiseService("simRosReadDistance",ROS_server::simRosReadDistanceService);
	simRosReadForceSensorServer = node->advertiseService("simRosReadForceSensor",ROS_server::simRosReadForceSensorService);
	simRosReadProximitySensorServer = node->advertiseService("simRosReadProximitySensor",ROS_server::simRosReadProximitySensorService);
	simRosReadVisionSensorServer = node->advertiseService("simRosReadVisionSensor",ROS_server::simRosReadVisionSensorService);
	simRosRemoveObjectServer = node->advertiseService("simRosRemoveObject",ROS_server::simRosRemoveObjectService);
	simRosRemoveModelServer = node->advertiseService("simRosRemoveModel",ROS_server::simRosRemoveModelService);
	simRosRemoveUIServer = node->advertiseService("simRosRemoveUI",ROS_server::simRosRemoveUIService);
	simRosSetArrayParameterServer = node->advertiseService("simRosSetArrayParameter",ROS_server::simRosSetArrayParameterService);
	simRosSetBooleanParameterServer = node->advertiseService("simRosSetBooleanParameter",ROS_server::simRosSetBooleanParameterService);
	simRosSetFloatingParameterServer = node->advertiseService("simRosSetFloatingParameter",ROS_server::simRosSetFloatingParameterService);
	simRosSetFloatSignalServer = node->advertiseService("simRosSetFloatSignal",ROS_server::simRosSetFloatSignalService);
	simRosSetIntegerParameterServer = node->advertiseService("simRosSetIntegerParameter",ROS_server::simRosSetIntegerParameterService);
	simRosSetIntegerSignalServer = node->advertiseService("simRosSetIntegerSignal",ROS_server::simRosSetIntegerSignalService);
	simRosSetJointForceServer = node->advertiseService("simRosSetJointForce",ROS_server::simRosSetJointForceService);
	simRosSetJointPositionServer = node->advertiseService("simRosSetJointPosition",ROS_server::simRosSetJointPositionService);
	simRosSetJointTargetPositionServer = node->advertiseService("simRosSetJointTargetPosition",ROS_server::simRosSetJointTargetPositionService);
	simRosSetJointTargetVelocityServer = node->advertiseService("simRosSetJointTargetVelocity",ROS_server::simRosSetJointTargetVelocityService);
	simRosSetModelPropertyServer = node->advertiseService("simRosSetModelProperty",ROS_server::simRosSetModelPropertyService);
	simRosSetObjectFloatParameterServer = node->advertiseService("simRosSetObjectFloatParameter",ROS_server::simRosSetObjectFloatParameterService);
	simRosSetObjectIntParameterServer = node->advertiseService("simRosSetObjectIntParameter",ROS_server::simRosSetObjectIntParameterService);
	simRosSetObjectPoseServer = node->advertiseService("simRosSetObjectPose",ROS_server::simRosSetObjectPoseService);
	simRosSetObjectParentServer = node->advertiseService("simRosSetObjectParent",ROS_server::simRosSetObjectParentService);
	simRosSetObjectPositionServer = node->advertiseService("simRosSetObjectPosition",ROS_server::simRosSetObjectPositionService);
	simRosSetObjectSelectionServer = node->advertiseService("simRosSetObjectSelection",ROS_server::simRosSetObjectSelectionService);
	simRosGetInfoServer = node->advertiseService("simRosGetInfo",ROS_server::simRosGetInfoService);
	simRosSetSphericalJointMatrixServer = node->advertiseService("simRosSetSphericalJointMatrix",ROS_server::simRosSetSphericalJointMatrixService);
	simRosSetStringSignalServer = node->advertiseService("simRosSetStringSignal",ROS_server::simRosSetStringSignalService);
	simRosSetUIButtonLabelServer = node->advertiseService("simRosSetUIButtonLabel",ROS_server::simRosSetUIButtonLabelService);
	simRosSetUIButtonPropertyServer = node->advertiseService("simRosSetUIButtonProperty",ROS_server::simRosSetUIButtonPropertyService);
	simRosSetUISliderServer = node->advertiseService("simRosSetUISlider",ROS_server::simRosSetUISliderService);
	simRosSetVisionSensorImageServer = node->advertiseService("simRosSetVisionSensorImage",ROS_server::simRosSetVisionSensorImageService);
	simRosStartSimulationServer = node->advertiseService("simRosStartSimulation",ROS_server::simRosStartSimulationService);
	simRosStopSimulationServer = node->advertiseService("simRosStopSimulation",ROS_server::simRosStopSimulationService);
	simRosSynchronousServer = node->advertiseService("simRosSynchronous",ROS_server::simRosSynchronousService);
	simRosSynchronousTriggerServer = node->advertiseService("simRosSynchronousTrigger",ROS_server::simRosSynchronousTriggerService);
	simRosTransferFileServer = node->advertiseService("simRosTransferFile",ROS_server::simRosTransferFileService);
	simRosEnablePublisherServer = node->advertiseService("simRosEnablePublisher",ROS_server::simRosEnablePublisherService);
	simRosDisablePublisherServer = node->advertiseService("simRosDisablePublisher",ROS_server::simRosDisablePublisherService);
//	simRosGetObjectQuaternionServer = node->advertiseService("simRosGetObjectQuaternion",ROS_server::simRosGetObjectQuaternionService);
	simRosSetObjectQuaternionServer = node->advertiseService("simRosSetObjectQuaternion",ROS_server::simRosSetObjectQuaternionService);
	simRosEnableSubscriberServer = node->advertiseService("simRosEnableSubscriber",ROS_server::simRosEnableSubscriberService);
	simRosDisableSubscriberServer = node->advertiseService("simRosDisableSubscriber",ROS_server::simRosDisableSubscriberService);
	simRosSetJointStateServer = node->advertiseService("simRosSetJointState",ROS_server::simRosSetJointStateService);
	simRosCreateDummyServer = node->advertiseService("simRosCreateDummy",ROS_server::simRosCreateDummyService);
	simRosGetAndClearStringSignalServer = node->advertiseService("simRosGetAndClearStringSignal",ROS_server::simRosGetAndClearStringSignalService);
	simRosGetObjectGroupDataServer = node->advertiseService("simRosGetObjectGroupData",ROS_server::simRosGetObjectGroupDataService);
}

void ROS_server::disableAPIServices()
{
	simRosAddStatusbarMessageServer.shutdown();
	simRosAuxiliaryConsoleCloseServer.shutdown();
	simRosAuxiliaryConsoleOpenServer.shutdown();
	simRosAuxiliaryConsolePrintServer.shutdown();
	simRosAuxiliaryConsoleShowServer.shutdown();
	simRosBreakForceSensorServer.shutdown();
	simRosClearFloatSignalServer.shutdown();
	simRosClearIntegerSignalServer.shutdown();
	simRosClearStringSignalServer.shutdown();
	simRosCloseSceneServer.shutdown();
	simRosCopyPasteObjectsServer.shutdown();
	simRosDisplayDialogServer.shutdown();
	simRosEndDialogServer.shutdown();
	simRosEraseFileServer.shutdown();
	simRosGetArrayParameterServer.shutdown();
	simRosGetBooleanParameterServer.shutdown();
	simRosGetCollisionHandleServer.shutdown();
	simRosGetDialogInputServer.shutdown();
	simRosGetDialogResultServer.shutdown();
	simRosGetDistanceHandleServer.shutdown();
	simRosGetFloatingParameterServer.shutdown();
	simRosGetFloatSignalServer.shutdown();
	simRosGetIntegerParameterServer.shutdown();
	simRosGetIntegerSignalServer.shutdown();
	simRosGetJointMatrixServer.shutdown();
	simRosGetJointStateServer.shutdown();
	simRosGetLastErrorsServer.shutdown();
	simRosGetModelPropertyServer.shutdown();
	simRosGetObjectChildServer.shutdown();
	simRosGetObjectFloatParameterServer.shutdown();
	simRosGetObjectHandleServer.shutdown();
	simRosGetObjectIntParameterServer.shutdown();
//	simRosGetObjectOrientationServer.shutdown();
	simRosGetObjectParentServer.shutdown();
//	simRosGetObjectPositionServer.shutdown();
	simRosGetObjectPoseServer.shutdown();
	simRosGetObjectsServer.shutdown();
	simRosGetObjectSelectionServer.shutdown();
	simRosGetStringParameterServer.shutdown();
	simRosGetStringSignalServer.shutdown();
	simRosGetUIButtonPropertyServer.shutdown();
	simRosGetUIEventButtonServer.shutdown();
	simRosGetUIHandleServer.shutdown();
	simRosGetUISliderServer.shutdown();
	simRosGetVisionSensorDepthBufferServer.shutdown();
	simRosGetVisionSensorImageServer.shutdown();
	simRosLoadModelServer.shutdown();
	simRosLoadSceneServer.shutdown();
	simRosLoadUIServer.shutdown();
	simRosPauseSimulationServer.shutdown();
	simRosReadCollisionServer.shutdown();
	simRosReadDistanceServer.shutdown();
	simRosReadForceSensorServer.shutdown();
	simRosReadProximitySensorServer.shutdown();
	simRosReadVisionSensorServer.shutdown();
	simRosRemoveObjectServer.shutdown();
	simRosRemoveModelServer.shutdown();
	simRosRemoveUIServer.shutdown();
	simRosSetArrayParameterServer.shutdown();
	simRosSetBooleanParameterServer.shutdown();
	simRosSetFloatingParameterServer.shutdown();
	simRosSetFloatSignalServer.shutdown();
	simRosSetIntegerParameterServer.shutdown();
	simRosSetIntegerSignalServer.shutdown();
	simRosSetJointForceServer.shutdown();
	simRosSetJointPositionServer.shutdown();
	simRosSetJointTargetPositionServer.shutdown();
	simRosSetJointTargetVelocityServer.shutdown();
	simRosSetModelPropertyServer.shutdown();
	simRosSetObjectFloatParameterServer.shutdown();
	simRosSetObjectIntParameterServer.shutdown();
	simRosSetObjectPoseServer.shutdown();
	simRosSetObjectParentServer.shutdown();
	simRosSetObjectPositionServer.shutdown();
	simRosSetObjectSelectionServer.shutdown();
	simRosGetInfoServer.shutdown();
	simRosSetSphericalJointMatrixServer.shutdown();
	simRosSetStringSignalServer.shutdown();
	simRosSetUIButtonLabelServer.shutdown();
	simRosSetUIButtonPropertyServer.shutdown();
	simRosSetUISliderServer.shutdown();
	simRosSetVisionSensorImageServer.shutdown();
	simRosStartSimulationServer.shutdown();
	simRosStopSimulationServer.shutdown();
	simRosSynchronousServer.shutdown();
	simRosSynchronousTriggerServer.shutdown();
	simRosTransferFileServer.shutdown();
	simRosEnablePublisherServer.shutdown();
	simRosDisablePublisherServer.shutdown();
//	simRosGetObjectQuaternionServer.shutdown();
	simRosSetObjectQuaternionServer.shutdown();
	simRosEnableSubscriberServer.shutdown();
	simRosDisableSubscriberServer.shutdown();
	simRosSetJointStateServer.shutdown();
	simRosCreateDummyServer.shutdown();
	simRosGetAndClearStringSignalServer.shutdown();
	simRosGetObjectGroupDataServer.shutdown();
	_last50Errors.clear();
}
int ROS_server::_handleServiceErrors_start()
{
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_report);
	return(errorModeSaved);
}

void ROS_server::_handleServiceErrors_end(int errorReportMode)
{
	char* err=simGetLastError(); // this also clears the last error
	if (err!=NULL)
	{
		if (_last50Errors.size()>=50)
			_last50Errors.erase(_last50Errors.begin());
		_last50Errors.push_back(err);
		simReleaseBuffer(err);
	}
	simSetIntegerParameter(sim_intparam_error_report_mode,errorReportMode);
}

bool ROS_server::simRosAddStatusbarMessageService(vrep_common::simRosAddStatusbarMessage::Request &req,vrep_common::simRosAddStatusbarMessage::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simAddStatusbarMessage(req.message.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosAuxiliaryConsoleCloseService(vrep_common::simRosAuxiliaryConsoleClose::Request &req,vrep_common::simRosAuxiliaryConsoleClose::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simAuxiliaryConsoleClose(req.consoleHandle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosAuxiliaryConsoleOpenService(vrep_common::simRosAuxiliaryConsoleOpen::Request &req,vrep_common::simRosAuxiliaryConsoleOpen::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int* pos=NULL;
	int* siz=NULL;
	float* textCol=NULL;
	float* backCol=NULL;
	if (req.position.size()>=2)
		pos=&req.position[0];
	if (req.size.size()>=2)
		siz=&req.size[0];
	if (req.textColor.size()>=3)
		textCol=&req.textColor[0];
	if (req.backgroundColor.size()>=3)
		backCol=&req.backgroundColor[0];
	res.consoleHandle=simAuxiliaryConsoleOpen(req.title.c_str(),req.maxLines,req.mode,pos,siz,textCol,backCol);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosAuxiliaryConsolePrintService(vrep_common::simRosAuxiliaryConsolePrint::Request &req,vrep_common::simRosAuxiliaryConsolePrint::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	if (req.text.length()==0)
		res.result=simAuxiliaryConsolePrint(req.consoleHandle,NULL);
	else
		res.result=simAuxiliaryConsolePrint(req.consoleHandle,req.text.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosAuxiliaryConsoleShowService(vrep_common::simRosAuxiliaryConsoleShow::Request &req,vrep_common::simRosAuxiliaryConsoleShow::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simAuxiliaryConsoleShow(req.consoleHandle,req.showState);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosBreakForceSensorService(vrep_common::simRosBreakForceSensor::Request &req,vrep_common::simRosBreakForceSensor::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simBreakForceSensor(req.objectHandle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosClearFloatSignalService(vrep_common::simRosClearFloatSignal::Request &req,vrep_common::simRosClearFloatSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	if (req.signalName.length()==0)
		res.result=simClearFloatSignal(NULL);
	else
		res.result=simClearFloatSignal(req.signalName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosClearIntegerSignalService(vrep_common::simRosClearIntegerSignal::Request &req,vrep_common::simRosClearIntegerSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	if (req.signalName.length()==0)
		res.result=simClearIntegerSignal(NULL);
	else
		res.result=simClearIntegerSignal(req.signalName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosClearStringSignalService(vrep_common::simRosClearStringSignal::Request &req,vrep_common::simRosClearStringSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	if (req.signalName.length()==0)
		res.result=simClearStringSignal(NULL);
	else
		res.result=simClearStringSignal(req.signalName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosCloseSceneService(vrep_common::simRosCloseScene::Request &req,vrep_common::simRosCloseScene::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simCloseScene();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosCopyPasteObjectsService(vrep_common::simRosCopyPasteObjects::Request &req,vrep_common::simRosCopyPasteObjects::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	// 1. Save current selection state:
	int initialSelSize=simGetObjectSelectionSize();
	int* initialSelection=new int[initialSelSize];
	simGetObjectSelection(initialSelection);
	simRemoveObjectFromSelection(sim_handle_all,0);

	// 2. Select objects we wanna copy and paste:
	int cnt=req.objectHandles.size();
	for (int i=0;i<cnt;i++)
		simAddObjectToSelection(sim_handle_single,req.objectHandles[i]);
	int toCopySelSize=simGetObjectSelectionSize();

	// 3. Copy and paste the selection:
	simCopyPasteSelectedObjects();

	// 4. Send back the handles of the new objects:
	int newSelSize=simGetObjectSelectionSize();
	res.newObjectHandles.resize(newSelSize);
	if (newSelSize!=0)
		simGetObjectSelection(&res.newObjectHandles[0]);
	// 4.5 We return as result the number of copied objects (not necessarily the number of returned object handles!):
	res.result=toCopySelSize;

	// 4. Restore previous selection state
	simRemoveObjectFromSelection(sim_handle_all,0);
	for (int i=0;i<initialSelSize;i++)
		simAddObjectToSelection(sim_handle_single,initialSelection[i]);
	delete[] initialSelection;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosDisplayDialogService(vrep_common::simRosDisplayDialog::Request &req,vrep_common::simRosDisplayDialog::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float* col1=NULL;
	float* col2=NULL;
	if (req.titleColors.size()>=6)
		col1=&req.titleColors[0];
	if (req.dialogColors.size()>=6)
		col2=&req.dialogColors[0];
	int uiHandle;
	int handle=simDisplayDialog(req.titleText.c_str(),req.mainText.c_str(),req.dialogType,req.initialText.c_str(),col1,col2,&uiHandle);
	res.dialogHandle=handle;
	res.uiHandle=uiHandle;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosEndDialogService(vrep_common::simRosEndDialog::Request &req,vrep_common::simRosEndDialog::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simEndDialog(req.dialogHandle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosEraseFileService(vrep_common::simRosEraseFile::Request &req,vrep_common::simRosEraseFile::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=-1;
	if (remove(req.fileName.c_str())==0)
		res.result=0;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetArrayParameterService(vrep_common::simRosGetArrayParameter::Request &req,vrep_common::simRosGetArrayParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float vals[3];
	res.result=simGetArrayParameter(req.parameter,vals);
	res.parameterValues.resize(3);
	res.parameterValues[0]=vals[0];
	res.parameterValues[1]=vals[1];
	res.parameterValues[2]=vals[2];
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetBooleanParameterService(vrep_common::simRosGetBooleanParameter::Request &req,vrep_common::simRosGetBooleanParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.parameterValue=simGetBooleanParameter(req.parameter);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetCollisionHandleService(vrep_common::simRosGetCollisionHandle::Request &req,vrep_common::simRosGetCollisionHandle::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.handle=simGetCollisionHandle(req.collisionName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetDialogInputService(vrep_common::simRosGetDialogInput::Request &req,vrep_common::simRosGetDialogInput::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	char* input=simGetDialogInput(req.dialogHandle);
	if (input!=NULL)
	{
		res.result=1;
		res.input=input;
		simReleaseBuffer(input);
	}
	else
	{
		res.result=-1;
		res.input="";
	}
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetDialogResultService(vrep_common::simRosGetDialogResult::Request &req,vrep_common::simRosGetDialogResult::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simGetDialogResult(req.dialogHandle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetDistanceHandleService(vrep_common::simRosGetDistanceHandle::Request &req,vrep_common::simRosGetDistanceHandle::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.handle=simGetDistanceHandle(req.distanceName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetFloatingParameterService(vrep_common::simRosGetFloatingParameter::Request &req,vrep_common::simRosGetFloatingParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float p;
	res.result=simGetFloatingParameter(req.parameter,&p);
	res.parameterValue=p;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetFloatSignalService(vrep_common::simRosGetFloatSignal::Request &req,vrep_common::simRosGetFloatSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float signalValue;
	res.result=simGetFloatSignal(req.signalName.c_str(),&signalValue);
	res.signalValue=signalValue;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetIntegerParameterService(vrep_common::simRosGetIntegerParameter::Request &req,vrep_common::simRosGetIntegerParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int p;
	res.result=simGetIntegerParameter(req.parameter,&p);
	res.parameterValue=p;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetIntegerSignalService(vrep_common::simRosGetIntegerSignal::Request &req,vrep_common::simRosGetIntegerSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int signalValue;
	res.result=simGetIntegerSignal(req.signalName.c_str(),&signalValue);
	res.signalValue=signalValue;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetJointMatrixService(vrep_common::simRosGetJointMatrix::Request &req,vrep_common::simRosGetJointMatrix::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float v;
	if (simGetObjectFloatParameter(req.handle,2005,&v)==1)
	{
		res.transform.transform.translation.x=(double)v;
		simGetObjectFloatParameter(req.handle,2006,&v);
		res.transform.transform.translation.y=(double)v;
		simGetObjectFloatParameter(req.handle,2007,&v);
		res.transform.transform.translation.z=(double)v;
		simGetObjectFloatParameter(req.handle,2008,&v);
		res.transform.transform.rotation.x=(double)v;
		simGetObjectFloatParameter(req.handle,2009,&v);
		res.transform.transform.rotation.y=(double)v;
		simGetObjectFloatParameter(req.handle,2010,&v);
		res.transform.transform.rotation.z=(double)v;
		simGetObjectFloatParameter(req.handle,2011,&v);
		res.transform.transform.rotation.w=(double)v;
		res.transform.header.seq=_simulationFrameID;
		res.transform.header.stamp=ros::Time::now();
		res.result=1;
	}

	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetJointStateService(vrep_common::simRosGetJointState::Request &req,vrep_common::simRosGetJointState::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();

	res.state.header.seq=_simulationFrameID;
	res.state.header.stamp=ros::Time::now();
	if (req.handle==sim_handle_all)
	{
		int index=0;
		while (true)
		{
			int h=simGetObjects(index++,sim_object_joint_type);
			if (h!=-1)
			{
				char* nm=simGetObjectName(h);
				res.state.name.push_back(nm);
				simReleaseBuffer(nm);
				float v;
				simGetJointPosition(h,&v);
				res.state.position.push_back((double)v);
				simGetObjectFloatParameter(h,2012,&v);
				res.state.velocity.push_back((double)v);
				simJointGetForce(h,&v);
				res.state.effort.push_back((double)v);
			}
			else
				break;
		}
		res.result=1;
	}
	else
	{
		float v;
		if (simGetJointPosition(req.handle,&v)!=-1)
		{
			char* nm=simGetObjectName(req.handle);
			res.state.name.push_back(nm);
			simReleaseBuffer(nm);
			res.state.position.push_back((double)v);
			simGetObjectFloatParameter(req.handle,2012,&v);
			res.state.velocity.push_back((double)v);
			simJointGetForce(req.handle,&v);
			res.state.effort.push_back((double)v);
			res.result=1;
		}
		else
			res.result=-1;
	}
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetLastErrorsService(vrep_common::simRosGetLastErrors::Request &req,vrep_common::simRosGetLastErrors::Response &res)
{
	res.errorStrings.clear();
	res.errorCnt=0;
	for (unsigned int i=0;i<_last50Errors.size();i++)
	{
		res.errorStrings+=_last50Errors[i];
		res.errorStrings+='\0';
		res.errorCnt++;
	}
	_last50Errors.clear();
	return true;
}

bool ROS_server::simRosGetModelPropertyService(vrep_common::simRosGetModelProperty::Request &req,vrep_common::simRosGetModelProperty::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.propertyValue=simGetModelProperty(req.handle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectChildService(vrep_common::simRosGetObjectChild::Request &req,vrep_common::simRosGetObjectChild::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.childHandle=simGetObjectChild(req.handle,req.index);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectFloatParameterService(vrep_common::simRosGetObjectFloatParameter::Request &req,vrep_common::simRosGetObjectFloatParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simGetObjectFloatParameter(req.handle,req.parameterID,&res.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectHandleService(vrep_common::simRosGetObjectHandle::Request &req,vrep_common::simRosGetObjectHandle::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.handle=simGetObjectHandle(req.objectName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectIntParameterService(vrep_common::simRosGetObjectIntParameter::Request &req,vrep_common::simRosGetObjectIntParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simGetObjectIntParameter(req.handle,req.parameterID,&res.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}
/*
bool ROS_server::simRosGetObjectOrientationService(vrep_common::simRosGetObjectOrientation::Request &req,vrep_common::simRosGetObjectOrientation::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.eulerAngles.resize(3);
	res.result=simGetObjectOrientation(req.handle,req.relativeToObjectHandle,&res.eulerAngles[0]);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}
*/
/*
bool ROS_server::simRosGetObjectQuaternionService(vrep_common::simRosGetObjectQuaternion::Request &req,vrep_common::simRosGetObjectQuaternion::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.quaternion.resize(4);
	res.result=simGetObjectQuaternion(req.handle,req.relativeToObjectHandle,&res.quaternion[0]);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}
*/
bool ROS_server::simRosGetObjectParentService(vrep_common::simRosGetObjectParent::Request &req,vrep_common::simRosGetObjectParent::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.parentHandle=simGetObjectParent(req.handle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}
/*
bool ROS_server::simRosGetObjectPositionService(vrep_common::simRosGetObjectPosition::Request &req,vrep_common::simRosGetObjectPosition::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.position.resize(3);
	res.result=simGetObjectPosition(req.handle,req.relativeToObjectHandle,&res.position[0]);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}
*/
bool ROS_server::simRosGetObjectPoseService(vrep_common::simRosGetObjectPose::Request &req,vrep_common::simRosGetObjectPose::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float p[3];
	float q[4];
	res.result=simGetObjectPosition(req.handle,req.relativeToObjectHandle,p);
	if (res.result!=-1)
	{
		simGetObjectQuaternion(req.handle,req.relativeToObjectHandle,q);	
		res.pose.header.seq=_simulationFrameID;
		res.pose.header.stamp=ros::Time::now();
		res.pose.pose.position.x=(double)p[0];
		res.pose.pose.position.y=(double)p[1];
		res.pose.pose.position.z=(double)p[2];
		res.pose.pose.orientation.x=(double)q[0];
		res.pose.pose.orientation.y=(double)q[1];
		res.pose.pose.orientation.z=(double)q[2];
		res.pose.pose.orientation.w=(double)q[3];
	}
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectsService(vrep_common::simRosGetObjects::Request &req,vrep_common::simRosGetObjects::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=0;
	res.handles.clear();
	while (true)
	{
		int h=simGetObjects(res.result++,req.objectType);
		if (h==-1)
			break;
		res.handles.push_back(h);
	}
	res.result--;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectSelectionService(vrep_common::simRosGetObjectSelection::Request &req,vrep_common::simRosGetObjectSelection::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int s=simGetObjectSelectionSize();
	res.handles.resize(s);
	if (s>0)
		simGetObjectSelection(&res.handles[0]);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetStringParameterService(vrep_common::simRosGetStringParameter::Request &req,vrep_common::simRosGetStringParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	char* str=simGetStringParameter(req.parameter);
	if (str!=NULL)
	{
		res.parameterValue=str;
		simReleaseBuffer(str);
		res.result=1;
	}
	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetStringSignalService(vrep_common::simRosGetStringSignal::Request &req,vrep_common::simRosGetStringSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int signalLength;
	char* signalValue=simGetStringSignal(req.signalName.c_str(),&signalLength);
	if (signalValue!=NULL)
	{
		res.signalValue.clear();
		for (int i=0;i<signalLength;i++)
			res.signalValue+=signalValue[i];
		simReleaseBuffer(signalValue);
		res.result=1;
	}
	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetUIButtonPropertyService(vrep_common::simRosGetUIButtonProperty::Request &req,vrep_common::simRosGetUIButtonProperty::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.propertyValue=simGetUIButtonProperty(req.uiHandle,req.buttonID);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetUIEventButtonService(vrep_common::simRosGetUIEventButton::Request &req,vrep_common::simRosGetUIEventButton::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.auxiliaryValues.resize(2);
	res.buttonID=simGetUIEventButton(req.uiHandle,&res.auxiliaryValues[0]);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetUIHandleService(vrep_common::simRosGetUIHandle::Request &req,vrep_common::simRosGetUIHandle::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.handle=simGetUIHandle(req.uiName.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetUISliderService(vrep_common::simRosGetUISlider::Request &req,vrep_common::simRosGetUISlider::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.position=simGetUISlider(req.uiHandle,req.buttonID);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetVisionSensorDepthBufferService(vrep_common::simRosGetVisionSensorDepthBuffer::Request &req,vrep_common::simRosGetVisionSensorDepthBuffer::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.resolution.resize(2);
	if (simGetVisionSensorResolution(req.handle,&res.resolution[0])!=-1)
	{
		float* img=simGetVisionSensorDepthBuffer(req.handle);
		if (img!=NULL)
		{
			int bs=res.resolution[0]*res.resolution[1];
			res.buffer.resize(bs);
			for (int i=0;i<bs;i++)
				res.buffer[i]=img[i];
			simReleaseBuffer((simChar*)img);
			res.result=1;
		}
		else
			res.result=-1;
	}
	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetVisionSensorImageService(vrep_common::simRosGetVisionSensorImage::Request &req,vrep_common::simRosGetVisionSensorImage::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int resol[2];
	if (simGetVisionSensorResolution(req.handle,resol)!=-1)
	{
		unsigned char* image_buf=simGetVisionSensorCharImage(req.handle,NULL,NULL);
		int bytesPerPixel;
		if (req.options&1)
			bytesPerPixel=1;
		else
			bytesPerPixel=3;

		if (req.options&1)
			res.image.encoding=sensor_msgs::image_encodings::MONO8;
		else
			res.image.encoding=sensor_msgs::image_encodings::RGB8;

		res.image.height=resol[1];
		res.image.width=resol[0];
		res.image.step=res.image.width*bytesPerPixel; //Set the image stride in bytes
		
		int data_len=res.image.step*res.image.height;
		res.image.data.resize(data_len);
		res.image.is_bigendian=0;

		if (req.options&1)
		{ // B/W
			for(unsigned int i=0;i<res.image.height;i++)
			{
				int msg_idx=(res.image.height-i-1)*res.image.step;
				int buf_idx=i*res.image.step;
				for(unsigned int j=0;j<res.image.step;j++)
					res.image.data[msg_idx+j]=(unsigned char)((image_buf[3*(buf_idx+j)+0]+image_buf[3*(buf_idx+j)+1]+image_buf[3*(buf_idx+j)+2])/3);
			}
		}
		else
		{ // Color
			for(unsigned int i=0;i<res.image.height;i++)
			{
				int msg_idx=(res.image.height-i-1)*res.image.step;
				int buf_idx=i*res.image.step;
				for(unsigned int j=0;j<res.image.step;j++)
					res.image.data[msg_idx+j]=image_buf[buf_idx+j];
			}
		}
		
		simReleaseBuffer((char*)image_buf);

		res.result=1;
	}
	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosLoadModelService(vrep_common::simRosLoadModel::Request &req,vrep_common::simRosLoadModel::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	simRemoveObjectFromSelection(sim_handle_all,-1);
	int initValue=simGetBooleanParameter(sim_boolparam_scene_and_model_load_messages);
	simSetBooleanParameter(sim_boolparam_scene_and_model_load_messages,0);
	res.result=simLoadModel(req.fileName.c_str());
	simSetBooleanParameter(sim_boolparam_scene_and_model_load_messages,initValue);
	res.baseHandle=simGetObjectLastSelection();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosLoadSceneService(vrep_common::simRosLoadScene::Request &req,vrep_common::simRosLoadScene::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int initValue=simGetBooleanParameter(sim_boolparam_scene_and_model_load_messages);
	simSetBooleanParameter(sim_boolparam_scene_and_model_load_messages,0);
	res.result=simLoadScene(req.fileName.c_str());
	simSetBooleanParameter(sim_boolparam_scene_and_model_load_messages,initValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosLoadUIService(vrep_common::simRosLoadUI::Request &req,vrep_common::simRosLoadUI::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int handles[1000];
	res.result=simLoadUI(req.fileName.c_str(),1000,handles);
	if (res.result>0)
	{
		res.uiHandles.resize(res.result);
		for (int i=0;i<res.result;i++)
			res.uiHandles[i]=handles[i];
	}
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosPauseSimulationService(vrep_common::simRosPauseSimulation::Request &req,vrep_common::simRosPauseSimulation::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simPauseSimulation();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosReadCollisionService(vrep_common::simRosReadCollision::Request &req,vrep_common::simRosReadCollision::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.collisionState=simReadCollision(req.handle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosReadDistanceService(vrep_common::simRosReadDistance::Request &req,vrep_common::simRosReadDistance::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simReadDistance(req.handle,&res.distance);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosReadForceSensorService(vrep_common::simRosReadForceSensor::Request &req,vrep_common::simRosReadForceSensor::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float f[3];
	float t[3];
	res.result=simReadForceSensor(req.handle,f,t);
	res.force.x=(double)f[0];
	res.force.y=(double)f[1];
	res.force.z=(double)f[2];
	res.torque.x=(double)t[0];
	res.torque.y=(double)t[1];
	res.torque.z=(double)t[2];
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosReadProximitySensorService(vrep_common::simRosReadProximitySensor::Request &req,vrep_common::simRosReadProximitySensor::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float detectedPoint[4];
	res.detectedPoint.resize(3);
	res.normalVector.resize(3);
	res.result=simReadProximitySensor(req.handle,detectedPoint,&res.detectedObject,&res.normalVector[0]);
	for (int i=0;i<3;i++)
		res.detectedPoint[i]=detectedPoint[i];
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosReadVisionSensorService(vrep_common::simRosReadVisionSensor::Request &req,vrep_common::simRosReadVisionSensor::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float* auxValues;
	int* auxValuesCount;
	res.result=simReadVisionSensor(req.handle,&auxValues,&auxValuesCount);
	if (res.result>=0)
	{
		int packetCnt=auxValuesCount[0];
		res.packetSizes.resize(packetCnt);
		res.packetData.clear();
		int cnt=0;
		for (int i=0;i<packetCnt;i++)
		{
			res.packetSizes[i]=auxValuesCount[1+i];
			for (int j=0;j<res.packetSizes[i];j++)
				res.packetData.push_back(auxValues[cnt++]);
		}
		simReleaseBuffer((char*)auxValues);
		simReleaseBuffer((char*)auxValuesCount);
	}
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosRemoveObjectService(vrep_common::simRosRemoveObject::Request &req,vrep_common::simRosRemoveObject::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simRemoveObject(req.handle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosRemoveModelService(vrep_common::simRosRemoveModel::Request &req,vrep_common::simRosRemoveModel::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simRemoveModel(req.handle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosRemoveUIService(vrep_common::simRosRemoveUI::Request &req,vrep_common::simRosRemoveUI::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simRemoveUI(req.handle);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetArrayParameterService(vrep_common::simRosSetArrayParameter::Request &req,vrep_common::simRosSetArrayParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	if (req.parameterValues.size()>=3)
		res.result=simSetArrayParameter(req.parameter,&req.parameterValues[0]);
	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetBooleanParameterService(vrep_common::simRosSetBooleanParameter::Request &req,vrep_common::simRosSetBooleanParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetBooleanParameter(req.parameter,req.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetFloatingParameterService(vrep_common::simRosSetFloatingParameter::Request &req,vrep_common::simRosSetFloatingParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetFloatingParameter(req.parameter,req.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetFloatSignalService(vrep_common::simRosSetFloatSignal::Request &req,vrep_common::simRosSetFloatSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetFloatSignal(req.signalName.c_str(),req.signalValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetIntegerParameterService(vrep_common::simRosSetIntegerParameter::Request &req,vrep_common::simRosSetIntegerParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetIntegerParameter(req.parameter,req.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetIntegerSignalService(vrep_common::simRosSetIntegerSignal::Request &req,vrep_common::simRosSetIntegerSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetIntegerSignal(req.signalName.c_str(),req.signalValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetJointForceService(vrep_common::simRosSetJointForce::Request &req,vrep_common::simRosSetJointForce::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetJointForce(req.handle,(float)req.forceOrTorque);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetJointPositionService(vrep_common::simRosSetJointPosition::Request &req,vrep_common::simRosSetJointPosition::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetJointPosition(req.handle,(float)req.position);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetJointTargetPositionService(vrep_common::simRosSetJointTargetPosition::Request &req,vrep_common::simRosSetJointTargetPosition::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetJointTargetPosition(req.handle,(float)req.targetPosition);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetJointTargetVelocityService(vrep_common::simRosSetJointTargetVelocity::Request &req,vrep_common::simRosSetJointTargetVelocity::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetJointTargetVelocity(req.handle,(float)req.targetVelocity);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetModelPropertyService(vrep_common::simRosSetModelProperty::Request &req,vrep_common::simRosSetModelProperty::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetModelProperty(req.handle,req.propertyValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectFloatParameterService(vrep_common::simRosSetObjectFloatParameter::Request &req,vrep_common::simRosSetObjectFloatParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetObjectFloatParameter(req.handle,req.parameter,req.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectIntParameterService(vrep_common::simRosSetObjectIntParameter::Request &req,vrep_common::simRosSetObjectIntParameter::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetObjectIntParameter(req.handle,req.parameter,req.parameterValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectPoseService(vrep_common::simRosSetObjectPose::Request &req,vrep_common::simRosSetObjectPose::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float p[3]={(float)req.pose.position.x,(float)req.pose.position.y,(float)req.pose.position.z};
	float q[4]={(float)req.pose.orientation.x,(float)req.pose.orientation.y,(float)req.pose.orientation.z,(float)req.pose.orientation.w};
	res.result=simSetObjectPosition(req.handle,req.relativeToObjectHandle,p);
	if (res.result!=-1)
		simSetObjectQuaternion(req.handle,req.relativeToObjectHandle,q);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectQuaternionService(vrep_common::simRosSetObjectQuaternion::Request &req,vrep_common::simRosSetObjectQuaternion::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float q[4]={(float)req.quaternion.x,(float)req.quaternion.y,(float)req.quaternion.z,(float)req.quaternion.w};
	res.result=simSetObjectQuaternion(req.handle,req.relativeToObjectHandle,q);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectParentService(vrep_common::simRosSetObjectParent::Request &req,vrep_common::simRosSetObjectParent::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetObjectParent(req.handle,req.parentHandle,req.keepInPlace);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectPositionService(vrep_common::simRosSetObjectPosition::Request &req,vrep_common::simRosSetObjectPosition::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float p[3]={(float)req.position.x,(float)req.position.y,(float)req.position.z};
	res.result=simSetObjectPosition(req.handle,req.relativeToObjectHandle,p);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetObjectSelectionService(vrep_common::simRosSetObjectSelection::Request &req,vrep_common::simRosSetObjectSelection::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	simRemoveObjectFromSelection(sim_handle_all,0);
	int cnt=req.handles.size();
	for (int i=0;i<cnt;i++)
		simAddObjectToSelection(sim_handle_single,req.handles[i]);
	res.result=simGetObjectSelectionSize();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetInfoService(vrep_common::simRosGetInfo::Request &req,vrep_common::simRosGetInfo::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int simState=simGetSimulationState();
	res.simulatorState=0;
	if (simState!=sim_simulation_stopped)
	{
		res.simulatorState|=1; // simulation is not stopped
		if (simState==sim_simulation_paused)
			res.simulatorState|=2; // simulation is paused
	}
	if (simGetRealTimeSimulation()>0)
		res.simulatorState|=4;
	int editModeType;
	simGetIntegerParameter(sim_intparam_edit_mode_type,&editModeType);
	res.simulatorState|=(editModeType<<3);
	res.simulationTime=simGetSimulationTime();
	res.timeStep=simGetSimulationTimeStep();
	res.headerInfo.seq=_simulationFrameID;
	res.headerInfo.stamp=ros::Time::now();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetSphericalJointMatrixService(vrep_common::simRosSetSphericalJointMatrix::Request &req,vrep_common::simRosSetSphericalJointMatrix::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetObjectFloatParameter(req.handle,2013,(float)req.quaternion.x);
	if (res.result>0)
	{
		simSetObjectFloatParameter(req.handle,2014,(float)req.quaternion.y);
		simSetObjectFloatParameter(req.handle,2015,(float)req.quaternion.z);
		simSetObjectFloatParameter(req.handle,2016,(float)req.quaternion.w);
	}
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetStringSignalService(vrep_common::simRosSetStringSignal::Request &req,vrep_common::simRosSetStringSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetStringSignal(req.signalName.c_str(),req.signalValue.c_str(),req.signalValue.length());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosAppendStringSignalService(vrep_common::simRosAppendStringSignal::Request &req,vrep_common::simRosAppendStringSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	std::string theNewString;
	int stringLength;
	char* stringSignal=simGetStringSignal(req.signalName.c_str(),&stringLength);
	if (stringSignal!=NULL)
	{
		theNewString=std::string(stringSignal,stringLength);
		simReleaseBuffer(stringSignal);
	}
	theNewString+=std::string(req.signalValue.c_str(),req.signalValue.length());
	res.result=simSetStringSignal(req.signalName.c_str(),theNewString.c_str(),theNewString.length());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetUIButtonLabelService(vrep_common::simRosSetUIButtonLabel::Request &req,vrep_common::simRosSetUIButtonLabel::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetUIButtonLabel(req.uiHandle,req.buttonID,req.upStateLabel.c_str(),req.downStateLabel.c_str());
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetUIButtonPropertyService(vrep_common::simRosSetUIButtonProperty::Request &req,vrep_common::simRosSetUIButtonProperty::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetUIButtonProperty(req.uiHandle,req.buttonID,req.propertyValue);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetUISliderService(vrep_common::simRosSetUISlider::Request &req,vrep_common::simRosSetUISlider::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simSetUISlider(req.uiHandle,req.buttonID,req.position);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSetVisionSensorImageService(vrep_common::simRosSetVisionSensorImage::Request &req,vrep_common::simRosSetVisionSensorImage::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();

	int resol[2];
	int result=simGetVisionSensorResolution(req.handle,resol);
	res.result=-1;
	if (result!=-1)
	{
		if (req.image.encoding==sensor_msgs::image_encodings::RGB8)
		{
			if ((req.image.step==req.image.width*3)&&(int(req.image.height)==resol[1])&&(int(req.image.width)==resol[0]))
			{
				float* image_buff=new float[3*resol[0]*resol[1]];
				for(unsigned int i=0;i<req.image.height;i++)
				{
					int msg_idx=(req.image.height-i-1)*req.image.step;
					int buf_idx=i*req.image.step;
					for(unsigned int j=0;j<req.image.step;j++)
						image_buff[buf_idx+j]=float(req.image.data[msg_idx+j])/255.0f;
				}
				simSetVisionSensorImage(req.handle,image_buff);
				delete[] image_buff;
				res.result=1;
			}
		}
		if (req.image.encoding==sensor_msgs::image_encodings::MONO8)
		{
			if ((req.image.step==req.image.width)&&(int(req.image.height)==resol[1])&&(int(req.image.width)==resol[0]))
			{
				float* image_buff=new float[3*resol[0]*resol[1]];
				for(unsigned int i=0;i<req.image.height;i++)
				{
					int msg_idx=(req.image.height-i-1)*req.image.step;
					int buf_idx=i*req.image.step;
					for(unsigned int j=0;j<req.image.step;j++)
					{
						image_buff[3*(buf_idx+j)+0]=float(req.image.data[msg_idx+j])/255.0f;
						image_buff[3*(buf_idx+j)+1]=float(req.image.data[msg_idx+j])/255.0f;
						image_buff[3*(buf_idx+j)+2]=float(req.image.data[msg_idx+j])/255.0f;
					}
				}
				simSetVisionSensorImage(req.handle,image_buff);
				delete[] image_buff;
				res.result=1;
			}
		}
	}
	else
		res.result=-1;

	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosStartSimulationService(vrep_common::simRosStartSimulation::Request &req,vrep_common::simRosStartSimulation::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simStartSimulation();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosStopSimulationService(vrep_common::simRosStopSimulation::Request &req,vrep_common::simRosStopSimulation::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	res.result=simStopSimulation();
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosSynchronousService(vrep_common::simRosSynchronous::Request &req,vrep_common::simRosSynchronous::Response &res)
{
	if (req.enable)
	{
		_waitTriggerEnable=true;
		_waitForTrigger=true;
	}
	else
		_waitTriggerEnable=false;
	res.result=1;
	return true;
}

bool ROS_server::simRosSynchronousTriggerService(vrep_common::simRosSynchronousTrigger::Request &req,vrep_common::simRosSynchronousTrigger::Response &res)
{
	if (_waitTriggerEnable)
	{
		_waitForTrigger=false;
		res.result=1;
	}
	else
		res.result=-1;
	return true;
}

bool ROS_server::simRosTransferFileService(vrep_common::simRosTransferFile::Request &req,vrep_common::simRosTransferFile::Response &res)
{
	res.result=-1;
	FILE* file=fopen(req.fileName.c_str(),"wb");
	if (file!=NULL)
	{
		res.result=1;
		fwrite(&req.data[0],1,req.data.size(),file);
		fclose(file);
	}
	return true;
}

bool ROS_server::simRosEnablePublisherService(vrep_common::simRosEnablePublisher::Request &req,vrep_common::simRosEnablePublisher::Response &res)
{
	int simState=simGetSimulationState();
	if (simState&sim_simulation_advancing)
		res.effectiveTopicName=addPublisher(req.topicName.c_str(),req.queueSize,req.streamCmd,req.auxInt1,req.auxInt2,req.auxString.c_str(),0);
	else
		res.effectiveTopicName=""; // Error, simulation is not running!
	return true;
}

bool ROS_server::simRosDisablePublisherService(vrep_common::simRosDisablePublisher::Request &req,vrep_common::simRosDisablePublisher::Response &res)
{
	res.referenceCounter=removePublisher(req.topicName.c_str(),false);
	return true;
}


bool ROS_server::simRosEnableSubscriberService(vrep_common::simRosEnableSubscriber::Request &req,vrep_common::simRosEnableSubscriber::Response &res)
{
	int simState=simGetSimulationState();
	if (simState&sim_simulation_advancing)
		res.subscriberID=addSubscriber(req.topicName.c_str(),req.queueSize,req.streamCmd,req.auxInt1,req.auxInt2,req.auxString.c_str(),-1,-1);
	else
		res.subscriberID=-1; // Error, simulation is not running!
	return true;
}

bool ROS_server::simRosDisableSubscriberService(vrep_common::simRosDisableSubscriber::Request &req,vrep_common::simRosDisableSubscriber::Response &res)
{
	res.result=removeSubscriber(req.subscriberID);
	return true;
}

bool ROS_server::simRosSetJointStateService(vrep_common::simRosSetJointState::Request &req,vrep_common::simRosSetJointState::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();

	int setOkCnt=-1; // error
	if ( (req.handles.size()!=0)&&(req.handles.size()==req.setModes.size())&&(req.handles.size()==req.values.size()) )
	{
		setOkCnt=0;
		for (unsigned int i=0;i<req.handles.size();i++)
		{
			int handle=req.handles[i];
			unsigned char setMode=req.setModes[i];
			float val=req.values[i];
			if (setMode==0)
			{
				if (simSetJointPosition(handle,val)!=-1)
					setOkCnt++;
			}
			if (setMode==1)
			{
				if (simSetJointTargetPosition(handle,val)!=-1)
					setOkCnt++;
			}
			if (setMode==2)
			{
				if (simSetJointTargetVelocity(handle,val)!=-1)
					setOkCnt++;
			}
			if (setMode==3)
			{
				if (simSetJointForce(handle,val)!=-1)
					setOkCnt++;
			}
		}
	}
	res.result=setOkCnt;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosCreateDummyService(vrep_common::simRosCreateDummy::Request &req,vrep_common::simRosCreateDummy::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	float* co=NULL;
	float col[12];
	if (req.colors.size()==12)
	{
		for (int i=0;i<12;i++)
			col[i]=req.colors[i];
		co=col;
	}
	res.dummyHandle=simCreateDummy(req.size,co);
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetAndClearStringSignalService(vrep_common::simRosGetAndClearStringSignal::Request &req,vrep_common::simRosGetAndClearStringSignal::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();
	int signalLength;
	char* signalValue=simGetStringSignal(req.signalName.c_str(),&signalLength);
	if (signalValue!=NULL)
	{
		res.signalValue.clear();
		for (int i=0;i<signalLength;i++)
			res.signalValue+=signalValue[i];
		simReleaseBuffer(signalValue);
		simClearStringSignal(req.signalName.c_str());
		res.result=1;
	}
	else
		res.result=-1;
	_handleServiceErrors_end(errorModeSaved);
	return true;
}

bool ROS_server::simRosGetObjectGroupDataService(vrep_common::simRosGetObjectGroupData::Request &req,vrep_common::simRosGetObjectGroupData::Response &res)
{
	int errorModeSaved=_handleServiceErrors_start();

	std::vector<int> handles;
	std::vector<int> intData;
	std::vector<float> floatData;
	std::vector<std::string> stringData;
	if (getObjectGroupData(req.objectType,req.dataType,handles,intData,floatData,stringData))
	{
		for (int i=0;i<int(handles.size());i++)
			res.handles.push_back(handles[i]);
		for (int i=0;i<int(intData.size());i++)
			res.intData.push_back(intData[i]);
		for (int i=0;i<int(floatData.size());i++)
			res.floatData.push_back(floatData[i]);
		for (int i=0;i<int(stringData.size());i++)
			res.strings.push_back(stringData[i]);
	}

	_handleServiceErrors_end(errorModeSaved);
	return true;
}
