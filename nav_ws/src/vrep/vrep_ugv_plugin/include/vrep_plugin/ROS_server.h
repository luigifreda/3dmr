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

#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include "vrepSubscriber.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <nifti_robot_driver_msgs/Tracks.h>
#include <nifti_robot_driver_msgs/TracksStamped.h>
#include "vrep_common/VisionSensorDepthBuff.h"
#include "vrep_common/ForceSensorData.h"
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VisionSensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/ObjectGroupData.h"

// API services:
#include "vrep_common/simRosAddStatusbarMessage.h"
#include "vrep_common/simRosAuxiliaryConsoleClose.h"
#include "vrep_common/simRosAuxiliaryConsoleOpen.h"
#include "vrep_common/simRosAuxiliaryConsolePrint.h"
#include "vrep_common/simRosAuxiliaryConsoleShow.h"
#include "vrep_common/simRosBreakForceSensor.h"
#include "vrep_common/simRosClearFloatSignal.h"
#include "vrep_common/simRosClearIntegerSignal.h"
#include "vrep_common/simRosClearStringSignal.h"
#include "vrep_common/simRosCloseScene.h"
#include "vrep_common/simRosCopyPasteObjects.h"
#include "vrep_common/simRosDisplayDialog.h"
#include "vrep_common/simRosEndDialog.h"
#include "vrep_common/simRosEraseFile.h"
#include "vrep_common/simRosGetArrayParameter.h"
#include "vrep_common/simRosGetBooleanParameter.h"
#include "vrep_common/simRosGetCollisionHandle.h"
#include "vrep_common/simRosGetDialogInput.h"
#include "vrep_common/simRosGetDialogResult.h"
#include "vrep_common/simRosGetDistanceHandle.h"
#include "vrep_common/simRosGetFloatingParameter.h"
#include "vrep_common/simRosGetFloatSignal.h"
#include "vrep_common/simRosGetIntegerParameter.h"
#include "vrep_common/simRosGetIntegerSignal.h"
#include "vrep_common/simRosGetJointMatrix.h"
#include "vrep_common/simRosGetJointState.h"
#include "vrep_common/simRosGetLastErrors.h"
#include "vrep_common/simRosGetModelProperty.h"
#include "vrep_common/simRosGetObjectChild.h"
#include "vrep_common/simRosGetObjectFloatParameter.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosGetObjectIntParameter.h"
//#include "vrep_common/simRosGetObjectOrientation.h"
#include "vrep_common/simRosGetObjectParent.h"
//#include "vrep_common/simRosGetObjectPosition.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosGetObjects.h"
#include "vrep_common/simRosGetObjectSelection.h"
#include "vrep_common/simRosGetStringParameter.h"
#include "vrep_common/simRosGetStringSignal.h"
#include "vrep_common/simRosGetUIButtonProperty.h"
#include "vrep_common/simRosGetUIEventButton.h"
#include "vrep_common/simRosGetUIHandle.h"
#include "vrep_common/simRosGetUISlider.h"
#include "vrep_common/simRosGetVisionSensorDepthBuffer.h"
#include "vrep_common/simRosGetVisionSensorImage.h"
#include "vrep_common/simRosLoadModel.h"
#include "vrep_common/simRosLoadScene.h"
#include "vrep_common/simRosLoadUI.h"
#include "vrep_common/simRosPauseSimulation.h"
#include "vrep_common/simRosReadCollision.h"
#include "vrep_common/simRosReadDistance.h"
#include "vrep_common/simRosReadForceSensor.h"
#include "vrep_common/simRosReadProximitySensor.h"
#include "vrep_common/simRosReadVisionSensor.h"
#include "vrep_common/simRosRemoveObject.h"
#include "vrep_common/simRosRemoveModel.h"
#include "vrep_common/simRosRemoveUI.h"
#include "vrep_common/simRosSetArrayParameter.h"
#include "vrep_common/simRosSetBooleanParameter.h"
#include "vrep_common/simRosSetFloatingParameter.h"
#include "vrep_common/simRosSetFloatSignal.h"
#include "vrep_common/simRosSetIntegerParameter.h"
#include "vrep_common/simRosSetIntegerSignal.h"
#include "vrep_common/simRosSetJointForce.h"
#include "vrep_common/simRosSetJointPosition.h"
#include "vrep_common/simRosSetJointTargetPosition.h"
#include "vrep_common/simRosSetJointTargetVelocity.h"
#include "vrep_common/simRosSetModelProperty.h"
#include "vrep_common/simRosSetObjectFloatParameter.h"
#include "vrep_common/simRosSetObjectIntParameter.h"
#include "vrep_common/simRosSetObjectPose.h"
#include "vrep_common/simRosSetObjectParent.h"
#include "vrep_common/simRosSetObjectPosition.h"
#include "vrep_common/simRosSetObjectSelection.h"
#include "vrep_common/simRosGetInfo.h"
#include "vrep_common/simRosSetSphericalJointMatrix.h"
#include "vrep_common/simRosSetStringSignal.h"
#include "vrep_common/simRosAppendStringSignal.h"
#include "vrep_common/simRosSetUIButtonLabel.h"
#include "vrep_common/simRosSetUIButtonProperty.h"
#include "vrep_common/simRosSetUISlider.h"
#include "vrep_common/simRosSetVisionSensorImage.h"
#include "vrep_common/simRosStartSimulation.h"
#include "vrep_common/simRosStopSimulation.h"
#include "vrep_common/simRosSynchronous.h"
#include "vrep_common/simRosSynchronousTrigger.h"
#include "vrep_common/simRosTransferFile.h"
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosDisablePublisher.h"
//#include "vrep_common/simRosGetObjectQuaternion.h"
#include "vrep_common/simRosSetObjectQuaternion.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosDisableSubscriber.h"
#include "vrep_common/simRosSetJointState.h"
#include "vrep_common/simRosCreateDummy.h"
#include "vrep_common/simRosGetAndClearStringSignal.h"
#include "vrep_common/simRosGetObjectGroupData.h"

#include "tf_transform_broadcaster_custom.h"

class SSpecificPublisherData 
{
    public:
        SSpecificPublisherData() {}
        virtual ~SSpecificPublisherData();
};

struct SPublisherData
{
	int cmdID;
	int auxInt1;
	int auxInt2;
	std::string auxStr;
	int publishCnt; // -1=publisher is asleep
	std::string topicName;
	ros::Publisher generalPublisher;
	image_transport::Publisher imagePublisher;
	SSpecificPublisherData* specificPublisherData;
	int dependencyCnt;
};

class ROS_server
{
	public:
		static bool initialize();
		static void shutDown();

		static void instancePass();
		static void simulationAboutToStart();
		static bool mainScriptAboutToBeCalled();
		static void simulationEnded();

		static std::string addPublisher(const char* topicName,int queueSize,int streamCmd,int auxInt1,int auxInt2,const char* auxString,int publishCnt);
		static int removePublisher(const char* topicName,bool ignoreReferenceCounter);
		static int wakePublisher(const char* topicName,int publishCnt);

		static int addSubscriber(const char* topicName,int queueSize,int streamCmd,int auxInt1,int auxInt2,const char* auxString,int callbackTag_before,int callbackTag_after);
		static bool removeSubscriber(int subscriberID);

	private:
		ROS_server() {}; 
		
		static ros::NodeHandle* node;
                static tf::TransformBroadcaster* tf_broadcaster;
                static tf::TransformBroadcasterCustom* tf_old_broadcaster;

		static image_transport::ImageTransport* images_streamer;
		static int imgStreamerCnt;

		static void enableAPIServices();
		static void disableAPIServices();
		static void spinOnce();

		static bool getObjectGroupData(int objectType,int dataType,std::vector<int>& handles,std::vector<int>& intData,std::vector<float>& floatData,std::vector<std::string>& stringData);

		static int getPublisherIndexFromCmd(int streamCmd,int auxInt1,int auxInt2,const char* auxString);
		static int getPublisherIndexFromTopicName(const char* topicName);
		static void removeAllPublishers();
		static bool launchPublisher(SPublisherData& pub,int queueSize);
		static void shutDownPublisher(SPublisherData& pub);
		static void streamAllData();

		static bool streamVisionSensorImage(SPublisherData& pub, const ros::Time & now,bool& publishedSomething);
		static bool streamLaserCloud(SPublisherData& pub, const ros::Time & now,bool& publishedSomething);
		static bool streamDepthSensorCloud(SPublisherData& pub, const ros::Time & now,bool& publishedSomething);

		static std::vector<SPublisherData> publishers;
		static ros::Publisher infoPublisher; // special publisher that is active also when simulation is not running!

		static void removeAllSubscribers();
		static std::vector<CSubscriberData*> subscribers;
		static int lastSubscriberID;

		static int _handleServiceErrors_start();
		static void _handleServiceErrors_end(int errorReportMode);
		static std::vector<std::string> _last50Errors;

		static bool _waitTriggerEnable;
		static bool _waitForTrigger;

		static int _simulationFrameID;
		

		//===================================================================================================
		//================================== API services ===================================================
		//===================================================================================================

		static ros::ServiceServer simRosAddStatusbarMessageServer;
		static bool simRosAddStatusbarMessageService(vrep_common::simRosAddStatusbarMessage::Request &req,vrep_common::simRosAddStatusbarMessage::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsoleCloseServer;
		static bool simRosAuxiliaryConsoleCloseService(vrep_common::simRosAuxiliaryConsoleClose::Request &req,vrep_common::simRosAuxiliaryConsoleClose::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsoleOpenServer;
		static bool simRosAuxiliaryConsoleOpenService(vrep_common::simRosAuxiliaryConsoleOpen::Request &req,vrep_common::simRosAuxiliaryConsoleOpen::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsolePrintServer;
		static bool simRosAuxiliaryConsolePrintService(vrep_common::simRosAuxiliaryConsolePrint::Request &req,vrep_common::simRosAuxiliaryConsolePrint::Response &res);

		static ros::ServiceServer simRosAuxiliaryConsoleShowServer;
		static bool simRosAuxiliaryConsoleShowService(vrep_common::simRosAuxiliaryConsoleShow::Request &req,vrep_common::simRosAuxiliaryConsoleShow::Response &res);

		static ros::ServiceServer simRosBreakForceSensorServer;
		static bool simRosBreakForceSensorService(vrep_common::simRosBreakForceSensor::Request &req,vrep_common::simRosBreakForceSensor::Response &res);

		static ros::ServiceServer simRosClearFloatSignalServer;
		static bool simRosClearFloatSignalService(vrep_common::simRosClearFloatSignal::Request &req,vrep_common::simRosClearFloatSignal::Response &res);

		static ros::ServiceServer simRosClearIntegerSignalServer;
		static bool simRosClearIntegerSignalService(vrep_common::simRosClearIntegerSignal::Request &req,vrep_common::simRosClearIntegerSignal::Response &res);

		static ros::ServiceServer simRosClearStringSignalServer;
		static bool simRosClearStringSignalService(vrep_common::simRosClearStringSignal::Request &req,vrep_common::simRosClearStringSignal::Response &res);

		static ros::ServiceServer simRosCloseSceneServer;
		static bool simRosCloseSceneService(vrep_common::simRosCloseScene::Request &req,vrep_common::simRosCloseScene::Response &res);

		static ros::ServiceServer simRosCopyPasteObjectsServer;
		static bool simRosCopyPasteObjectsService(vrep_common::simRosCopyPasteObjects::Request &req,vrep_common::simRosCopyPasteObjects::Response &res);

		static ros::ServiceServer simRosDisplayDialogServer;
		static bool simRosDisplayDialogService(vrep_common::simRosDisplayDialog::Request &req,vrep_common::simRosDisplayDialog::Response &res);

		static ros::ServiceServer simRosEndDialogServer;
		static bool simRosEndDialogService(vrep_common::simRosEndDialog::Request &req,vrep_common::simRosEndDialog::Response &res);

		static ros::ServiceServer simRosEraseFileServer;
		static bool simRosEraseFileService(vrep_common::simRosEraseFile::Request &req,vrep_common::simRosEraseFile::Response &res);

		static ros::ServiceServer simRosGetArrayParameterServer;
		static bool simRosGetArrayParameterService(vrep_common::simRosGetArrayParameter::Request &req,vrep_common::simRosGetArrayParameter::Response &res);

		static ros::ServiceServer simRosGetBooleanParameterServer;
		static bool simRosGetBooleanParameterService(vrep_common::simRosGetBooleanParameter::Request &req,vrep_common::simRosGetBooleanParameter::Response &res);

		static ros::ServiceServer simRosGetCollisionHandleServer;
		static bool simRosGetCollisionHandleService(vrep_common::simRosGetCollisionHandle::Request &req,vrep_common::simRosGetCollisionHandle::Response &res);

		static ros::ServiceServer simRosGetDialogInputServer;
		static bool simRosGetDialogInputService(vrep_common::simRosGetDialogInput::Request &req,vrep_common::simRosGetDialogInput::Response &res);

		static ros::ServiceServer simRosGetDialogResultServer;
		static bool simRosGetDialogResultService(vrep_common::simRosGetDialogResult::Request &req,vrep_common::simRosGetDialogResult::Response &res);

		static ros::ServiceServer simRosGetDistanceHandleServer;
		static bool simRosGetDistanceHandleService(vrep_common::simRosGetDistanceHandle::Request &req,vrep_common::simRosGetDistanceHandle::Response &res);

		static ros::ServiceServer simRosGetFloatingParameterServer;
		static bool simRosGetFloatingParameterService(vrep_common::simRosGetFloatingParameter::Request &req,vrep_common::simRosGetFloatingParameter::Response &res);

		static ros::ServiceServer simRosGetFloatSignalServer;
		static bool simRosGetFloatSignalService(vrep_common::simRosGetFloatSignal::Request &req,vrep_common::simRosGetFloatSignal::Response &res);

		static ros::ServiceServer simRosGetIntegerParameterServer;
		static bool simRosGetIntegerParameterService(vrep_common::simRosGetIntegerParameter::Request &req,vrep_common::simRosGetIntegerParameter::Response &res);

		static ros::ServiceServer simRosGetIntegerSignalServer;
		static bool simRosGetIntegerSignalService(vrep_common::simRosGetIntegerSignal::Request &req,vrep_common::simRosGetIntegerSignal::Response &res);

		static ros::ServiceServer simRosGetJointMatrixServer;
		static bool simRosGetJointMatrixService(vrep_common::simRosGetJointMatrix::Request &req,vrep_common::simRosGetJointMatrix::Response &res);

		static ros::ServiceServer simRosGetJointStateServer;
		static bool simRosGetJointStateService(vrep_common::simRosGetJointState::Request &req,vrep_common::simRosGetJointState::Response &res);

		static ros::ServiceServer simRosGetLastErrorsServer;
		static bool simRosGetLastErrorsService(vrep_common::simRosGetLastErrors::Request &req,vrep_common::simRosGetLastErrors::Response &res);

		static ros::ServiceServer simRosGetModelPropertyServer;
		static bool simRosGetModelPropertyService(vrep_common::simRosGetModelProperty::Request &req,vrep_common::simRosGetModelProperty::Response &res);

		static ros::ServiceServer simRosGetObjectChildServer;
		static bool simRosGetObjectChildService(vrep_common::simRosGetObjectChild::Request &req,vrep_common::simRosGetObjectChild::Response &res);

		static ros::ServiceServer simRosGetObjectFloatParameterServer;
		static bool simRosGetObjectFloatParameterService(vrep_common::simRosGetObjectFloatParameter::Request &req,vrep_common::simRosGetObjectFloatParameter::Response &res);

		static ros::ServiceServer simRosGetObjectHandleServer;
		static bool simRosGetObjectHandleService(vrep_common::simRosGetObjectHandle::Request &req,vrep_common::simRosGetObjectHandle::Response &res);

		static ros::ServiceServer simRosGetObjectIntParameterServer;
		static bool simRosGetObjectIntParameterService(vrep_common::simRosGetObjectIntParameter::Request &req,vrep_common::simRosGetObjectIntParameter::Response &res);

//		static ros::ServiceServer simRosGetObjectOrientationServer;
//		static bool simRosGetObjectOrientationService(vrep_common::simRosGetObjectOrientation::Request &req,vrep_common::simRosGetObjectOrientation::Response &res);

		static ros::ServiceServer simRosGetObjectParentServer;
		static bool simRosGetObjectParentService(vrep_common::simRosGetObjectParent::Request &req,vrep_common::simRosGetObjectParent::Response &res);

//		static ros::ServiceServer simRosGetObjectPositionServer;
//		static bool simRosGetObjectPositionService(vrep_common::simRosGetObjectPosition::Request &req,vrep_common::simRosGetObjectPosition::Response &res);

		static ros::ServiceServer simRosGetObjectPoseServer;
		static bool simRosGetObjectPoseService(vrep_common::simRosGetObjectPose::Request &req,vrep_common::simRosGetObjectPose::Response &res);

		static ros::ServiceServer simRosGetObjectsServer;
		static bool simRosGetObjectsService(vrep_common::simRosGetObjects::Request &req,vrep_common::simRosGetObjects::Response &res);

		static ros::ServiceServer simRosGetObjectSelectionServer;
		static bool simRosGetObjectSelectionService(vrep_common::simRosGetObjectSelection::Request &req,vrep_common::simRosGetObjectSelection::Response &res);

		static ros::ServiceServer simRosGetStringParameterServer;
		static bool simRosGetStringParameterService(vrep_common::simRosGetStringParameter::Request &req,vrep_common::simRosGetStringParameter::Response &res);

		static ros::ServiceServer simRosGetStringSignalServer;
		static bool simRosGetStringSignalService(vrep_common::simRosGetStringSignal::Request &req,vrep_common::simRosGetStringSignal::Response &res);

		static ros::ServiceServer simRosGetUIButtonPropertyServer;
		static bool simRosGetUIButtonPropertyService(vrep_common::simRosGetUIButtonProperty::Request &req,vrep_common::simRosGetUIButtonProperty::Response &res);

		static ros::ServiceServer simRosGetUIEventButtonServer;
		static bool simRosGetUIEventButtonService(vrep_common::simRosGetUIEventButton::Request &req,vrep_common::simRosGetUIEventButton::Response &res);

		static ros::ServiceServer simRosGetUIHandleServer;
		static bool simRosGetUIHandleService(vrep_common::simRosGetUIHandle::Request &req,vrep_common::simRosGetUIHandle::Response &res);

		static ros::ServiceServer simRosGetUISliderServer;
		static bool simRosGetUISliderService(vrep_common::simRosGetUISlider::Request &req,vrep_common::simRosGetUISlider::Response &res);

		static ros::ServiceServer simRosGetVisionSensorDepthBufferServer;
		static bool simRosGetVisionSensorDepthBufferService(vrep_common::simRosGetVisionSensorDepthBuffer::Request &req,vrep_common::simRosGetVisionSensorDepthBuffer::Response &res);

		static ros::ServiceServer simRosGetVisionSensorImageServer;
		static bool simRosGetVisionSensorImageService(vrep_common::simRosGetVisionSensorImage::Request &req,vrep_common::simRosGetVisionSensorImage::Response &res);

		static ros::ServiceServer simRosLoadModelServer;
		static bool simRosLoadModelService(vrep_common::simRosLoadModel::Request &req,vrep_common::simRosLoadModel::Response &res);

		static ros::ServiceServer simRosLoadSceneServer;
		static bool simRosLoadSceneService(vrep_common::simRosLoadScene::Request &req,vrep_common::simRosLoadScene::Response &res);

		static ros::ServiceServer simRosLoadUIServer;
		static bool simRosLoadUIService(vrep_common::simRosLoadUI::Request &req,vrep_common::simRosLoadUI::Response &res);

		static ros::ServiceServer simRosPauseSimulationServer;
		static bool simRosPauseSimulationService(vrep_common::simRosPauseSimulation::Request &req,vrep_common::simRosPauseSimulation::Response &res);

		static ros::ServiceServer simRosReadCollisionServer;
		static bool simRosReadCollisionService(vrep_common::simRosReadCollision::Request &req,vrep_common::simRosReadCollision::Response &res);

		static ros::ServiceServer simRosReadDistanceServer;
		static bool simRosReadDistanceService(vrep_common::simRosReadDistance::Request &req,vrep_common::simRosReadDistance::Response &res);

		static ros::ServiceServer simRosReadForceSensorServer;
		static bool simRosReadForceSensorService(vrep_common::simRosReadForceSensor::Request &req,vrep_common::simRosReadForceSensor::Response &res);

		static ros::ServiceServer simRosReadProximitySensorServer;
		static bool simRosReadProximitySensorService(vrep_common::simRosReadProximitySensor::Request &req,vrep_common::simRosReadProximitySensor::Response &res);

		static ros::ServiceServer simRosReadVisionSensorServer;
		static bool simRosReadVisionSensorService(vrep_common::simRosReadVisionSensor::Request &req,vrep_common::simRosReadVisionSensor::Response &res);

		static ros::ServiceServer simRosRemoveObjectServer;
		static bool simRosRemoveObjectService(vrep_common::simRosRemoveObject::Request &req,vrep_common::simRosRemoveObject::Response &res);

		static ros::ServiceServer simRosRemoveModelServer;
		static bool simRosRemoveModelService(vrep_common::simRosRemoveModel::Request &req,vrep_common::simRosRemoveModel::Response &res);

		static ros::ServiceServer simRosRemoveUIServer;
		static bool simRosRemoveUIService(vrep_common::simRosRemoveUI::Request &req,vrep_common::simRosRemoveUI::Response &res);

		static ros::ServiceServer simRosSetArrayParameterServer;
		static bool simRosSetArrayParameterService(vrep_common::simRosSetArrayParameter::Request &req,vrep_common::simRosSetArrayParameter::Response &res);

		static ros::ServiceServer simRosSetBooleanParameterServer;
		static bool simRosSetBooleanParameterService(vrep_common::simRosSetBooleanParameter::Request &req,vrep_common::simRosSetBooleanParameter::Response &res);

		static ros::ServiceServer simRosSetFloatingParameterServer;
		static bool simRosSetFloatingParameterService(vrep_common::simRosSetFloatingParameter::Request &req,vrep_common::simRosSetFloatingParameter::Response &res);

		static ros::ServiceServer simRosSetFloatSignalServer;
		static bool simRosSetFloatSignalService(vrep_common::simRosSetFloatSignal::Request &req,vrep_common::simRosSetFloatSignal::Response &res);

		static ros::ServiceServer simRosSetIntegerParameterServer;
		static bool simRosSetIntegerParameterService(vrep_common::simRosSetIntegerParameter::Request &req,vrep_common::simRosSetIntegerParameter::Response &res);

		static ros::ServiceServer simRosSetIntegerSignalServer;
		static bool simRosSetIntegerSignalService(vrep_common::simRosSetIntegerSignal::Request &req,vrep_common::simRosSetIntegerSignal::Response &res);

		static ros::ServiceServer simRosSetJointForceServer;
		static bool simRosSetJointForceService(vrep_common::simRosSetJointForce::Request &req,vrep_common::simRosSetJointForce::Response &res);

		static ros::ServiceServer simRosSetJointPositionServer;
		static bool simRosSetJointPositionService(vrep_common::simRosSetJointPosition::Request &req,vrep_common::simRosSetJointPosition::Response &res);

		static ros::ServiceServer simRosSetJointTargetPositionServer;
		static bool simRosSetJointTargetPositionService(vrep_common::simRosSetJointTargetPosition::Request &req,vrep_common::simRosSetJointTargetPosition::Response &res);

		static ros::ServiceServer simRosSetJointTargetVelocityServer;
		static bool simRosSetJointTargetVelocityService(vrep_common::simRosSetJointTargetVelocity::Request &req,vrep_common::simRosSetJointTargetVelocity::Response &res);

		static ros::ServiceServer simRosSetModelPropertyServer;
		static bool simRosSetModelPropertyService(vrep_common::simRosSetModelProperty::Request &req,vrep_common::simRosSetModelProperty::Response &res);

		static ros::ServiceServer simRosSetObjectFloatParameterServer;
		static bool simRosSetObjectFloatParameterService(vrep_common::simRosSetObjectFloatParameter::Request &req,vrep_common::simRosSetObjectFloatParameter::Response &res);

		static ros::ServiceServer simRosSetObjectIntParameterServer;
		static bool simRosSetObjectIntParameterService(vrep_common::simRosSetObjectIntParameter::Request &req,vrep_common::simRosSetObjectIntParameter::Response &res);

		static ros::ServiceServer simRosSetObjectPoseServer;
		static bool simRosSetObjectPoseService(vrep_common::simRosSetObjectPose::Request &req,vrep_common::simRosSetObjectPose::Response &res);

		static ros::ServiceServer simRosSetObjectParentServer;
		static bool simRosSetObjectParentService(vrep_common::simRosSetObjectParent::Request &req,vrep_common::simRosSetObjectParent::Response &res);

		static ros::ServiceServer simRosSetObjectPositionServer;
		static bool simRosSetObjectPositionService(vrep_common::simRosSetObjectPosition::Request &req,vrep_common::simRosSetObjectPosition::Response &res);

		static ros::ServiceServer simRosSetObjectSelectionServer;
		static bool simRosSetObjectSelectionService(vrep_common::simRosSetObjectSelection::Request &req,vrep_common::simRosSetObjectSelection::Response &res);

		static ros::ServiceServer simRosGetInfoServer;
		static bool simRosGetInfoService(vrep_common::simRosGetInfo::Request &req,vrep_common::simRosGetInfo::Response &res);

		static ros::ServiceServer simRosSetSphericalJointMatrixServer;
		static bool simRosSetSphericalJointMatrixService(vrep_common::simRosSetSphericalJointMatrix::Request &req,vrep_common::simRosSetSphericalJointMatrix::Response &res);

		static ros::ServiceServer simRosSetStringSignalServer;
		static bool simRosSetStringSignalService(vrep_common::simRosSetStringSignal::Request &req,vrep_common::simRosSetStringSignal::Response &res);

		static ros::ServiceServer simRosAppendStringSignalServer;
		static bool simRosAppendStringSignalService(vrep_common::simRosAppendStringSignal::Request &req,vrep_common::simRosAppendStringSignal::Response &res);

		static ros::ServiceServer simRosSetUIButtonLabelServer;
		static bool simRosSetUIButtonLabelService(vrep_common::simRosSetUIButtonLabel::Request &req,vrep_common::simRosSetUIButtonLabel::Response &res);

		static ros::ServiceServer simRosSetUIButtonPropertyServer;
		static bool simRosSetUIButtonPropertyService(vrep_common::simRosSetUIButtonProperty::Request &req,vrep_common::simRosSetUIButtonProperty::Response &res);

		static ros::ServiceServer simRosSetUISliderServer;
		static bool simRosSetUISliderService(vrep_common::simRosSetUISlider::Request &req,vrep_common::simRosSetUISlider::Response &res);

		static ros::ServiceServer simRosSetVisionSensorImageServer;
		static bool simRosSetVisionSensorImageService(vrep_common::simRosSetVisionSensorImage::Request &req,vrep_common::simRosSetVisionSensorImage::Response &res);

		static ros::ServiceServer simRosStartSimulationServer;
		static bool simRosStartSimulationService(vrep_common::simRosStartSimulation::Request &req,vrep_common::simRosStartSimulation::Response &res);

		static ros::ServiceServer simRosStopSimulationServer;
		static bool simRosStopSimulationService(vrep_common::simRosStopSimulation::Request &req,vrep_common::simRosStopSimulation::Response &res);

		static ros::ServiceServer simRosSynchronousServer;
		static bool simRosSynchronousService(vrep_common::simRosSynchronous::Request &req,vrep_common::simRosSynchronous::Response &res);

		static ros::ServiceServer simRosSynchronousTriggerServer;
		static bool simRosSynchronousTriggerService(vrep_common::simRosSynchronousTrigger::Request &req,vrep_common::simRosSynchronousTrigger::Response &res);

		static ros::ServiceServer simRosTransferFileServer;
		static bool simRosTransferFileService(vrep_common::simRosTransferFile::Request &req,vrep_common::simRosTransferFile::Response &res);

		static ros::ServiceServer simRosEnablePublisherServer;
		static bool simRosEnablePublisherService(vrep_common::simRosEnablePublisher::Request &req,vrep_common::simRosEnablePublisher::Response &res);

		static ros::ServiceServer simRosDisablePublisherServer;
		static bool simRosDisablePublisherService(vrep_common::simRosDisablePublisher::Request &req,vrep_common::simRosDisablePublisher::Response &res);

//		static ros::ServiceServer simRosGetObjectQuaternionServer;
//		static bool simRosGetObjectQuaternionService(vrep_common::simRosGetObjectQuaternion::Request &req,vrep_common::simRosGetObjectQuaternion::Response &res);

		static ros::ServiceServer simRosSetObjectQuaternionServer;
		static bool simRosSetObjectQuaternionService(vrep_common::simRosSetObjectQuaternion::Request &req,vrep_common::simRosSetObjectQuaternion::Response &res);

		static ros::ServiceServer simRosEnableSubscriberServer;
		static bool simRosEnableSubscriberService(vrep_common::simRosEnableSubscriber::Request &req,vrep_common::simRosEnableSubscriber::Response &res);

		static ros::ServiceServer simRosDisableSubscriberServer;
		static bool simRosDisableSubscriberService(vrep_common::simRosDisableSubscriber::Request &req,vrep_common::simRosDisableSubscriber::Response &res);

		static ros::ServiceServer simRosSetJointStateServer;
		static bool simRosSetJointStateService(vrep_common::simRosSetJointState::Request &req,vrep_common::simRosSetJointState::Response &res);

		static ros::ServiceServer simRosCreateDummyServer;
		static bool simRosCreateDummyService(vrep_common::simRosCreateDummy::Request &req,vrep_common::simRosCreateDummy::Response &res);

		static ros::ServiceServer simRosGetAndClearStringSignalServer;
		static bool simRosGetAndClearStringSignalService(vrep_common::simRosGetAndClearStringSignal::Request &req,vrep_common::simRosGetAndClearStringSignal::Response &res);

		static ros::ServiceServer simRosGetObjectGroupDataServer;
		static bool simRosGetObjectGroupDataService(vrep_common::simRosGetObjectGroupData::Request &req,vrep_common::simRosGetObjectGroupData::Response &res);
};

#endif
