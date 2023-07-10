cmake_minimum_required(VERSION 2.8.3)
project(vrep_common)


find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(FILES ForceSensorData.msg  JointSetStateData.msg  ObjectGroupData.msg  ProximitySensorData.msg  VisionSensorData.msg  VisionSensorDepthBuff.msg  VrepInfo.msg  ScriptFunctionCallData.msg)

add_service_files(FILES simRosAddStatusbarMessage.srv      simRosGetDialogInput.srv           simRosGetUIEventButton.srv            simRosSetJointState.srv
simRosAppendStringSignal.srv       simRosGetDialogResult.srv          simRosGetUIHandle.srv                 simRosSetJointTargetPosition.srv
simRosAuxiliaryConsoleClose.srv    simRosGetDistanceHandle.srv        simRosGetUISlider.srv                 simRosSetJointTargetVelocity.srv
simRosAuxiliaryConsoleOpen.srv     simRosGetFloatingParameter.srv     simRosGetVisionSensorDepthBuffer.srv  simRosSetModelProperty.srv
simRosAuxiliaryConsolePrint.srv    simRosGetFloatSignal.srv           simRosGetVisionSensorImage.srv        simRosSetObjectFloatParameter.srv
simRosAuxiliaryConsoleShow.srv     simRosGetInfo.srv                  simRosLoadModel.srv                   simRosSetObjectIntParameter.srv
simRosBreakForceSensor.srv         simRosGetIntegerParameter.srv      simRosLoadScene.srv                   simRosSetObjectParent.srv
simRosClearFloatSignal.srv         simRosGetIntegerSignal.srv         simRosLoadUI.srv                      simRosSetObjectPose.srv
simRosClearIntegerSignal.srv       simRosGetJointMatrix.srv           simRosPauseSimulation.srv             simRosSetObjectPosition.srv
simRosClearStringSignal.srv        simRosGetJointState.srv            simRosReadCollision.srv               simRosSetObjectQuaternion.srv
simRosCloseScene.srv               simRosGetLastErrors.srv            simRosReadDistance.srv                simRosSetObjectSelection.srv
simRosCopyPasteObjects.srv         simRosGetModelProperty.srv         simRosReadForceSensor.srv             simRosSetSphericalJointMatrix.srv
simRosCreateDummy.srv              simRosGetObjectChild.srv           simRosReadProximitySensor.srv         simRosSetStringSignal.srv
simRosDisablePublisher.srv         simRosGetObjectFloatParameter.srv  simRosReadVisionSensor.srv            simRosSetUIButtonLabel.srv
simRosDisableSubscriber.srv        simRosGetObjectGroupData.srv       simRosRemoveObject.srv                simRosSetUIButtonProperty.srv
simRosDisplayDialog.srv            simRosGetObjectHandle.srv          simRosRemoveUI.srv                    simRosSetUISlider.srv
simRosEnablePublisher.srv          simRosGetObjectIntParameter.srv    simRosSetArrayParameter.srv           simRosSetVisionSensorImage.srv
simRosEnableSubscriber.srv         simRosGetObjectParent.srv          simRosSetBooleanParameter.srv         simRosStartSimulation.srv
simRosEndDialog.srv                simRosGetObjectPose.srv            simRosSetFloatingParameter.srv        simRosStopSimulation.srv
simRosEraseFile.srv                simRosGetObjectSelection.srv       simRosSetFloatSignal.srv              simRosSynchronous.srv
simRosGetAndClearStringSignal.srv  simRosGetObjects.srv               simRosSetIntegerParameter.srv         simRosSynchronousTrigger.srv
simRosGetArrayParameter.srv        simRosGetStringParameter.srv       simRosSetIntegerSignal.srv            simRosTransferFile.srv
simRosGetBooleanParameter.srv      simRosGetStringSignal.srv          simRosSetJointForce.srv               simRosRemoveModel.srv
simRosGetCollisionHandle.srv       simRosGetUIButtonProperty.srv      simRosSetJointPosition.srv            simRosGetCollectionHandle.srv
simRosCallScriptFunction.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS message_runtime
)
