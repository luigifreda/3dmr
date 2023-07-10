// This file is part of V-REP, the Virtual Robot Experimentation Platform.
// 
// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// V-REP is dual-licensed, under the terms of EITHER (at your option):
//   1. V-REP commercial license (contact us for details)
//   2. GNU GPL (see below)
// 
// GNU GPL license:
// -------------------------------------------------------------------
// V-REP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// V-REP IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with V-REP.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

#include "../include/v_repLib.h"
#include <stdio.h>
#if defined (__linux) || defined (__APPLE__)
	#include <dlfcn.h>
#endif

#ifndef V_REP_LIBRARY

ptrSimRunSimulator simRunSimulator=0;
ptrSimGetSimulatorMessage simGetSimulatorMessage=0;
ptrSimGetMainWindow simGetMainWindow=0;
ptrSimGetLastError simGetLastError=0;
ptrSimLoadModule simLoadModule=0;
ptrSimUnloadModule simUnloadModule=0;
ptrSimSendModuleMessage simSendModuleMessage=0;
ptrSimSetBooleanParameter simSetBooleanParameter=0;
ptrSimGetBooleanParameter simGetBooleanParameter=0;
ptrSimSetIntegerParameter simSetIntegerParameter=0;
ptrSimGetIntegerParameter simGetIntegerParameter=0;
ptrSimSetFloatingParameter simSetFloatingParameter=0;
ptrSimGetFloatingParameter simGetFloatingParameter=0;
ptrSimSetStringParameter simSetStringParameter=0;
ptrSimGetStringParameter simGetStringParameter=0;
ptrSimGetObjectHandle simGetObjectHandle=0;
ptrSimRemoveObject simRemoveObject=0;
ptrSimRemoveModel simRemoveModel=0;
ptrSimGetObjectName simGetObjectName=0;
ptrSimGetObjects simGetObjects=0;
ptrSimSetObjectName simSetObjectName=0;
ptrSimGetCollectionHandle simGetCollectionHandle=0;
ptrSimRemoveCollection simRemoveCollection=0;
ptrSimGetCollectionName simGetCollectionName=0;
ptrSimSetCollectionName simSetCollectionName=0;
ptrSimGetObjectMatrix simGetObjectMatrix=0;
ptrSimSetObjectMatrix simSetObjectMatrix=0;
ptrSimGetObjectPosition simGetObjectPosition=0;
ptrSimSetObjectPosition simSetObjectPosition=0;
ptrSimGetObjectOrientation simGetObjectOrientation=0;
ptrSimSetObjectOrientation simSetObjectOrientation=0;
ptrSimGetJointPosition simGetJointPosition=0;
ptrSimSetJointPosition simSetJointPosition=0;
ptrSimSetJointTargetPosition simSetJointTargetPosition=0;
ptrSimGetJointTargetPosition simGetJointTargetPosition=0;
ptrSimSetJointForce simSetJointForce=0;
ptrSimGetPathPosition simGetPathPosition=0;
ptrSimSetPathPosition simSetPathPosition=0;
ptrSimGetPathLength simGetPathLength=0;
ptrSimGetJointMatrix simGetJointMatrix=0;
ptrSimSetSphericalJointMatrix simSetSphericalJointMatrix=0;
ptrSimGetJointInterval simGetJointInterval=0;
ptrSimSetJointInterval simSetJointInterval=0;
ptrSimGetObjectParent simGetObjectParent=0;
ptrSimGetObjectChild simGetObjectChild=0;
ptrSimSetObjectParent simSetObjectParent=0;
ptrSimGetObjectType simGetObjectType=0;
ptrSimGetJointType simGetJointType=0;
ptrSimBuildIdentityMatrix simBuildIdentityMatrix=0;
ptrSimCopyMatrix simCopyMatrix=0;
ptrSimBuildMatrix simBuildMatrix=0;
ptrSimGetEulerAnglesFromMatrix simGetEulerAnglesFromMatrix=0;
ptrSimInvertMatrix simInvertMatrix=0;
ptrSimMultiplyMatrices simMultiplyMatrices=0;
ptrSimInterpolateMatrices simInterpolateMatrices=0;
ptrSimTransformVector simTransformVector=0;
ptrSimReservedCommand simReservedCommand=0;
ptrSimGetSimulationTime simGetSimulationTime=0;
ptrSimGetSimulationState simGetSimulationState=0;
ptrSimGetSystemTime simGetSystemTime=0;
ptrSimGetSystemTimeInMilliseconds simGetSystemTimeInMilliseconds=0;
ptrSimGetSystemTimeInMs simGetSystemTimeInMs=0;
ptrSimLoadScene simLoadScene=0;
ptrSimCloseScene simCloseScene=0;
ptrSimSaveScene simSaveScene=0;
ptrSimLoadModel simLoadModel=0;
ptrSimSaveModel simSaveModel=0;
ptrSimLoadUI simLoadUI=0;
ptrSimSaveUI simSaveUI=0;
ptrSimAddStatusbarMessage simAddStatusbarMessage=0;
ptrSimAddModuleMenuEntry simAddModuleMenuEntry=0;
ptrSimSetModuleMenuItemState simSetModuleMenuItemState=0;
ptrSimDoesFileExist simDoesFileExist=0;
ptrSimIsObjectInSelection simIsObjectInSelection=0;
ptrSimAddObjectToSelection simAddObjectToSelection=0;
ptrSimRemoveObjectFromSelection simRemoveObjectFromSelection=0;
ptrSimGetObjectSelectionSize simGetObjectSelectionSize=0;
ptrSimGetObjectLastSelection simGetObjectLastSelection=0;
ptrSimGetObjectSelection simGetObjectSelection=0;
ptrSimSearchPath simSearchPath=0;
ptrSimInitializePathSearch simInitializePathSearch=0;
ptrSimPerformPathSearchStep simPerformPathSearchStep=0;
ptrSimHandleCollision simHandleCollision=0;
ptrSimReadCollision simReadCollision=0;
ptrSimHandleDistance simHandleDistance=0;
ptrSimReadDistance simReadDistance=0;
ptrSimHandleProximitySensor simHandleProximitySensor=0;
ptrSimReadProximitySensor simReadProximitySensor=0;
ptrSimHandleMill simHandleMill=0;
ptrSimHandleIkGroup simHandleIkGroup=0;
ptrSimCheckIkGroup simCheckIkGroup=0;
ptrSimHandleDynamics simHandleDynamics=0;
ptrSimGetMechanismHandle simGetMechanismHandle=0;
ptrSimGetPathPlanningHandle simGetPathPlanningHandle=0;
ptrSimHandleMechanism simHandleMechanism=0;
ptrSimGetScriptHandle simGetScriptHandle=0;
ptrSimSetScriptText simSetScriptText=0;
ptrSimGetScriptText simGetScriptText=0;
ptrSimGetScriptProperty simGetScriptProperty=0;
ptrSimAssociateScriptWithObject simAssociateScriptWithObject=0;
ptrSimGetScript simGetScript=0;
ptrSimGetScriptAssociatedWithObject simGetScriptAssociatedWithObject=0;
ptrSimGetCustomizationScriptAssociatedWithObject simGetCustomizationScriptAssociatedWithObject=0;
ptrSimGetObjectAssociatedWithScript simGetObjectAssociatedWithScript=0;
ptrSimGetScriptName simGetScriptName=0;
ptrSimHandleMainScript simHandleMainScript=0;
ptrSimHandleGeneralCallbackScript simHandleGeneralCallbackScript=0;
ptrSimResetScript simResetScript=0;
ptrSimAddScript simAddScript=0;
ptrSimRemoveScript simRemoveScript=0;
ptrSimRefreshDialogs simRefreshDialogs=0;
ptrSimGetCollisionHandle simGetCollisionHandle=0;
ptrSimGetDistanceHandle simGetDistanceHandle=0;
ptrSimGetIkGroupHandle simGetIkGroupHandle=0;
ptrSimResetCollision simResetCollision=0;
ptrSimResetDistance simResetDistance=0;
ptrSimResetProximitySensor simResetProximitySensor=0;
ptrSimResetMill simResetMill=0;
ptrSimCheckProximitySensor simCheckProximitySensor=0;
ptrSimCheckProximitySensorEx simCheckProximitySensorEx=0;
ptrSimCheckProximitySensorEx2 simCheckProximitySensorEx2=0;
ptrSimAddSceneCustomData simAddSceneCustomData=0;
ptrSimGetSceneCustomDataLength simGetSceneCustomDataLength=0;
ptrSimGetSceneCustomData simGetSceneCustomData=0;
ptrSimAddObjectCustomData simAddObjectCustomData=0;
ptrSimGetObjectCustomDataLength simGetObjectCustomDataLength=0;
ptrSimGetObjectCustomData simGetObjectCustomData=0;
ptrSimCreateBuffer simCreateBuffer=0;
ptrSimReleaseBuffer simReleaseBuffer=0;
ptrSimCheckCollision simCheckCollision=0;
ptrSimCheckCollisionEx simCheckCollisionEx=0;
ptrSimCheckDistance simCheckDistance=0;
ptrSimGetObjectConfiguration simGetObjectConfiguration=0;
ptrSimSetObjectConfiguration simSetObjectConfiguration=0;
ptrSimGetConfigurationTree simGetConfigurationTree=0;
ptrSimSetConfigurationTree simSetConfigurationTree=0;
ptrSimSetSimulationTimeStep simSetSimulationTimeStep=0;
ptrSimGetSimulationTimeStep simGetSimulationTimeStep=0;
ptrSimGetRealTimeSimulation simGetRealTimeSimulation=0;
ptrSimIsRealTimeSimulationStepNeeded simIsRealTimeSimulationStepNeeded=0;
ptrSimAdjustRealTimeTimer simAdjustRealTimeTimer=0;
ptrSimGetSimulationPassesPerRenderingPass simGetSimulationPassesPerRenderingPass=0;
ptrSimAdvanceSimulationByOneStep simAdvanceSimulationByOneStep=0;
ptrSimStartSimulation simStartSimulation=0;
ptrSimStopSimulation simStopSimulation=0;
ptrSimPauseSimulation simPauseSimulation=0;
ptrSimBroadcastMessage simBroadcastMessage=0;
ptrSimGetModuleName simGetModuleName=0;
ptrSimGetScriptSimulationParameter simGetScriptSimulationParameter=0;
ptrSimSetScriptSimulationParameter simSetScriptSimulationParameter=0;
ptrSimFloatingViewAdd simFloatingViewAdd=0;
ptrSimFloatingViewRemove simFloatingViewRemove=0;
ptrSimAdjustView simAdjustView=0;
ptrSimResetPath simResetPath=0;
ptrSimHandlePath simHandlePath=0;
ptrSimResetJoint simResetJoint=0;
ptrSimHandleJoint simHandleJoint=0;
ptrSimSetLastError simSetLastError=0;
ptrSimHandleGraph simHandleGraph=0;
ptrSimResetGraph simResetGraph=0;
ptrSimSetNavigationMode simSetNavigationMode=0;
ptrSimGetNavigationMode simGetNavigationMode=0;
ptrSimSetPage simSetPage=0;
ptrSimGetPage simGetPage=0;
ptrSimDisplayDialog simDisplayDialog=0;
ptrSimGetDialogResult simGetDialogResult=0;
ptrSimGetDialogInput simGetDialogInput=0;
ptrSimEndDialog simEndDialog=0;
ptrSimRegisterCustomLuaFunction simRegisterCustomLuaFunction=0;
ptrSimRegisterCustomLuaVariable simRegisterCustomLuaVariable=0;
ptrSimSetJointTargetVelocity simSetJointTargetVelocity=0;
ptrSimGetJointTargetVelocity simGetJointTargetVelocity=0;
ptrSimSetPathTargetNominalVelocity simSetPathTargetNominalVelocity=0;
ptrSimLockInterface simLockInterface=0;
ptrSimGetScriptRawBuffer simGetScriptRawBuffer=0;
ptrSimSetScriptRawBuffer simSetScriptRawBuffer=0;
ptrSimReleaseScriptRawBuffer simReleaseScriptRawBuffer=0;
ptrSimCopyPasteSelectedObjects simCopyPasteSelectedObjects=0;
ptrSimCopyPasteObjects simCopyPasteObjects=0;
ptrSimScaleSelectedObjects simScaleSelectedObjects=0;
ptrSimScaleObjects simScaleObjects=0;
ptrSimDeleteSelectedObjects simDeleteSelectedObjects=0;
ptrSimGetObjectUniqueIdentifier simGetObjectUniqueIdentifier=0;
ptrSimGetNameSuffix simGetNameSuffix=0;
ptrSimSendData simSendData=0;
ptrSimReceiveData simReceiveData=0;
ptrSimSetGraphUserData simSetGraphUserData=0;
ptrSimSetNameSuffix simSetNameSuffix=0;
ptrSimAddDrawingObject simAddDrawingObject=0;
ptrSimRemoveDrawingObject simRemoveDrawingObject=0;
ptrSimAddDrawingObjectItem simAddDrawingObjectItem=0;
ptrSimAddParticleObject simAddParticleObject=0;
ptrSimRemoveParticleObject simRemoveParticleObject=0;
ptrSimAddParticleObjectItem simAddParticleObjectItem=0;
ptrSimGetObjectSizeFactor simGetObjectSizeFactor=0;
ptrSimAnnounceSceneContentChange simAnnounceSceneContentChange=0;
ptrSimResetMilling simResetMilling=0;
ptrSimApplyMilling simApplyMilling=0;
ptrSimSetIntegerSignal simSetIntegerSignal=0;
ptrSimGetIntegerSignal simGetIntegerSignal=0;
ptrSimClearIntegerSignal simClearIntegerSignal=0;
ptrSimSetFloatSignal simSetFloatSignal=0;
ptrSimGetFloatSignal simGetFloatSignal=0;
ptrSimClearFloatSignal simClearFloatSignal=0;
ptrSimSetStringSignal simSetStringSignal=0;
ptrSimGetStringSignal simGetStringSignal=0;
ptrSimClearStringSignal simClearStringSignal=0;
ptrSimGetSignalName simGetSignalName=0;
ptrSimSetObjectProperty simSetObjectProperty=0;
ptrSimGetObjectProperty simGetObjectProperty=0;
ptrSimSetObjectSpecialProperty simSetObjectSpecialProperty=0;
ptrSimGetObjectSpecialProperty simGetObjectSpecialProperty=0;
ptrSimGetPositionOnPath simGetPositionOnPath=0;
ptrSimGetDataOnPath simGetDataOnPath=0;
ptrSimGetOrientationOnPath simGetOrientationOnPath=0;
ptrSimGetClosestPositionOnPath simGetClosestPositionOnPath=0;
ptrSimReadForceSensor simReadForceSensor=0;
ptrSimBreakForceSensor simBreakForceSensor=0;
ptrSimGetShapeVertex simGetShapeVertex=0;
ptrSimGetShapeTriangle simGetShapeTriangle=0;
ptrSimSetLightParameters simSetLightParameters=0;
ptrSimGetLightParameters simGetLightParameters=0;
ptrSimHandleVarious simHandleVarious=0;
ptrSimGetVelocity simGetVelocity=0;
ptrSimGetObjectVelocity simGetObjectVelocity=0;
ptrSimAddForceAndTorque simAddForceAndTorque=0;
ptrSimAddForce simAddForce=0;
ptrSimSetExplicitHandling simSetExplicitHandling=0;
ptrSimGetExplicitHandling simGetExplicitHandling=0;
ptrSimGetLinkDummy simGetLinkDummy=0;
ptrSimSetLinkDummy simSetLinkDummy=0;
ptrSimSetModelProperty simSetModelProperty=0;
ptrSimGetModelProperty simGetModelProperty=0;
ptrSimSetShapeColor simSetShapeColor=0;
ptrSimGetShapeColor simGetShapeColor=0;
ptrSimResetDynamicObject simResetDynamicObject=0;
ptrSimSetJointMode simSetJointMode=0;
ptrSimGetJointMode simGetJointMode=0;
ptrSimSerialOpen simSerialOpen=0;
ptrSimSerialClose simSerialClose=0;
ptrSimSerialSend simSerialSend=0;
ptrSimSerialRead simSerialRead=0;
ptrSimSerialCheck simSerialCheck=0;
ptrSimSerialPortOpen simSerialPortOpen=0;
ptrSimSerialPortClose simSerialPortClose=0;
ptrSimSerialPortSend simSerialPortSend=0;
ptrSimSerialPortRead simSerialPortRead=0;
ptrSimGetContactInfo simGetContactInfo=0;
ptrSimSetThreadIsFree simSetThreadIsFree=0;
ptrSimTubeOpen simTubeOpen=0;
ptrSimTubeClose simTubeClose=0;
ptrSimTubeWrite simTubeWrite=0;
ptrSimTubeRead simTubeRead=0;
ptrSimTubeStatus simTubeStatus=0;
ptrSimAuxiliaryConsoleOpen simAuxiliaryConsoleOpen=0;
ptrSimAuxiliaryConsoleClose simAuxiliaryConsoleClose=0;
ptrSimAuxiliaryConsoleShow simAuxiliaryConsoleShow=0;
ptrSimAuxiliaryConsolePrint simAuxiliaryConsolePrint=0;
ptrSimImportShape simImportShape=0;
ptrSimImportMesh simImportMesh=0;
ptrSimExportMesh simExportMesh=0;
ptrSimCreateMeshShape simCreateMeshShape=0;
ptrSimCreatePureShape simCreatePureShape=0;
ptrSimCreateHeightfieldShape simCreateHeightfieldShape=0;
ptrSimGetShapeMesh simGetShapeMesh=0;
ptrSimAddBanner simAddBanner=0;
ptrSimRemoveBanner simRemoveBanner=0;
ptrSimCreateJoint simCreateJoint=0;
ptrSimCreateDummy simCreateDummy=0;
ptrSimCreateProximitySensor simCreateProximitySensor=0;
ptrSimCreatePath simCreatePath=0;
ptrSimInsertPathCtrlPoints simInsertPathCtrlPoints=0;
ptrSimCutPathCtrlPoints simCutPathCtrlPoints=0;
ptrSimCreateForceSensor simCreateForceSensor=0;
ptrSimCreateVisionSensor simCreateVisionSensor=0;
ptrSimRegisterContactCallback simRegisterContactCallback=0;
ptrSimGetObjectIntParameter simGetObjectIntParameter=0;
ptrSimSetObjectIntParameter simSetObjectIntParameter=0;
ptrSimGetObjectFloatParameter simGetObjectFloatParameter=0;
ptrSimSetObjectFloatParameter simSetObjectFloatParameter=0;
ptrSimGetObjectStringParameter simGetObjectStringParameter=0;
ptrSimSetObjectStringParameter simSetObjectStringParameter=0;
ptrSimSetSimulationPassesPerRenderingPass simSetSimulationPassesPerRenderingPass=0;
ptrSimGetRotationAxis simGetRotationAxis=0;
ptrSimRotateAroundAxis simRotateAroundAxis=0;
ptrSimJointGetForce simJointGetForce=0;
ptrSimGetJointForce simGetJointForce=0;
ptrSimSetArrayParameter simSetArrayParameter=0;
ptrSimGetArrayParameter simGetArrayParameter=0;
ptrSimSetIkGroupProperties simSetIkGroupProperties=0;
ptrSimSetIkElementProperties simSetIkElementProperties=0;
ptrSimCameraFitToView simCameraFitToView=0;
ptrSimPersistentDataWrite simPersistentDataWrite=0;
ptrSimPersistentDataRead simPersistentDataRead=0;
ptrSimIsHandleValid simIsHandleValid=0;
ptrSimHandleVisionSensor simHandleVisionSensor=0;
ptrSimReadVisionSensor simReadVisionSensor=0;
ptrSimResetVisionSensor simResetVisionSensor=0;
ptrSimCheckVisionSensor simCheckVisionSensor=0;
ptrSimCheckVisionSensorEx simCheckVisionSensorEx=0;
ptrSimGetVisionSensorResolution simGetVisionSensorResolution=0;
ptrSimGetVisionSensorImage simGetVisionSensorImage=0;
ptrSimGetVisionSensorCharImage simGetVisionSensorCharImage=0;
ptrSimSetVisionSensorImage simSetVisionSensorImage=0;
ptrSimSetVisionSensorCharImage simSetVisionSensorCharImage=0;
ptrSimGetVisionSensorDepthBuffer simGetVisionSensorDepthBuffer=0;
ptrSimCreateUI simCreateUI=0;
ptrSimCreateUIButton simCreateUIButton=0;
ptrSimGetUIHandle simGetUIHandle=0;
ptrSimGetUIProperty simGetUIProperty=0;
ptrSimGetUIEventButton simGetUIEventButton=0;
ptrSimSetUIProperty simSetUIProperty=0;
ptrSimGetUIButtonProperty simGetUIButtonProperty=0;
ptrSimSetUIButtonProperty simSetUIButtonProperty=0;
ptrSimGetUIButtonSize simGetUIButtonSize=0;
ptrSimSetUIButtonLabel simSetUIButtonLabel=0;
ptrSimGetUIButtonLabel simGetUIButtonLabel=0;
ptrSimSetUISlider simSetUISlider=0;
ptrSimGetUISlider simGetUISlider=0;
ptrSimSetUIButtonColor simSetUIButtonColor=0;
ptrSimSetUIButtonTexture simSetUIButtonTexture=0;
ptrSimCreateUIButtonArray simCreateUIButtonArray=0;
ptrSimSetUIButtonArrayColor simSetUIButtonArrayColor=0;
ptrSimDeleteUIButtonArray simDeleteUIButtonArray=0;
ptrSimRemoveUI simRemoveUI=0;
ptrSimSetUIPosition simSetUIPosition=0;
ptrSimGetUIPosition simGetUIPosition=0;
ptrSimGetObjectQuaternion simGetObjectQuaternion=0;
ptrSimSetObjectQuaternion simSetObjectQuaternion=0;
ptrSimRMLPosition simRMLPosition=0;
ptrSimRMLVelocity simRMLVelocity=0;
ptrSimRMLPos simRMLPos=0;
ptrSimRMLVel simRMLVel=0;
ptrSimRMLStep simRMLStep=0;
ptrSimRMLRemove simRMLRemove=0;
ptrSimBuildMatrixQ simBuildMatrixQ=0;
ptrSimGetQuaternionFromMatrix simGetQuaternionFromMatrix=0;
ptrSimFileDialog simFileDialog=0;
ptrSimMsgBox simMsgBox=0;
ptrSimSetShapeMassAndInertia simSetShapeMassAndInertia=0;
ptrSimGetShapeMassAndInertia simGetShapeMassAndInertia=0;
ptrSimGroupShapes simGroupShapes=0;
ptrSimUngroupShape simUngroupShape=0;
ptrSimConvexDecompose simConvexDecompose=0;
ptrSimGetIkGroupMatrix simGetIkGroupMatrix=0;
ptrSimGetMotionPlanningHandle simGetMotionPlanningHandle=0;
ptrSimGetMpConfigForTipPose simGetMpConfigForTipPose=0;
ptrSimFindMpPath simFindMpPath=0;
ptrSimSimplifyMpPath simSimplifyMpPath=0;
ptrSimGetMpConfigTransition simGetMpConfigTransition=0;
ptrSimAddGhost simAddGhost=0;
ptrSimModifyGhost simModifyGhost=0;
ptrSimQuitSimulator simQuitSimulator=0;
ptrSimGetThreadId simGetThreadId=0;
ptrSimLockResources simLockResources=0;
ptrSimUnlockResources simUnlockResources=0;
ptrSimEnableEventCallback simEnableEventCallback=0;
ptrSimGetMaterialId simGetMaterialId=0;
ptrSimSetShapeMaterial simSetShapeMaterial=0;
ptrSimGetShapeMaterial simGetShapeMaterial=0;
ptrSimFindIkPath simFindIkPath=0;
ptrSimGetTextureId simGetTextureId=0;
ptrSimReadTexture simReadTexture=0;
ptrSimWriteTexture simWriteTexture=0;
ptrSimCreateTexture simCreateTexture=0;
ptrSimWriteCustomDataBlock simWriteCustomDataBlock=0;
ptrSimReadCustomDataBlock simReadCustomDataBlock=0;
ptrSimAddPointCloud simAddPointCloud=0;
ptrSimModifyPointCloud simModifyPointCloud=0;
ptrSimGetShapeGeomInfo simGetShapeGeomInfo=0;
ptrSimGetObjectsInTree simGetObjectsInTree=0;
ptrSimSetObjectSizeValues simSetObjectSizeValues=0;
ptrSimGetObjectSizeValues simGetObjectSizeValues=0;
ptrSimScaleObject simScaleObject=0;
ptrSimSetShapeTexture simSetShapeTexture=0;
ptrSimGetShapeTextureId simGetShapeTextureId=0;
ptrSimGetCollectionObjects simGetCollectionObjects=0;
ptrSimHandleCustomizationScripts simHandleCustomizationScripts=0;
ptrSimSetScriptAttribute simSetScriptAttribute=0;
ptrSimGetScriptAttribute simGetScriptAttribute=0;
ptrSimReorientShapeBoundingBox simReorientShapeBoundingBox=0;
ptrSimSwitchThread simSwitchThread=0;
ptrSimCreateIkGroup simCreateIkGroup=0;
ptrSimRemoveIkGroup simRemoveIkGroup=0;
ptrSimCreateIkElement simCreateIkElement=0;
ptrSimCreateMotionPlanning simCreateMotionPlanning=0;
ptrSimRemoveMotionPlanning simRemoveMotionPlanning=0;
ptrSimCreateCollection simCreateCollection=0;
ptrSimAddObjectToCollection simAddObjectToCollection=0;
ptrSimSaveImage simSaveImage=0;









ptr_simGetContactCallbackCount _simGetContactCallbackCount=0;
ptr_simGetContactCallback _simGetContactCallback=0;
ptr_simSetDynamicSimulationIconCode _simSetDynamicSimulationIconCode=0;
ptr_simSetDynamicObjectFlagForVisualization _simSetDynamicObjectFlagForVisualization=0;
ptr_simGetObjectListSize _simGetObjectListSize=0;
ptr_simGetObjectFromIndex _simGetObjectFromIndex=0;
ptr_simGetObjectID _simGetObjectID=0;
ptr_simGetObjectType _simGetObjectType=0;
ptr_simGetObjectChildren _simGetObjectChildren=0;
ptr_simGetGeomProxyFromShape _simGetGeomProxyFromShape=0;
ptr_simGetParentObject _simGetParentObject=0;
ptr_simGetObject _simGetObject=0;
ptr_simGetIkGroupObject _simGetIkGroupObject=0;
ptr_simMpHandleIkGroupObject _simMpHandleIkGroupObject=0;
ptr_simGetObjectLocalTransformation _simGetObjectLocalTransformation=0;
ptr_simSetObjectLocalTransformation _simSetObjectLocalTransformation=0;
ptr_simSetObjectCumulativeTransformation _simSetObjectCumulativeTransformation=0;
ptr_simGetObjectCumulativeTransformation _simGetObjectCumulativeTransformation=0;
ptr_simIsShapeDynamicallyStatic _simIsShapeDynamicallyStatic=0;
ptr_simGetTreeDynamicProperty _simGetTreeDynamicProperty=0;
ptr_simGetDummyLinkType _simGetDummyLinkType=0;
ptr_simGetJointMode _simGetJointMode=0;
ptr_simIsJointInHybridOperation _simIsJointInHybridOperation=0;
ptr_simDisableDynamicTreeForManipulation _simDisableDynamicTreeForManipulation=0;
ptr_simIsShapeDynamicallyRespondable _simIsShapeDynamicallyRespondable=0;
ptr_simGetDynamicCollisionMask _simGetDynamicCollisionMask=0;
ptr_simGetLastParentForLocalGlobalCollidable _simGetLastParentForLocalGlobalCollidable=0;
ptr_simSetShapeIsStaticAndNotRespondableButDynamicTag _simSetShapeIsStaticAndNotRespondableButDynamicTag=0;
ptr_simGetShapeIsStaticAndNotRespondableButDynamicTag _simGetShapeIsStaticAndNotRespondableButDynamicTag=0;
ptr_simSetJointPosition _simSetJointPosition=0;
ptr_simGetJointPosition _simGetJointPosition=0;
ptr_simSetDynamicMotorPositionControlTargetPosition _simSetDynamicMotorPositionControlTargetPosition=0;
ptr_simGetInitialDynamicVelocity _simGetInitialDynamicVelocity=0;
ptr_simSetInitialDynamicVelocity _simSetInitialDynamicVelocity=0;
ptr_simGetInitialDynamicAngVelocity _simGetInitialDynamicAngVelocity=0;
ptr_simSetInitialDynamicAngVelocity _simSetInitialDynamicAngVelocity=0;
ptr_simGetStartSleeping _simGetStartSleeping=0;
ptr_simGetWasPutToSleepOnce _simGetWasPutToSleepOnce=0;
ptr_simGetDynamicsFullRefreshFlag _simGetDynamicsFullRefreshFlag=0;
ptr_simSetDynamicsFullRefreshFlag _simSetDynamicsFullRefreshFlag=0;
ptr_simSetGeomProxyDynamicsFullRefreshFlag _simSetGeomProxyDynamicsFullRefreshFlag=0;
ptr_simGetGeomProxyDynamicsFullRefreshFlag _simGetGeomProxyDynamicsFullRefreshFlag=0;
ptr_simGetParentFollowsDynamic _simGetParentFollowsDynamic=0;
ptr_simSetShapeDynamicVelocity _simSetShapeDynamicVelocity=0;
ptr_simGetAdditionalForceAndTorque _simGetAdditionalForceAndTorque=0;
ptr_simClearAdditionalForceAndTorque _simClearAdditionalForceAndTorque=0;
ptr_simGetJointPositionInterval _simGetJointPositionInterval=0;
ptr_simGetJointType _simGetJointType=0;
ptr_simGetJointOdeParameters _simGetJointOdeParameters=0;
ptr_simGetJointBulletParameters _simGetJointBulletParameters=0;
ptr_simIsForceSensorBroken _simIsForceSensorBroken=0;
ptr_simGetDynamicForceSensorLocalTransformationPart2 _simGetDynamicForceSensorLocalTransformationPart2=0;
ptr_simIsDynamicMotorEnabled _simIsDynamicMotorEnabled=0;
ptr_simIsDynamicMotorPositionCtrlEnabled _simIsDynamicMotorPositionCtrlEnabled=0;
ptr_simIsDynamicMotorTorqueModulationEnabled _simIsDynamicMotorTorqueModulationEnabled=0;
ptr_simGetMotorPid _simGetMotorPid=0;
ptr_simGetDynamicMotorTargetPosition _simGetDynamicMotorTargetPosition=0;
ptr_simGetDynamicMotorTargetVelocity _simGetDynamicMotorTargetVelocity=0;
ptr_simGetDynamicMotorMaxForce _simGetDynamicMotorMaxForce=0;
ptr_simGetDynamicMotorUpperLimitVelocity _simGetDynamicMotorUpperLimitVelocity=0;
ptr_simSetDynamicMotorReflectedPositionFromDynamicEngine _simSetDynamicMotorReflectedPositionFromDynamicEngine=0;
ptr_simSetJointSphericalTransformation _simSetJointSphericalTransformation=0;
ptr_simAddForceSensorCumulativeForcesAndTorques _simAddForceSensorCumulativeForcesAndTorques=0;
ptr_simAddJointCumulativeForcesOrTorques _simAddJointCumulativeForcesOrTorques=0;
ptr_simSetDynamicJointLocalTransformationPart2 _simSetDynamicJointLocalTransformationPart2=0;
ptr_simSetDynamicForceSensorLocalTransformationPart2 _simSetDynamicForceSensorLocalTransformationPart2=0;
ptr_simSetDynamicJointLocalTransformationPart2IsValid _simSetDynamicJointLocalTransformationPart2IsValid=0;
ptr_simSetDynamicForceSensorLocalTransformationPart2IsValid _simSetDynamicForceSensorLocalTransformationPart2IsValid=0;
ptr_simGetGeomWrapFromGeomProxy _simGetGeomWrapFromGeomProxy=0;
ptr_simGetLocalInertiaFrame _simGetLocalInertiaFrame=0;
ptr_simGetPurePrimitiveType _simGetPurePrimitiveType=0;
ptr_simIsGeomWrapGeometric _simIsGeomWrapGeometric=0;
ptr_simIsGeomWrapConvex _simIsGeomWrapConvex=0;
ptr_simGetGeometricCount _simGetGeometricCount=0;
ptr_simGetAllGeometrics _simGetAllGeometrics=0;
ptr_simGetPurePrimitiveSizes _simGetPurePrimitiveSizes=0;
ptr_simMakeDynamicAnnouncement _simMakeDynamicAnnouncement=0;
ptr_simGetVerticesLocalFrame _simGetVerticesLocalFrame=0;
ptr_simGetHeightfieldData _simGetHeightfieldData=0;
ptr_simGetCumulativeMeshes _simGetCumulativeMeshes=0;
ptr_simGetOdeMaxContactFrictionCFMandERP _simGetOdeMaxContactFrictionCFMandERP=0;
ptr_simGetBulletCollisionMargin _simGetBulletCollisionMargin=0;
ptr_simGetBulletStickyContact _simGetBulletStickyContact=0;
ptr_simGetBulletRestitution _simGetBulletRestitution=0;
ptr_simGetMass _simGetMass=0;
ptr_simGetPrincipalMomentOfInertia _simGetPrincipalMomentOfInertia=0;
ptr_simGetDamping _simGetDamping=0;
ptr_simGetFriction _simGetFriction=0;
ptr_simGetGravity _simGetGravity=0;
ptr_simGetTimeDiffInMs _simGetTimeDiffInMs=0;
ptr_simDoEntitiesCollide _simDoEntitiesCollide=0;
ptr_simGetDistanceBetweenEntitiesIfSmaller _simGetDistanceBetweenEntitiesIfSmaller=0;
ptr_simHandleJointControl _simHandleJointControl=0;
ptr_simHandleCustomContact _simHandleCustomContact=0;
ptr_simGetPureHollowScaling _simGetPureHollowScaling=0;
ptr_simGetVortexParameters _simGetVortexParameters=0;
ptr_simGetJointCallbackCallOrder _simGetJointCallbackCallOrder=0;
ptr_simGetNewtonParameters _simGetNewtonParameters=0;


LIBRARY loadVrepLibrary(const char* pathAndFilename)
{
	#ifdef QT_FRAMEWORK
		QLibrary* lib=new QLibrary(pathAndFilename);
		if (!lib->load())
		{
			delete lib;
			lib=NULL;
		}
		return lib;
	#else
		#ifdef _WIN32
			return LoadLibrary(pathAndFilename);
		#elif defined (__linux) || defined (__APPLE__)
			return dlopen(pathAndFilename,RTLD_LAZY);
		#endif
	#endif // QT_FRAMEWORK
}

void unloadVrepLibrary(LIBRARY lib)
{
	#ifdef QT_FRAMEWORK
		if (lib!=0)
		{
			lib->unload();
			delete lib;
		}
	#else
		#ifdef _WIN32
			if (lib!=0)
				FreeLibrary(lib);
		#elif defined (__linux) || defined (__APPLE__)
				dlclose(lib);
		#endif
	#endif // QT_FRAMEWORK
}

FARPROC _getProcAddress(LIBRARY lib,const char* funcName)
{
	#ifdef QT_FRAMEWORK
		return (void*)lib->resolve(funcName);
	#else
		#ifdef _WIN32
			return GetProcAddress(lib,funcName);
		#elif defined (__linux) || defined (__APPLE__)
			return dlsym(lib,funcName);
		#endif
	#endif // QT_FRAMEWORK
}

int getVrepProcAddresses(LIBRARY lib)
{
	simRunSimulator=(ptrSimRunSimulator)(_getProcAddress(lib,"simRunSimulator"));
	simGetSimulatorMessage=(ptrSimGetSimulatorMessage)(_getProcAddress(lib,"simGetSimulatorMessage"));
	simGetMainWindow=(ptrSimGetMainWindow)(_getProcAddress(lib,"simGetMainWindow"));
	simGetLastError=(ptrSimGetLastError)(_getProcAddress(lib,"simGetLastError"));
	simLoadModule=(ptrSimLoadModule)(_getProcAddress(lib,"simLoadModule"));
	simUnloadModule=(ptrSimUnloadModule)(_getProcAddress(lib,"simUnloadModule"));
	simSendModuleMessage=(ptrSimSendModuleMessage)(_getProcAddress(lib,"simSendModuleMessage"));
	simSetBooleanParameter=(ptrSimSetBooleanParameter)(_getProcAddress(lib,"simSetBooleanParameter"));
	simGetBooleanParameter=(ptrSimGetBooleanParameter)(_getProcAddress(lib,"simGetBooleanParameter"));
	simSetIntegerParameter=(ptrSimSetIntegerParameter)(_getProcAddress(lib,"simSetIntegerParameter"));
	simGetIntegerParameter=(ptrSimGetIntegerParameter)(_getProcAddress(lib,"simGetIntegerParameter"));
	simSetFloatingParameter=(ptrSimSetFloatingParameter)(_getProcAddress(lib,"simSetFloatingParameter"));
	simGetFloatingParameter=(ptrSimGetFloatingParameter)(_getProcAddress(lib,"simGetFloatingParameter"));
	simSetStringParameter=(ptrSimSetStringParameter)(_getProcAddress(lib,"simSetStringParameter"));
	simGetStringParameter=(ptrSimGetStringParameter)(_getProcAddress(lib,"simGetStringParameter"));
	simGetObjectHandle=(ptrSimGetObjectHandle)(_getProcAddress(lib,"simGetObjectHandle"));
	simRemoveObject=(ptrSimRemoveObject)(_getProcAddress(lib,"simRemoveObject"));
	simRemoveModel=(ptrSimRemoveModel)(_getProcAddress(lib,"simRemoveModel"));
	simGetObjectName=(ptrSimGetObjectName)(_getProcAddress(lib,"simGetObjectName"));
	simGetObjects=(ptrSimGetObjects)(_getProcAddress(lib,"simGetObjects"));
	simSetObjectName=(ptrSimSetObjectName)(_getProcAddress(lib,"simSetObjectName"));
	simGetCollectionHandle=(ptrSimGetCollectionHandle)(_getProcAddress(lib,"simGetCollectionHandle"));
	simRemoveCollection=(ptrSimRemoveCollection)(_getProcAddress(lib,"simRemoveCollection"));
	simGetCollectionName=(ptrSimGetCollectionName)(_getProcAddress(lib,"simGetCollectionName"));
	simSetCollectionName=(ptrSimSetCollectionName)(_getProcAddress(lib,"simSetCollectionName"));
	simGetObjectMatrix=(ptrSimGetObjectMatrix)(_getProcAddress(lib,"simGetObjectMatrix"));
	simSetObjectMatrix=(ptrSimSetObjectMatrix)(_getProcAddress(lib,"simSetObjectMatrix"));
	simGetObjectPosition=(ptrSimGetObjectPosition)(_getProcAddress(lib,"simGetObjectPosition"));
	simSetObjectPosition=(ptrSimSetObjectPosition)(_getProcAddress(lib,"simSetObjectPosition"));
	simGetObjectOrientation=(ptrSimGetObjectOrientation)(_getProcAddress(lib,"simGetObjectOrientation"));
	simSetObjectOrientation=(ptrSimSetObjectOrientation)(_getProcAddress(lib,"simSetObjectOrientation"));
	simGetJointPosition=(ptrSimGetJointPosition)(_getProcAddress(lib,"simGetJointPosition"));
	simSetJointPosition=(ptrSimSetJointPosition)(_getProcAddress(lib,"simSetJointPosition"));
	simSetJointTargetPosition=(ptrSimSetJointTargetPosition)(_getProcAddress(lib,"simSetJointTargetPosition"));
	simGetJointTargetPosition=(ptrSimGetJointTargetPosition)(_getProcAddress(lib,"simGetJointTargetPosition"));
	simSetJointForce=(ptrSimSetJointForce)(_getProcAddress(lib,"simSetJointForce"));
	simGetPathPosition=(ptrSimGetPathPosition)(_getProcAddress(lib,"simGetPathPosition"));
	simSetPathPosition=(ptrSimSetPathPosition)(_getProcAddress(lib,"simSetPathPosition"));
	simGetPathLength=(ptrSimGetPathLength)(_getProcAddress(lib,"simGetPathLength"));
	simGetJointMatrix=(ptrSimGetJointMatrix)(_getProcAddress(lib,"simGetJointMatrix"));
	simSetSphericalJointMatrix=(ptrSimSetSphericalJointMatrix)(_getProcAddress(lib,"simSetSphericalJointMatrix"));
	simGetJointInterval=(ptrSimGetJointInterval)(_getProcAddress(lib,"simGetJointInterval"));
	simSetJointInterval=(ptrSimSetJointInterval)(_getProcAddress(lib,"simSetJointInterval"));
	simGetObjectParent=(ptrSimGetObjectParent)(_getProcAddress(lib,"simGetObjectParent"));
	simGetObjectChild=(ptrSimGetObjectChild)(_getProcAddress(lib,"simGetObjectChild"));
	simSetObjectParent=(ptrSimSetObjectParent)(_getProcAddress(lib,"simSetObjectParent"));
	simGetObjectType=(ptrSimGetObjectType)(_getProcAddress(lib,"simGetObjectType"));
	simGetJointType=(ptrSimGetJointType)(_getProcAddress(lib,"simGetJointType"));
	simBuildIdentityMatrix=(ptrSimBuildIdentityMatrix)(_getProcAddress(lib,"simBuildIdentityMatrix"));
	simCopyMatrix=(ptrSimCopyMatrix)(_getProcAddress(lib,"simCopyMatrix"));
	simBuildMatrix=(ptrSimBuildMatrix)(_getProcAddress(lib,"simBuildMatrix"));
	simGetEulerAnglesFromMatrix=(ptrSimGetEulerAnglesFromMatrix)(_getProcAddress(lib,"simGetEulerAnglesFromMatrix"));
	simInvertMatrix=(ptrSimInvertMatrix)(_getProcAddress(lib,"simInvertMatrix"));
	simMultiplyMatrices=(ptrSimMultiplyMatrices)(_getProcAddress(lib,"simMultiplyMatrices"));
	simInterpolateMatrices=(ptrSimInterpolateMatrices)(_getProcAddress(lib,"simInterpolateMatrices"));
	simTransformVector=(ptrSimTransformVector)(_getProcAddress(lib,"simTransformVector"));
	simReservedCommand=(ptrSimReservedCommand)(_getProcAddress(lib,"simReservedCommand"));
	simGetSimulationTime=(ptrSimGetSimulationTime)(_getProcAddress(lib,"simGetSimulationTime"));
	simGetSimulationState=(ptrSimGetSimulationState)(_getProcAddress(lib,"simGetSimulationState"));
	simGetSystemTime=(ptrSimGetSystemTime)(_getProcAddress(lib,"simGetSystemTime"));
	simGetSystemTimeInMilliseconds=(ptrSimGetSystemTimeInMilliseconds)(_getProcAddress(lib,"simGetSystemTimeInMilliseconds"));
	simGetSystemTimeInMs=(ptrSimGetSystemTimeInMs)(_getProcAddress(lib,"simGetSystemTimeInMs"));
	simLoadScene=(ptrSimLoadScene)(_getProcAddress(lib,"simLoadScene"));
	simCloseScene=(ptrSimCloseScene)(_getProcAddress(lib,"simCloseScene"));
	simSaveScene=(ptrSimSaveScene)(_getProcAddress(lib,"simSaveScene"));
	simLoadModel=(ptrSimLoadModel)(_getProcAddress(lib,"simLoadModel"));
	simSaveModel=(ptrSimSaveModel)(_getProcAddress(lib,"simSaveModel"));
	simLoadUI=(ptrSimLoadUI)(_getProcAddress(lib,"simLoadUI"));
	simSaveUI=(ptrSimSaveUI)(_getProcAddress(lib,"simSaveUI"));
	simAddStatusbarMessage=(ptrSimAddStatusbarMessage)(_getProcAddress(lib,"simAddStatusbarMessage"));
	simAddModuleMenuEntry=(ptrSimAddModuleMenuEntry)(_getProcAddress(lib,"simAddModuleMenuEntry"));
	simSetModuleMenuItemState=(ptrSimSetModuleMenuItemState)(_getProcAddress(lib,"simSetModuleMenuItemState"));
	simDoesFileExist=(ptrSimDoesFileExist)(_getProcAddress(lib,"simDoesFileExist"));
	simIsObjectInSelection=(ptrSimIsObjectInSelection)(_getProcAddress(lib,"simIsObjectInSelection"));
	simAddObjectToSelection=(ptrSimAddObjectToSelection)(_getProcAddress(lib,"simAddObjectToSelection"));
	simRemoveObjectFromSelection=(ptrSimRemoveObjectFromSelection)(_getProcAddress(lib,"simRemoveObjectFromSelection"));
	simGetObjectSelectionSize=(ptrSimGetObjectSelectionSize)(_getProcAddress(lib,"simGetObjectSelectionSize"));
	simGetObjectLastSelection=(ptrSimGetObjectLastSelection)(_getProcAddress(lib,"simGetObjectLastSelection"));
	simGetObjectSelection=(ptrSimGetObjectSelection)(_getProcAddress(lib,"simGetObjectSelection"));
	simSearchPath=(ptrSimSearchPath)(_getProcAddress(lib,"simSearchPath"));
	simInitializePathSearch=(ptrSimInitializePathSearch)(_getProcAddress(lib,"simInitializePathSearch"));
	simPerformPathSearchStep=(ptrSimPerformPathSearchStep)(_getProcAddress(lib,"simPerformPathSearchStep"));
	simHandleCollision=(ptrSimHandleCollision)(_getProcAddress(lib,"simHandleCollision"));
	simReadCollision=(ptrSimReadCollision)(_getProcAddress(lib,"simReadCollision"));
	simHandleDistance=(ptrSimHandleDistance)(_getProcAddress(lib,"simHandleDistance"));
	simReadDistance=(ptrSimReadDistance)(_getProcAddress(lib,"simReadDistance"));
	simHandleProximitySensor=(ptrSimHandleProximitySensor)(_getProcAddress(lib,"simHandleProximitySensor"));
	simReadProximitySensor=(ptrSimReadProximitySensor)(_getProcAddress(lib,"simReadProximitySensor"));
	simHandleMill=(ptrSimHandleMill)(_getProcAddress(lib,"simHandleMill"));
	simHandleIkGroup=(ptrSimHandleIkGroup)(_getProcAddress(lib,"simHandleIkGroup"));
	simCheckIkGroup=(ptrSimCheckIkGroup)(_getProcAddress(lib,"simCheckIkGroup"));
	simHandleDynamics=(ptrSimHandleDynamics)(_getProcAddress(lib,"simHandleDynamics"));
	simGetMechanismHandle=(ptrSimGetMechanismHandle)(_getProcAddress(lib,"simGetMechanismHandle"));
	simGetPathPlanningHandle=(ptrSimGetPathPlanningHandle)(_getProcAddress(lib,"simGetPathPlanningHandle"));
	simHandleMechanism=(ptrSimHandleMechanism)(_getProcAddress(lib,"simHandleMechanism"));
	simGetScriptHandle=(ptrSimGetScriptHandle)(_getProcAddress(lib,"simGetScriptHandle"));
	simSetScriptText=(ptrSimSetScriptText)(_getProcAddress(lib,"simSetScriptText"));
	simGetScriptText=(ptrSimGetScriptText)(_getProcAddress(lib,"simGetScriptText"));
	simGetScriptProperty=(ptrSimGetScriptProperty)(_getProcAddress(lib,"simGetScriptProperty"));
	simAssociateScriptWithObject=(ptrSimAssociateScriptWithObject)(_getProcAddress(lib,"simAssociateScriptWithObject"));
	simGetScript=(ptrSimGetScript)(_getProcAddress(lib,"simGetScript"));
	simGetScriptAssociatedWithObject=(ptrSimGetScriptAssociatedWithObject)(_getProcAddress(lib,"simGetScriptAssociatedWithObject"));
	simGetCustomizationScriptAssociatedWithObject=(ptrSimGetCustomizationScriptAssociatedWithObject)(_getProcAddress(lib,"simGetCustomizationScriptAssociatedWithObject"));
	simGetObjectAssociatedWithScript=(ptrSimGetObjectAssociatedWithScript)(_getProcAddress(lib,"simGetObjectAssociatedWithScript"));
	simGetScriptName=(ptrSimGetScriptName)(_getProcAddress(lib,"simGetScriptName"));
	simHandleMainScript=(ptrSimHandleMainScript)(_getProcAddress(lib,"simHandleMainScript"));
	simHandleGeneralCallbackScript=(ptrSimHandleGeneralCallbackScript)(_getProcAddress(lib,"simHandleGeneralCallbackScript"));
	simResetScript=(ptrSimResetScript)(_getProcAddress(lib,"simResetScript"));
	simAddScript=(ptrSimAddScript)(_getProcAddress(lib,"simAddScript"));
	simRemoveScript=(ptrSimRemoveScript)(_getProcAddress(lib,"simRemoveScript"));
	simRefreshDialogs=(ptrSimRefreshDialogs)(_getProcAddress(lib,"simRefreshDialogs"));
	simGetCollisionHandle=(ptrSimGetCollisionHandle)(_getProcAddress(lib,"simGetCollisionHandle"));
	simGetDistanceHandle=(ptrSimGetDistanceHandle)(_getProcAddress(lib,"simGetDistanceHandle"));
	simGetIkGroupHandle=(ptrSimGetIkGroupHandle)(_getProcAddress(lib,"simGetIkGroupHandle"));
	simResetCollision=(ptrSimResetCollision)(_getProcAddress(lib,"simResetCollision"));
	simResetDistance=(ptrSimResetDistance)(_getProcAddress(lib,"simResetDistance"));
	simResetProximitySensor=(ptrSimResetProximitySensor)(_getProcAddress(lib,"simResetProximitySensor"));
	simResetMill=(ptrSimResetMill)(_getProcAddress(lib,"simResetMill"));
	simCheckProximitySensor=(ptrSimCheckProximitySensor)(_getProcAddress(lib,"simCheckProximitySensor"));
	simCheckProximitySensorEx=(ptrSimCheckProximitySensorEx)(_getProcAddress(lib,"simCheckProximitySensorEx"));
	simCheckProximitySensorEx2=(ptrSimCheckProximitySensorEx2)(_getProcAddress(lib,"simCheckProximitySensorEx2"));
	simAddSceneCustomData=(ptrSimAddSceneCustomData)(_getProcAddress(lib,"simAddSceneCustomData"));
	simGetSceneCustomDataLength=(ptrSimGetSceneCustomDataLength)(_getProcAddress(lib,"simGetSceneCustomDataLength"));
	simGetSceneCustomData=(ptrSimGetSceneCustomData)(_getProcAddress(lib,"simGetSceneCustomData"));
	simAddObjectCustomData=(ptrSimAddObjectCustomData)(_getProcAddress(lib,"simAddObjectCustomData"));
	simGetObjectCustomDataLength=(ptrSimGetObjectCustomDataLength)(_getProcAddress(lib,"simGetObjectCustomDataLength"));
	simGetObjectCustomData=(ptrSimGetObjectCustomData)(_getProcAddress(lib,"simGetObjectCustomData"));
	simCreateBuffer=(ptrSimCreateBuffer)(_getProcAddress(lib,"simCreateBuffer"));
	simReleaseBuffer=(ptrSimReleaseBuffer)(_getProcAddress(lib,"simReleaseBuffer"));
	simCheckCollision=(ptrSimCheckCollision)(_getProcAddress(lib,"simCheckCollision"));
	simCheckCollisionEx=(ptrSimCheckCollisionEx)(_getProcAddress(lib,"simCheckCollisionEx"));
	simCheckDistance=(ptrSimCheckDistance)(_getProcAddress(lib,"simCheckDistance"));
	simGetObjectConfiguration=(ptrSimGetObjectConfiguration)(_getProcAddress(lib,"simGetObjectConfiguration"));
	simSetObjectConfiguration=(ptrSimSetObjectConfiguration)(_getProcAddress(lib,"simSetObjectConfiguration"));
	simGetConfigurationTree=(ptrSimGetConfigurationTree)(_getProcAddress(lib,"simGetConfigurationTree"));
	simSetConfigurationTree=(ptrSimSetConfigurationTree)(_getProcAddress(lib,"simSetConfigurationTree"));
	simSetSimulationTimeStep=(ptrSimSetSimulationTimeStep)(_getProcAddress(lib,"simSetSimulationTimeStep"));
	simGetSimulationTimeStep=(ptrSimGetSimulationTimeStep)(_getProcAddress(lib,"simGetSimulationTimeStep"));
	simGetRealTimeSimulation=(ptrSimGetRealTimeSimulation)(_getProcAddress(lib,"simGetRealTimeSimulation"));
	simIsRealTimeSimulationStepNeeded=(ptrSimIsRealTimeSimulationStepNeeded)(_getProcAddress(lib,"simIsRealTimeSimulationStepNeeded"));
	simAdjustRealTimeTimer=(ptrSimAdjustRealTimeTimer)(_getProcAddress(lib,"simAdjustRealTimeTimer"));
	simGetSimulationPassesPerRenderingPass=(ptrSimGetSimulationPassesPerRenderingPass)(_getProcAddress(lib,"simGetSimulationPassesPerRenderingPass"));
	simAdvanceSimulationByOneStep=(ptrSimAdvanceSimulationByOneStep)(_getProcAddress(lib,"simAdvanceSimulationByOneStep"));
	simStartSimulation=(ptrSimStartSimulation)(_getProcAddress(lib,"simStartSimulation"));
	simStopSimulation=(ptrSimStopSimulation)(_getProcAddress(lib,"simStopSimulation"));
	simPauseSimulation=(ptrSimPauseSimulation)(_getProcAddress(lib,"simPauseSimulation"));
	simBroadcastMessage=(ptrSimBroadcastMessage)(_getProcAddress(lib,"simBroadcastMessage"));
	simGetModuleName=(ptrSimGetModuleName)(_getProcAddress(lib,"simGetModuleName"));
	simGetScriptSimulationParameter=(ptrSimGetScriptSimulationParameter)(_getProcAddress(lib,"simGetScriptSimulationParameter"));
	simSetScriptSimulationParameter=(ptrSimSetScriptSimulationParameter)(_getProcAddress(lib,"simSetScriptSimulationParameter"));
	simFloatingViewAdd=(ptrSimFloatingViewAdd)(_getProcAddress(lib,"simFloatingViewAdd"));
	simFloatingViewRemove=(ptrSimFloatingViewRemove)(_getProcAddress(lib,"simFloatingViewRemove"));
	simAdjustView=(ptrSimAdjustView)(_getProcAddress(lib,"simAdjustView"));
	simResetPath=(ptrSimResetPath)(_getProcAddress(lib,"simResetPath"));
	simHandlePath=(ptrSimHandlePath)(_getProcAddress(lib,"simHandlePath"));
	simResetJoint=(ptrSimResetJoint)(_getProcAddress(lib,"simResetJoint"));
	simHandleJoint=(ptrSimHandleJoint)(_getProcAddress(lib,"simHandleJoint"));
	simSetLastError=(ptrSimSetLastError)(_getProcAddress(lib,"simSetLastError"));
	simHandleGraph=(ptrSimHandleGraph)(_getProcAddress(lib,"simHandleGraph"));
	simResetGraph=(ptrSimResetGraph)(_getProcAddress(lib,"simResetGraph"));
	simSetNavigationMode=(ptrSimSetNavigationMode)(_getProcAddress(lib,"simSetNavigationMode"));
	simGetNavigationMode=(ptrSimGetNavigationMode)(_getProcAddress(lib,"simGetNavigationMode"));
	simSetPage=(ptrSimSetPage)(_getProcAddress(lib,"simSetPage"));
	simGetPage=(ptrSimGetPage)(_getProcAddress(lib,"simGetPage"));
	simDisplayDialog=(ptrSimDisplayDialog)(_getProcAddress(lib,"simDisplayDialog"));
	simGetDialogResult=(ptrSimGetDialogResult)(_getProcAddress(lib,"simGetDialogResult"));
	simGetDialogInput=(ptrSimGetDialogInput)(_getProcAddress(lib,"simGetDialogInput"));
	simEndDialog=(ptrSimEndDialog)(_getProcAddress(lib,"simEndDialog"));
	simRegisterCustomLuaFunction=(ptrSimRegisterCustomLuaFunction)(_getProcAddress(lib,"simRegisterCustomLuaFunction"));
	simRegisterCustomLuaVariable=(ptrSimRegisterCustomLuaVariable)(_getProcAddress(lib,"simRegisterCustomLuaVariable"));
	simSetJointTargetVelocity=(ptrSimSetJointTargetVelocity)(_getProcAddress(lib,"simSetJointTargetVelocity"));
	simGetJointTargetVelocity=(ptrSimGetJointTargetVelocity)(_getProcAddress(lib,"simGetJointTargetVelocity"));
	simSetPathTargetNominalVelocity=(ptrSimSetPathTargetNominalVelocity)(_getProcAddress(lib,"simSetPathTargetNominalVelocity"));
	simLockInterface=(ptrSimLockInterface)(_getProcAddress(lib,"simLockInterface"));
	simGetScriptRawBuffer=(ptrSimGetScriptRawBuffer)(_getProcAddress(lib,"simGetScriptRawBuffer"));
	simSetScriptRawBuffer=(ptrSimSetScriptRawBuffer)(_getProcAddress(lib,"simSetScriptRawBuffer"));
	simReleaseScriptRawBuffer=(ptrSimReleaseScriptRawBuffer)(_getProcAddress(lib,"simReleaseScriptRawBuffer"));
	simCopyPasteSelectedObjects=(ptrSimCopyPasteSelectedObjects)(_getProcAddress(lib,"simCopyPasteSelectedObjects"));
	simCopyPasteObjects=(ptrSimCopyPasteObjects)(_getProcAddress(lib,"simCopyPasteObjects"));
	simScaleSelectedObjects=(ptrSimScaleSelectedObjects)(_getProcAddress(lib,"simScaleSelectedObjects"));
	simScaleObjects=(ptrSimScaleObjects)(_getProcAddress(lib,"simScaleObjects"));
	simDeleteSelectedObjects=(ptrSimDeleteSelectedObjects)(_getProcAddress(lib,"simDeleteSelectedObjects"));
	simGetObjectUniqueIdentifier=(ptrSimGetObjectUniqueIdentifier)(_getProcAddress(lib,"simGetObjectUniqueIdentifier"));
	simGetNameSuffix=(ptrSimGetNameSuffix)(_getProcAddress(lib,"simGetNameSuffix"));
	simSendData=(ptrSimSendData)(_getProcAddress(lib,"simSendData"));
	simReceiveData=(ptrSimReceiveData)(_getProcAddress(lib,"simReceiveData"));
	simSetGraphUserData=(ptrSimSetGraphUserData)(_getProcAddress(lib,"simSetGraphUserData"));
	simSetNameSuffix=(ptrSimSetNameSuffix)(_getProcAddress(lib,"simSetNameSuffix"));
	simAddDrawingObject=(ptrSimAddDrawingObject)(_getProcAddress(lib,"simAddDrawingObject"));
	simRemoveDrawingObject=(ptrSimRemoveDrawingObject)(_getProcAddress(lib,"simRemoveDrawingObject"));
	simAddDrawingObjectItem=(ptrSimAddDrawingObjectItem)(_getProcAddress(lib,"simAddDrawingObjectItem"));
	simAddParticleObject=(ptrSimAddParticleObject)(_getProcAddress(lib,"simAddParticleObject"));
	simRemoveParticleObject=(ptrSimRemoveParticleObject)(_getProcAddress(lib,"simRemoveParticleObject"));
	simAddParticleObjectItem=(ptrSimAddParticleObjectItem)(_getProcAddress(lib,"simAddParticleObjectItem"));
	simGetObjectSizeFactor=(ptrSimGetObjectSizeFactor)(_getProcAddress(lib,"simGetObjectSizeFactor"));
	simAnnounceSceneContentChange=(ptrSimAnnounceSceneContentChange)(_getProcAddress(lib,"simAnnounceSceneContentChange"));
	simResetMilling=(ptrSimResetMilling)(_getProcAddress(lib,"simResetMilling"));
	simApplyMilling=(ptrSimApplyMilling)(_getProcAddress(lib,"simApplyMilling"));
	simSetIntegerSignal=(ptrSimSetIntegerSignal)(_getProcAddress(lib,"simSetIntegerSignal"));
	simGetIntegerSignal=(ptrSimGetIntegerSignal)(_getProcAddress(lib,"simGetIntegerSignal"));
	simClearIntegerSignal=(ptrSimClearIntegerSignal)(_getProcAddress(lib,"simClearIntegerSignal"));
	simSetFloatSignal=(ptrSimSetFloatSignal)(_getProcAddress(lib,"simSetFloatSignal"));
	simGetFloatSignal=(ptrSimGetFloatSignal)(_getProcAddress(lib,"simGetFloatSignal"));
	simClearFloatSignal=(ptrSimClearFloatSignal)(_getProcAddress(lib,"simClearFloatSignal"));
	simSetStringSignal=(ptrSimSetStringSignal)(_getProcAddress(lib,"simSetStringSignal"));
	simGetStringSignal=(ptrSimGetStringSignal)(_getProcAddress(lib,"simGetStringSignal"));
	simClearStringSignal=(ptrSimClearStringSignal)(_getProcAddress(lib,"simClearStringSignal"));
	simGetSignalName=(ptrSimGetSignalName)(_getProcAddress(lib,"simGetSignalName"));
	simSetObjectProperty=(ptrSimSetObjectProperty)(_getProcAddress(lib,"simSetObjectProperty"));
	simGetObjectProperty=(ptrSimGetObjectProperty)(_getProcAddress(lib,"simGetObjectProperty"));
	simSetObjectSpecialProperty=(ptrSimSetObjectSpecialProperty)(_getProcAddress(lib,"simSetObjectSpecialProperty"));
	simGetObjectSpecialProperty=(ptrSimGetObjectSpecialProperty)(_getProcAddress(lib,"simGetObjectSpecialProperty"));
	simGetPositionOnPath=(ptrSimGetPositionOnPath)(_getProcAddress(lib,"simGetPositionOnPath"));
	simGetDataOnPath=(ptrSimGetDataOnPath)(_getProcAddress(lib,"simGetDataOnPath"));
	simGetOrientationOnPath=(ptrSimGetOrientationOnPath)(_getProcAddress(lib,"simGetOrientationOnPath"));
	simGetClosestPositionOnPath=(ptrSimGetClosestPositionOnPath)(_getProcAddress(lib,"simGetClosestPositionOnPath"));
	simReadForceSensor=(ptrSimReadForceSensor)(_getProcAddress(lib,"simReadForceSensor"));
	simBreakForceSensor=(ptrSimBreakForceSensor)(_getProcAddress(lib,"simBreakForceSensor"));
	simGetShapeVertex=(ptrSimGetShapeVertex)(_getProcAddress(lib,"simGetShapeVertex"));
	simGetShapeTriangle=(ptrSimGetShapeTriangle)(_getProcAddress(lib,"simGetShapeTriangle"));
	simSetLightParameters=(ptrSimSetLightParameters)(_getProcAddress(lib,"simSetLightParameters"));
	simGetLightParameters=(ptrSimGetLightParameters)(_getProcAddress(lib,"simGetLightParameters"));
	simHandleVarious=(ptrSimHandleVarious)(_getProcAddress(lib,"simHandleVarious"));
	simGetVelocity=(ptrSimGetVelocity)(_getProcAddress(lib,"simGetVelocity"));
	simGetObjectVelocity=(ptrSimGetObjectVelocity)(_getProcAddress(lib,"simGetObjectVelocity"));
	simAddForceAndTorque=(ptrSimAddForceAndTorque)(_getProcAddress(lib,"simAddForceAndTorque"));
	simAddForce=(ptrSimAddForce)(_getProcAddress(lib,"simAddForce"));
	simSetExplicitHandling=(ptrSimSetExplicitHandling)(_getProcAddress(lib,"simSetExplicitHandling"));
	simGetExplicitHandling=(ptrSimGetExplicitHandling)(_getProcAddress(lib,"simGetExplicitHandling"));
	simGetLinkDummy=(ptrSimGetLinkDummy)(_getProcAddress(lib,"simGetLinkDummy"));
	simSetLinkDummy=(ptrSimSetLinkDummy)(_getProcAddress(lib,"simSetLinkDummy"));
	simSetModelProperty=(ptrSimSetModelProperty)(_getProcAddress(lib,"simSetModelProperty"));
	simGetModelProperty=(ptrSimGetModelProperty)(_getProcAddress(lib,"simGetModelProperty"));
	simSetShapeColor=(ptrSimSetShapeColor)(_getProcAddress(lib,"simSetShapeColor"));
	simGetShapeColor=(ptrSimGetShapeColor)(_getProcAddress(lib,"simGetShapeColor"));
	simResetDynamicObject=(ptrSimResetDynamicObject)(_getProcAddress(lib,"simResetDynamicObject"));
	simSetJointMode=(ptrSimSetJointMode)(_getProcAddress(lib,"simSetJointMode"));
	simGetJointMode=(ptrSimGetJointMode)(_getProcAddress(lib,"simGetJointMode"));
	simSerialOpen=(ptrSimSerialOpen)(_getProcAddress(lib,"simSerialOpen"));
	simSerialClose=(ptrSimSerialClose)(_getProcAddress(lib,"simSerialClose"));
	simSerialSend=(ptrSimSerialSend)(_getProcAddress(lib,"simSerialSend"));
	simSerialRead=(ptrSimSerialRead)(_getProcAddress(lib,"simSerialRead"));
	simSerialCheck=(ptrSimSerialCheck)(_getProcAddress(lib,"simSerialCheck"));
	simSerialPortOpen=(ptrSimSerialPortOpen)(_getProcAddress(lib,"simSerialPortOpen"));
	simSerialPortClose=(ptrSimSerialPortClose)(_getProcAddress(lib,"simSerialPortClose"));
	simSerialPortSend=(ptrSimSerialPortSend)(_getProcAddress(lib,"simSerialPortSend"));
	simSerialPortRead=(ptrSimSerialPortRead)(_getProcAddress(lib,"simSerialPortRead"));
	simGetContactInfo=(ptrSimGetContactInfo)(_getProcAddress(lib,"simGetContactInfo"));
	simSetThreadIsFree=(ptrSimSetThreadIsFree)(_getProcAddress(lib,"simSetThreadIsFree"));
	simTubeOpen=(ptrSimTubeOpen)(_getProcAddress(lib,"simTubeOpen"));
	simTubeClose=(ptrSimTubeClose)(_getProcAddress(lib,"simTubeClose"));
	simTubeWrite=(ptrSimTubeWrite)(_getProcAddress(lib,"simTubeWrite"));
	simTubeRead=(ptrSimTubeRead)(_getProcAddress(lib,"simTubeRead"));
	simTubeStatus=(ptrSimTubeStatus)(_getProcAddress(lib,"simTubeStatus"));
	simAuxiliaryConsoleOpen=(ptrSimAuxiliaryConsoleOpen)(_getProcAddress(lib,"simAuxiliaryConsoleOpen"));
	simAuxiliaryConsoleClose=(ptrSimAuxiliaryConsoleClose)(_getProcAddress(lib,"simAuxiliaryConsoleClose"));
	simAuxiliaryConsoleShow=(ptrSimAuxiliaryConsoleShow)(_getProcAddress(lib,"simAuxiliaryConsoleShow"));
	simAuxiliaryConsolePrint=(ptrSimAuxiliaryConsolePrint)(_getProcAddress(lib,"simAuxiliaryConsolePrint"));
	simImportShape=(ptrSimImportShape)(_getProcAddress(lib,"simImportShape"));
	simImportMesh=(ptrSimImportMesh)(_getProcAddress(lib,"simImportMesh"));
	simExportMesh=(ptrSimExportMesh)(_getProcAddress(lib,"simExportMesh"));
	simCreateMeshShape=(ptrSimCreateMeshShape)(_getProcAddress(lib,"simCreateMeshShape"));
	simCreatePureShape=(ptrSimCreatePureShape)(_getProcAddress(lib,"simCreatePureShape"));
	simCreateHeightfieldShape=(ptrSimCreateHeightfieldShape)(_getProcAddress(lib,"simCreateHeightfieldShape"));
	simGetShapeMesh=(ptrSimGetShapeMesh)(_getProcAddress(lib,"simGetShapeMesh"));
	simAddBanner=(ptrSimAddBanner)(_getProcAddress(lib,"simAddBanner"));
	simRemoveBanner=(ptrSimRemoveBanner)(_getProcAddress(lib,"simRemoveBanner"));
	simCreateJoint=(ptrSimCreateJoint)(_getProcAddress(lib,"simCreateJoint"));
	simCreateDummy=(ptrSimCreateDummy)(_getProcAddress(lib,"simCreateDummy"));
	simCreateProximitySensor=(ptrSimCreateProximitySensor)(_getProcAddress(lib,"simCreateProximitySensor"));
	simCreatePath=(ptrSimCreatePath)(_getProcAddress(lib,"simCreatePath"));
	simInsertPathCtrlPoints=(ptrSimInsertPathCtrlPoints)(_getProcAddress(lib,"simInsertPathCtrlPoints"));
	simCutPathCtrlPoints=(ptrSimCutPathCtrlPoints)(_getProcAddress(lib,"simCutPathCtrlPoints"));
	simCreateForceSensor=(ptrSimCreateForceSensor)(_getProcAddress(lib,"simCreateForceSensor"));
	simCreateVisionSensor=(ptrSimCreateVisionSensor)(_getProcAddress(lib,"simCreateVisionSensor"));
	simRegisterContactCallback=(ptrSimRegisterContactCallback)(_getProcAddress(lib,"simRegisterContactCallback"));
	simGetObjectIntParameter=(ptrSimGetObjectIntParameter)(_getProcAddress(lib,"simGetObjectIntParameter"));
	simSetObjectIntParameter=(ptrSimSetObjectIntParameter)(_getProcAddress(lib,"simSetObjectIntParameter"));
	simGetObjectFloatParameter=(ptrSimGetObjectFloatParameter)(_getProcAddress(lib,"simGetObjectFloatParameter"));
	simSetObjectFloatParameter=(ptrSimSetObjectFloatParameter)(_getProcAddress(lib,"simSetObjectFloatParameter"));
	simGetObjectStringParameter=(ptrSimGetObjectStringParameter)(_getProcAddress(lib,"simGetObjectStringParameter"));
	simSetObjectStringParameter=(ptrSimSetObjectStringParameter)(_getProcAddress(lib,"simSetObjectStringParameter"));
	simSetSimulationPassesPerRenderingPass=(ptrSimSetSimulationPassesPerRenderingPass)(_getProcAddress(lib,"simSetSimulationPassesPerRenderingPass"));
	simGetRotationAxis=(ptrSimGetRotationAxis)(_getProcAddress(lib,"simGetRotationAxis"));
	simRotateAroundAxis=(ptrSimRotateAroundAxis)(_getProcAddress(lib,"simRotateAroundAxis"));
	simJointGetForce=(ptrSimJointGetForce)(_getProcAddress(lib,"simJointGetForce"));
	simGetJointForce=(ptrSimGetJointForce)(_getProcAddress(lib,"simGetJointForce"));
	simSetArrayParameter=(ptrSimSetArrayParameter)(_getProcAddress(lib,"simSetArrayParameter"));
	simGetArrayParameter=(ptrSimGetArrayParameter)(_getProcAddress(lib,"simGetArrayParameter"));
	simSetIkGroupProperties=(ptrSimSetIkGroupProperties)(_getProcAddress(lib,"simSetIkGroupProperties"));
	simSetIkElementProperties=(ptrSimSetIkElementProperties)(_getProcAddress(lib,"simSetIkElementProperties"));
	simCameraFitToView=(ptrSimCameraFitToView)(_getProcAddress(lib,"simCameraFitToView"));
	simPersistentDataWrite=(ptrSimPersistentDataWrite)(_getProcAddress(lib,"simPersistentDataWrite"));
	simPersistentDataRead=(ptrSimPersistentDataRead)(_getProcAddress(lib,"simPersistentDataRead"));
	simIsHandleValid=(ptrSimIsHandleValid)(_getProcAddress(lib,"simIsHandleValid"));
	simHandleVisionSensor=(ptrSimHandleVisionSensor)(_getProcAddress(lib,"simHandleVisionSensor"));
	simReadVisionSensor=(ptrSimReadVisionSensor)(_getProcAddress(lib,"simReadVisionSensor"));
	simResetVisionSensor=(ptrSimResetVisionSensor)(_getProcAddress(lib,"simResetVisionSensor"));
	simCheckVisionSensor=(ptrSimCheckVisionSensor)(_getProcAddress(lib,"simCheckVisionSensor"));
	simCheckVisionSensorEx=(ptrSimCheckVisionSensorEx)(_getProcAddress(lib,"simCheckVisionSensorEx"));
	simGetVisionSensorResolution=(ptrSimGetVisionSensorResolution)(_getProcAddress(lib,"simGetVisionSensorResolution"));
	simGetVisionSensorImage=(ptrSimGetVisionSensorImage)(_getProcAddress(lib,"simGetVisionSensorImage"));
	simGetVisionSensorCharImage=(ptrSimGetVisionSensorCharImage)(_getProcAddress(lib,"simGetVisionSensorCharImage"));
	simSetVisionSensorImage=(ptrSimSetVisionSensorImage)(_getProcAddress(lib,"simSetVisionSensorImage"));
	simSetVisionSensorCharImage=(ptrSimSetVisionSensorCharImage)(_getProcAddress(lib,"simSetVisionSensorCharImage"));
	simGetVisionSensorDepthBuffer=(ptrSimGetVisionSensorDepthBuffer)(_getProcAddress(lib,"simGetVisionSensorDepthBuffer"));
	simCreateUI=(ptrSimCreateUI)(_getProcAddress(lib,"simCreateUI"));
	simCreateUIButton=(ptrSimCreateUIButton)(_getProcAddress(lib,"simCreateUIButton"));
	simGetUIHandle=(ptrSimGetUIHandle)(_getProcAddress(lib,"simGetUIHandle"));
	simGetUIProperty=(ptrSimGetUIProperty)(_getProcAddress(lib,"simGetUIProperty"));
	simGetUIEventButton=(ptrSimGetUIEventButton)(_getProcAddress(lib,"simGetUIEventButton"));
	simSetUIProperty=(ptrSimSetUIProperty)(_getProcAddress(lib,"simSetUIProperty"));
	simGetUIButtonProperty=(ptrSimGetUIButtonProperty)(_getProcAddress(lib,"simGetUIButtonProperty"));
	simSetUIButtonProperty=(ptrSimSetUIButtonProperty)(_getProcAddress(lib,"simSetUIButtonProperty"));
	simGetUIButtonSize=(ptrSimGetUIButtonSize)(_getProcAddress(lib,"simGetUIButtonSize"));
	simSetUIButtonLabel=(ptrSimSetUIButtonLabel)(_getProcAddress(lib,"simSetUIButtonLabel"));
	simGetUIButtonLabel=(ptrSimGetUIButtonLabel)(_getProcAddress(lib,"simGetUIButtonLabel"));
	simSetUISlider=(ptrSimSetUISlider)(_getProcAddress(lib,"simSetUISlider"));
	simGetUISlider=(ptrSimGetUISlider)(_getProcAddress(lib,"simGetUISlider"));
	simSetUIButtonColor=(ptrSimSetUIButtonColor)(_getProcAddress(lib,"simSetUIButtonColor"));
	simSetUIButtonTexture=(ptrSimSetUIButtonTexture)(_getProcAddress(lib,"simSetUIButtonTexture"));
	simCreateUIButtonArray=(ptrSimCreateUIButtonArray)(_getProcAddress(lib,"simCreateUIButtonArray"));
	simSetUIButtonArrayColor=(ptrSimSetUIButtonArrayColor)(_getProcAddress(lib,"simSetUIButtonArrayColor"));
	simDeleteUIButtonArray=(ptrSimDeleteUIButtonArray)(_getProcAddress(lib,"simDeleteUIButtonArray"));
	simRemoveUI=(ptrSimRemoveUI)(_getProcAddress(lib,"simRemoveUI"));
	simSetUIPosition=(ptrSimSetUIPosition)(_getProcAddress(lib,"simSetUIPosition"));
	simGetUIPosition=(ptrSimGetUIPosition)(_getProcAddress(lib,"simGetUIPosition"));
	simGetObjectQuaternion=(ptrSimGetObjectQuaternion)(_getProcAddress(lib,"simGetObjectQuaternion"));
	simSetObjectQuaternion=(ptrSimSetObjectQuaternion)(_getProcAddress(lib,"simSetObjectQuaternion"));
	simRMLPosition=(ptrSimRMLPosition)(_getProcAddress(lib,"simRMLPosition"));
	simRMLVelocity=(ptrSimRMLVelocity)(_getProcAddress(lib,"simRMLVelocity"));
	simRMLPos=(ptrSimRMLPos)(_getProcAddress(lib,"simRMLPos"));
	simRMLVel=(ptrSimRMLVel)(_getProcAddress(lib,"simRMLVel"));
	simRMLStep=(ptrSimRMLStep)(_getProcAddress(lib,"simRMLStep"));
	simRMLRemove=(ptrSimRMLRemove)(_getProcAddress(lib,"simRMLRemove"));
	simBuildMatrixQ=(ptrSimBuildMatrixQ)(_getProcAddress(lib,"simBuildMatrixQ"));
	simGetQuaternionFromMatrix=(ptrSimGetQuaternionFromMatrix)(_getProcAddress(lib,"simGetQuaternionFromMatrix"));
	simFileDialog=(ptrSimFileDialog)(_getProcAddress(lib,"simFileDialog"));
	simMsgBox=(ptrSimMsgBox)(_getProcAddress(lib,"simMsgBox"));
	simSetShapeMassAndInertia=(ptrSimSetShapeMassAndInertia)(_getProcAddress(lib,"simSetShapeMassAndInertia"));
	simGetShapeMassAndInertia=(ptrSimGetShapeMassAndInertia)(_getProcAddress(lib,"simGetShapeMassAndInertia"));
	simGroupShapes=(ptrSimGroupShapes)(_getProcAddress(lib,"simGroupShapes"));
	simUngroupShape=(ptrSimUngroupShape)(_getProcAddress(lib,"simUngroupShape"));
	simConvexDecompose=(ptrSimConvexDecompose)(_getProcAddress(lib,"simConvexDecompose"));
	simGetIkGroupMatrix=(ptrSimGetIkGroupMatrix)(_getProcAddress(lib,"simGetIkGroupMatrix"));
	simGetMotionPlanningHandle=(ptrSimGetMotionPlanningHandle)(_getProcAddress(lib,"simGetMotionPlanningHandle"));
	simGetMpConfigForTipPose=(ptrSimGetMpConfigForTipPose)(_getProcAddress(lib,"simGetMpConfigForTipPose"));
	simFindMpPath=(ptrSimFindMpPath)(_getProcAddress(lib,"simFindMpPath"));
	simSimplifyMpPath=(ptrSimSimplifyMpPath)(_getProcAddress(lib,"simSimplifyMpPath"));
	simGetMpConfigTransition=(ptrSimGetMpConfigTransition)(_getProcAddress(lib,"simGetMpConfigTransition"));
	simAddGhost=(ptrSimAddGhost)(_getProcAddress(lib,"simAddGhost"));
	simModifyGhost=(ptrSimModifyGhost)(_getProcAddress(lib,"simModifyGhost"));
	simQuitSimulator=(ptrSimQuitSimulator)(_getProcAddress(lib,"simQuitSimulator"));
	simGetThreadId=(ptrSimGetThreadId)(_getProcAddress(lib,"simGetThreadId"));
	simLockResources=(ptrSimLockResources)(_getProcAddress(lib,"simLockResources"));
	simUnlockResources=(ptrSimUnlockResources)(_getProcAddress(lib,"simUnlockResources"));
	simEnableEventCallback=(ptrSimEnableEventCallback)(_getProcAddress(lib,"simEnableEventCallback"));
	simGetMaterialId=(ptrSimGetMaterialId)(_getProcAddress(lib,"simGetMaterialId"));
	simSetShapeMaterial=(ptrSimSetShapeMaterial)(_getProcAddress(lib,"simSetShapeMaterial"));
	simGetShapeMaterial=(ptrSimGetShapeMaterial)(_getProcAddress(lib,"simGetShapeMaterial"));
	simFindIkPath=(ptrSimFindIkPath)(_getProcAddress(lib,"simFindIkPath"));
	simGetTextureId=(ptrSimGetTextureId)(_getProcAddress(lib,"simGetTextureId"));
	simReadTexture=(ptrSimReadTexture)(_getProcAddress(lib,"simReadTexture"));
	simWriteTexture=(ptrSimWriteTexture)(_getProcAddress(lib,"simWriteTexture"));
	simCreateTexture=(ptrSimCreateTexture)(_getProcAddress(lib,"simCreateTexture"));
	simWriteCustomDataBlock=(ptrSimWriteCustomDataBlock)(_getProcAddress(lib,"simWriteCustomDataBlock"));
	simReadCustomDataBlock=(ptrSimReadCustomDataBlock)(_getProcAddress(lib,"simReadCustomDataBlock"));
	simAddPointCloud=(ptrSimAddPointCloud)(_getProcAddress(lib,"simAddPointCloud"));
	simModifyPointCloud=(ptrSimModifyPointCloud)(_getProcAddress(lib,"simModifyPointCloud"));
	simGetShapeGeomInfo=(ptrSimGetShapeGeomInfo)(_getProcAddress(lib,"simGetShapeGeomInfo"));
	simGetObjectsInTree=(ptrSimGetObjectsInTree)(_getProcAddress(lib,"simGetObjectsInTree"));
	simSetObjectSizeValues=(ptrSimSetObjectSizeValues)(_getProcAddress(lib,"simSetObjectSizeValues"));
	simGetObjectSizeValues=(ptrSimGetObjectSizeValues)(_getProcAddress(lib,"simGetObjectSizeValues"));
	simScaleObject=(ptrSimScaleObject)(_getProcAddress(lib,"simScaleObject"));
	simSetShapeTexture=(ptrSimSetShapeTexture)(_getProcAddress(lib,"simSetShapeTexture"));
	simGetShapeTextureId=(ptrSimGetShapeTextureId)(_getProcAddress(lib,"simGetShapeTextureId"));
	simGetCollectionObjects=(ptrSimGetCollectionObjects)(_getProcAddress(lib,"simGetCollectionObjects"));
	simHandleCustomizationScripts=(ptrSimHandleCustomizationScripts)(_getProcAddress(lib,"simHandleCustomizationScripts"));
	simSetScriptAttribute=(ptrSimSetScriptAttribute)(_getProcAddress(lib,"simSetScriptAttribute"));
	simGetScriptAttribute=(ptrSimGetScriptAttribute)(_getProcAddress(lib,"simGetScriptAttribute"));
	simReorientShapeBoundingBox=(ptrSimReorientShapeBoundingBox)(_getProcAddress(lib,"simReorientShapeBoundingBox"));
	simSwitchThread=(ptrSimSwitchThread)(_getProcAddress(lib,"simSwitchThread"));
	simCreateIkGroup=(ptrSimCreateIkGroup)(_getProcAddress(lib,"simCreateIkGroup"));
	simRemoveIkGroup=(ptrSimRemoveIkGroup)(_getProcAddress(lib,"simRemoveIkGroup"));
	simCreateIkElement=(ptrSimCreateIkElement)(_getProcAddress(lib,"simCreateIkElement"));
	simCreateMotionPlanning=(ptrSimCreateMotionPlanning)(_getProcAddress(lib,"simCreateMotionPlanning"));
	simRemoveMotionPlanning=(ptrSimRemoveMotionPlanning)(_getProcAddress(lib,"simRemoveMotionPlanning"));
	simCreateCollection=(ptrSimCreateCollection)(_getProcAddress(lib,"simCreateCollection"));
	simAddObjectToCollection=(ptrSimAddObjectToCollection)(_getProcAddress(lib,"simAddObjectToCollection"));
	simSaveImage=(ptrSimSaveImage)(_getProcAddress(lib,"simSaveImage"));


	_simGetContactCallbackCount=(ptr_simGetContactCallbackCount)(_getProcAddress(lib,"_simGetContactCallbackCount"));
	_simGetContactCallback=(ptr_simGetContactCallback)(_getProcAddress(lib,"_simGetContactCallback"));
	_simSetDynamicSimulationIconCode=(ptr_simSetDynamicSimulationIconCode)(_getProcAddress(lib,"_simSetDynamicSimulationIconCode"));
	_simSetDynamicObjectFlagForVisualization=(ptr_simSetDynamicObjectFlagForVisualization)(_getProcAddress(lib,"_simSetDynamicObjectFlagForVisualization"));
	_simGetObjectListSize=(ptr_simGetObjectListSize)(_getProcAddress(lib,"_simGetObjectListSize"));
	_simGetObjectFromIndex=(ptr_simGetObjectFromIndex)(_getProcAddress(lib,"_simGetObjectFromIndex"));
	_simGetObjectID=(ptr_simGetObjectID)(_getProcAddress(lib,"_simGetObjectID"));
	_simGetObjectType=(ptr_simGetObjectType)(_getProcAddress(lib,"_simGetObjectType"));
	_simGetObjectChildren=(ptr_simGetObjectChildren)(_getProcAddress(lib,"_simGetObjectChildren"));
	_simGetGeomProxyFromShape=(ptr_simGetGeomProxyFromShape)(_getProcAddress(lib,"_simGetGeomProxyFromShape"));
	_simGetParentObject=(ptr_simGetParentObject)(_getProcAddress(lib,"_simGetParentObject"));
	_simGetObject=(ptr_simGetObject)(_getProcAddress(lib,"_simGetObject"));
	_simGetIkGroupObject=(ptr_simGetIkGroupObject)(_getProcAddress(lib,"_simGetIkGroupObject"));
	_simMpHandleIkGroupObject=(ptr_simMpHandleIkGroupObject)(_getProcAddress(lib,"_simMpHandleIkGroupObject"));
	_simGetObjectLocalTransformation=(ptr_simGetObjectLocalTransformation)(_getProcAddress(lib,"_simGetObjectLocalTransformation"));
	_simSetObjectLocalTransformation=(ptr_simSetObjectLocalTransformation)(_getProcAddress(lib,"_simSetObjectLocalTransformation"));
	_simSetObjectCumulativeTransformation=(ptr_simSetObjectCumulativeTransformation)(_getProcAddress(lib,"_simSetObjectCumulativeTransformation"));
	_simGetObjectCumulativeTransformation=(ptr_simGetObjectCumulativeTransformation)(_getProcAddress(lib,"_simGetObjectCumulativeTransformation"));
	_simIsShapeDynamicallyStatic=(ptr_simIsShapeDynamicallyStatic)(_getProcAddress(lib,"_simIsShapeDynamicallyStatic"));
	_simGetTreeDynamicProperty=(ptr_simGetTreeDynamicProperty)(_getProcAddress(lib,"_simGetTreeDynamicProperty"));
	_simGetDummyLinkType=(ptr_simGetDummyLinkType)(_getProcAddress(lib,"_simGetDummyLinkType"));
	_simGetJointMode=(ptr_simGetJointMode)(_getProcAddress(lib,"_simGetJointMode"));
	_simIsJointInHybridOperation=(ptr_simIsJointInHybridOperation)(_getProcAddress(lib,"_simIsJointInHybridOperation"));
	_simDisableDynamicTreeForManipulation=(ptr_simDisableDynamicTreeForManipulation)(_getProcAddress(lib,"_simDisableDynamicTreeForManipulation"));
	_simIsShapeDynamicallyRespondable=(ptr_simIsShapeDynamicallyRespondable)(_getProcAddress(lib,"_simIsShapeDynamicallyRespondable"));
	_simGetDynamicCollisionMask=(ptr_simGetDynamicCollisionMask)(_getProcAddress(lib,"_simGetDynamicCollisionMask"));
	_simGetLastParentForLocalGlobalCollidable=(ptr_simGetLastParentForLocalGlobalCollidable)(_getProcAddress(lib,"_simGetLastParentForLocalGlobalCollidable"));
	_simSetShapeIsStaticAndNotRespondableButDynamicTag=(ptr_simSetShapeIsStaticAndNotRespondableButDynamicTag)(_getProcAddress(lib,"_simSetShapeIsStaticAndNotRespondableButDynamicTag"));
	_simGetShapeIsStaticAndNotRespondableButDynamicTag=(ptr_simGetShapeIsStaticAndNotRespondableButDynamicTag)(_getProcAddress(lib,"_simGetShapeIsStaticAndNotRespondableButDynamicTag"));
	_simSetJointPosition=(ptr_simSetJointPosition)(_getProcAddress(lib,"_simSetJointPosition"));
	_simGetJointPosition=(ptr_simGetJointPosition)(_getProcAddress(lib,"_simGetJointPosition"));
	_simSetDynamicMotorPositionControlTargetPosition=(ptr_simSetDynamicMotorPositionControlTargetPosition)(_getProcAddress(lib,"_simSetDynamicMotorPositionControlTargetPosition"));
	_simGetInitialDynamicVelocity=(ptr_simGetInitialDynamicVelocity)(_getProcAddress(lib,"_simGetInitialDynamicVelocity"));
	_simSetInitialDynamicVelocity=(ptr_simSetInitialDynamicVelocity)(_getProcAddress(lib,"_simSetInitialDynamicVelocity"));
	_simGetInitialDynamicAngVelocity=(ptr_simGetInitialDynamicAngVelocity)(_getProcAddress(lib,"_simGetInitialDynamicAngVelocity"));
	_simSetInitialDynamicAngVelocity=(ptr_simSetInitialDynamicAngVelocity)(_getProcAddress(lib,"_simSetInitialDynamicAngVelocity"));
	_simGetStartSleeping=(ptr_simGetStartSleeping)(_getProcAddress(lib,"_simGetStartSleeping"));
	_simGetWasPutToSleepOnce=(ptr_simGetWasPutToSleepOnce)(_getProcAddress(lib,"_simGetWasPutToSleepOnce"));
	_simGetDynamicsFullRefreshFlag=(ptr_simGetDynamicsFullRefreshFlag)(_getProcAddress(lib,"_simGetDynamicsFullRefreshFlag"));
	_simSetDynamicsFullRefreshFlag=(ptr_simSetDynamicsFullRefreshFlag)(_getProcAddress(lib,"_simSetDynamicsFullRefreshFlag"));
	_simSetGeomProxyDynamicsFullRefreshFlag=(ptr_simSetGeomProxyDynamicsFullRefreshFlag)(_getProcAddress(lib,"_simSetGeomProxyDynamicsFullRefreshFlag"));
	_simGetGeomProxyDynamicsFullRefreshFlag=(ptr_simGetGeomProxyDynamicsFullRefreshFlag)(_getProcAddress(lib,"_simGetGeomProxyDynamicsFullRefreshFlag"));
	_simGetParentFollowsDynamic=(ptr_simGetParentFollowsDynamic)(_getProcAddress(lib,"_simGetParentFollowsDynamic"));
	_simSetShapeDynamicVelocity=(ptr_simSetShapeDynamicVelocity)(_getProcAddress(lib,"_simSetShapeDynamicVelocity"));
	_simGetAdditionalForceAndTorque=(ptr_simGetAdditionalForceAndTorque)(_getProcAddress(lib,"_simGetAdditionalForceAndTorque"));
	_simClearAdditionalForceAndTorque=(ptr_simClearAdditionalForceAndTorque)(_getProcAddress(lib,"_simClearAdditionalForceAndTorque"));
	_simGetJointPositionInterval=(ptr_simGetJointPositionInterval)(_getProcAddress(lib,"_simGetJointPositionInterval"));
	_simGetJointType=(ptr_simGetJointType)(_getProcAddress(lib,"_simGetJointType"));
	_simGetJointOdeParameters=(ptr_simGetJointOdeParameters)(_getProcAddress(lib,"_simGetJointOdeParameters"));
	_simGetJointBulletParameters=(ptr_simGetJointBulletParameters)(_getProcAddress(lib,"_simGetJointBulletParameters"));
	_simIsForceSensorBroken=(ptr_simIsForceSensorBroken)(_getProcAddress(lib,"_simIsForceSensorBroken"));
	_simGetDynamicForceSensorLocalTransformationPart2=(ptr_simGetDynamicForceSensorLocalTransformationPart2)(_getProcAddress(lib,"_simGetDynamicForceSensorLocalTransformationPart2"));
	_simIsDynamicMotorEnabled=(ptr_simIsDynamicMotorEnabled)(_getProcAddress(lib,"_simIsDynamicMotorEnabled"));
	_simIsDynamicMotorPositionCtrlEnabled=(ptr_simIsDynamicMotorPositionCtrlEnabled)(_getProcAddress(lib,"_simIsDynamicMotorPositionCtrlEnabled"));
	_simIsDynamicMotorTorqueModulationEnabled=(ptr_simIsDynamicMotorTorqueModulationEnabled)(_getProcAddress(lib,"_simIsDynamicMotorTorqueModulationEnabled"));
	_simGetMotorPid=(ptr_simGetMotorPid)(_getProcAddress(lib,"_simGetMotorPid"));
	_simGetDynamicMotorTargetPosition=(ptr_simGetDynamicMotorTargetPosition)(_getProcAddress(lib,"_simGetDynamicMotorTargetPosition"));
	_simGetDynamicMotorTargetVelocity=(ptr_simGetDynamicMotorTargetVelocity)(_getProcAddress(lib,"_simGetDynamicMotorTargetVelocity"));
	_simGetDynamicMotorMaxForce=(ptr_simGetDynamicMotorMaxForce)(_getProcAddress(lib,"_simGetDynamicMotorMaxForce"));
	_simGetDynamicMotorUpperLimitVelocity=(ptr_simGetDynamicMotorUpperLimitVelocity)(_getProcAddress(lib,"_simGetDynamicMotorUpperLimitVelocity"));
	_simSetDynamicMotorReflectedPositionFromDynamicEngine=(ptr_simSetDynamicMotorReflectedPositionFromDynamicEngine)(_getProcAddress(lib,"_simSetDynamicMotorReflectedPositionFromDynamicEngine"));
	_simSetJointSphericalTransformation=(ptr_simSetJointSphericalTransformation)(_getProcAddress(lib,"_simSetJointSphericalTransformation"));
	_simAddForceSensorCumulativeForcesAndTorques=(ptr_simAddForceSensorCumulativeForcesAndTorques)(_getProcAddress(lib,"_simAddForceSensorCumulativeForcesAndTorques"));
	_simAddJointCumulativeForcesOrTorques=(ptr_simAddJointCumulativeForcesOrTorques)(_getProcAddress(lib,"_simAddJointCumulativeForcesOrTorques"));
	_simSetDynamicJointLocalTransformationPart2=(ptr_simSetDynamicJointLocalTransformationPart2)(_getProcAddress(lib,"_simSetDynamicJointLocalTransformationPart2"));
	_simSetDynamicForceSensorLocalTransformationPart2=(ptr_simSetDynamicForceSensorLocalTransformationPart2)(_getProcAddress(lib,"_simSetDynamicForceSensorLocalTransformationPart2"));
	_simSetDynamicJointLocalTransformationPart2IsValid=(ptr_simSetDynamicJointLocalTransformationPart2IsValid)(_getProcAddress(lib,"_simSetDynamicJointLocalTransformationPart2IsValid"));
	_simSetDynamicForceSensorLocalTransformationPart2IsValid=(ptr_simSetDynamicForceSensorLocalTransformationPart2IsValid)(_getProcAddress(lib,"_simSetDynamicForceSensorLocalTransformationPart2IsValid"));
	_simGetGeomWrapFromGeomProxy=(ptr_simGetGeomWrapFromGeomProxy)(_getProcAddress(lib,"_simGetGeomWrapFromGeomProxy"));
	_simGetLocalInertiaFrame=(ptr_simGetLocalInertiaFrame)(_getProcAddress(lib,"_simGetLocalInertiaFrame"));
	_simGetPurePrimitiveType=(ptr_simGetPurePrimitiveType)(_getProcAddress(lib,"_simGetPurePrimitiveType"));
	_simIsGeomWrapGeometric=(ptr_simIsGeomWrapGeometric)(_getProcAddress(lib,"_simIsGeomWrapGeometric"));
	_simIsGeomWrapConvex=(ptr_simIsGeomWrapConvex)(_getProcAddress(lib,"_simIsGeomWrapConvex"));
	_simGetGeometricCount=(ptr_simGetGeometricCount)(_getProcAddress(lib,"_simGetGeometricCount"));
	_simGetAllGeometrics=(ptr_simGetAllGeometrics)(_getProcAddress(lib,"_simGetAllGeometrics"));
	_simGetPurePrimitiveSizes=(ptr_simGetPurePrimitiveSizes)(_getProcAddress(lib,"_simGetPurePrimitiveSizes"));
	_simMakeDynamicAnnouncement=(ptr_simMakeDynamicAnnouncement)(_getProcAddress(lib,"_simMakeDynamicAnnouncement"));
	_simGetVerticesLocalFrame=(ptr_simGetVerticesLocalFrame)(_getProcAddress(lib,"_simGetVerticesLocalFrame"));
	_simGetHeightfieldData=(ptr_simGetHeightfieldData)(_getProcAddress(lib,"_simGetHeightfieldData"));
	_simGetCumulativeMeshes=(ptr_simGetCumulativeMeshes)(_getProcAddress(lib,"_simGetCumulativeMeshes"));
	_simGetOdeMaxContactFrictionCFMandERP=(ptr_simGetOdeMaxContactFrictionCFMandERP)(_getProcAddress(lib,"_simGetOdeMaxContactFrictionCFMandERP"));
	_simGetBulletCollisionMargin=(ptr_simGetBulletCollisionMargin)(_getProcAddress(lib,"_simGetBulletCollisionMargin"));
	_simGetBulletStickyContact=(ptr_simGetBulletStickyContact)(_getProcAddress(lib,"_simGetBulletStickyContact"));
	_simGetBulletRestitution=(ptr_simGetBulletRestitution)(_getProcAddress(lib,"_simGetBulletRestitution"));
	_simGetMass=(ptr_simGetMass)(_getProcAddress(lib,"_simGetMass"));
	_simGetPrincipalMomentOfInertia=(ptr_simGetPrincipalMomentOfInertia)(_getProcAddress(lib,"_simGetPrincipalMomentOfInertia"));
	_simGetDamping=(ptr_simGetDamping)(_getProcAddress(lib,"_simGetDamping"));
	_simGetFriction=(ptr_simGetFriction)(_getProcAddress(lib,"_simGetFriction"));
	_simGetGravity=(ptr_simGetGravity)(_getProcAddress(lib,"_simGetGravity"));
	_simGetTimeDiffInMs=(ptr_simGetTimeDiffInMs)(_getProcAddress(lib,"_simGetTimeDiffInMs"));
	_simDoEntitiesCollide=(ptr_simDoEntitiesCollide)(_getProcAddress(lib,"_simDoEntitiesCollide"));
	_simGetDistanceBetweenEntitiesIfSmaller=(ptr_simGetDistanceBetweenEntitiesIfSmaller)(_getProcAddress(lib,"_simGetDistanceBetweenEntitiesIfSmaller"));
	_simHandleJointControl=(ptr_simHandleJointControl)(_getProcAddress(lib,"_simHandleJointControl"));
	_simHandleCustomContact=(ptr_simHandleCustomContact)(_getProcAddress(lib,"_simHandleCustomContact"));
	_simGetPureHollowScaling=(ptr_simGetPureHollowScaling)(_getProcAddress(lib,"_simGetPureHollowScaling"));
	_simGetVortexParameters=(ptr_simGetVortexParameters)(_getProcAddress(lib,"_simGetVortexParameters"));
	_simGetJointCallbackCallOrder=(ptr_simGetJointCallbackCallOrder)(_getProcAddress(lib,"_simGetJointCallbackCallOrder"));
	_simGetNewtonParameters=(ptr_simGetNewtonParameters)(_getProcAddress(lib,"_simGetNewtonParameters"));

	
	char couldNotFind[]="Could not find function";
	if (simRunSimulator==0)
	{
		printf("%s simRunSimulator\n",couldNotFind);
		return 0;
	}
	if (simGetSimulatorMessage==0)
	{
		printf("%s simGetSimulatorMessage\n",couldNotFind);
		return 0;
	}
	if (simGetMainWindow==0)
	{
		printf("%s simGetMainWindow\n",couldNotFind);
		return 0;
	}
	if (simGetLastError==0)
	{
		printf("%s simGetLastError\n",couldNotFind);
		return 0;
	}
	if (simLoadModule==0)
	{
		printf("%s simLoadModule\n",couldNotFind);
		return 0;
	}
	if (simUnloadModule==0)
	{
		printf("%s simUnloadModule\n",couldNotFind);
		return 0;
	}
	if (simSendModuleMessage==0)
	{
		printf("%s simSendModuleMessage\n",couldNotFind);
		return 0;
	}
	if (simSetBooleanParameter==0)
	{
		printf("%s simSetBooleanParameter\n",couldNotFind);
		return 0;
	}
	if (simGetBooleanParameter==0)
	{
		printf("%s simGetBooleanParameter\n",couldNotFind);
		return 0;
	}
	if (simSetIntegerParameter==0)
	{
		printf("%s simSetIntegerParameter\n",couldNotFind);
		return 0;
	}
	if (simGetIntegerParameter==0)
	{
		printf("%s simGetIntegerParameter\n",couldNotFind);
		return 0;
	}
	if (simSetFloatingParameter==0)
	{
		printf("%s simSetFloatingParameter\n",couldNotFind);
		return 0;
	}
	if (simGetFloatingParameter==0)
	{
		printf("%s simGetFloatingParameter\n",couldNotFind);
		return 0;
	}
	if (simSetStringParameter==0)
	{
		printf("%s simSetStringParameter\n",couldNotFind);
		return 0;
	}
	if (simGetStringParameter==0)
	{
		printf("%s simGetStringParameter\n",couldNotFind);
		return 0;
	}
	if (simGetObjectHandle==0)
	{
		printf("%s simGetObjectHandle\n",couldNotFind);
		return 0;
	}
	if (simRemoveObject==0)
	{
		printf("%s simRemoveObject\n",couldNotFind);
		return 0;
	}
	if (simRemoveModel==0)
	{
		printf("%s simRemoveModel\n",couldNotFind);
		return 0;
	}
	if (simGetObjectName==0)
	{
		printf("%s simGetObjectName\n",couldNotFind);
		return 0;
	}
	if (simGetObjects==0)
	{
		printf("%s simGetObjects\n",couldNotFind);
		return 0;
	}
	if (simSetObjectName==0)
	{
		printf("%s simSetObjectName\n",couldNotFind);
		return 0;
	}
	if (simGetCollectionHandle==0)
	{
		printf("%s simGetCollectionHandle\n",couldNotFind);
		return 0;
	}
	if (simRemoveCollection==0)
	{
		printf("%s simRemoveCollection\n",couldNotFind);
		return 0;
	}
	if (simGetCollectionName==0)
	{
		printf("%s simGetCollectionName\n",couldNotFind);
		return 0;
	}
	if (simSetCollectionName==0)
	{
		printf("%s simSetCollectionName\n",couldNotFind);
		return 0;
	}
	if (simGetObjectMatrix==0)
	{
		printf("%s simGetObjectMatrix\n",couldNotFind);
		return 0;
	}
	if (simSetObjectMatrix==0)
	{
		printf("%s simSetObjectMatrix\n",couldNotFind);
		return 0;
	}
	if (simGetObjectPosition==0)
	{
		printf("%s simGetObjectPosition\n",couldNotFind);
		return 0;
	}
	if (simSetObjectPosition==0)
	{
		printf("%s simSetObjectPosition\n",couldNotFind);
		return 0;
	}
	if (simGetObjectOrientation==0)
	{
		printf("%s simGetObjectOrientation\n",couldNotFind);
		return 0;
	}
	if (simSetObjectOrientation==0)
	{
		printf("%s simSetObjectOrientation\n",couldNotFind);
		return 0;
	}
	if (simGetJointPosition==0)
	{
		printf("%s simGetJointPosition\n",couldNotFind);
		return 0;
	}
	if (simSetJointPosition==0)
	{
		printf("%s simSetJointPosition\n",couldNotFind);
		return 0;
	}
	if (simSetJointTargetPosition==0)
	{
		printf("%s simSetJointTargetPosition\n",couldNotFind);
		return 0;
	}
	if (simGetJointTargetPosition==0)
	{
		printf("%s simGetJointTargetPosition\n",couldNotFind);
		return 0;
	}
	if (simSetJointForce==0)
	{
		printf("%s simSetJointForce\n",couldNotFind);
		return 0;
	}
	if (simGetPathPosition==0)
	{
		printf("%s simGetPathPosition\n",couldNotFind);
		return 0;
	}
	if (simSetPathPosition==0)
	{
		printf("%s simSetPathPosition\n",couldNotFind);
		return 0;
	}
	if (simGetPathLength==0)
	{
		printf("%s simGetPathLength\n",couldNotFind);
		return 0;
	}
	if (simGetJointMatrix==0)
	{
		printf("%s simGetJointMatrix\n",couldNotFind);
		return 0;
	}
	if (simSetSphericalJointMatrix==0)
	{
		printf("%s simSetSphericalJointMatrix\n",couldNotFind);
		return 0;
	}
	if (simGetJointInterval==0)
	{
		printf("%s simGetJointInterval\n",couldNotFind);
		return 0;
	}
	if (simSetJointInterval==0)
	{
		printf("%s simSetJointInterval\n",couldNotFind);
		return 0;
	}
	if (simGetObjectParent==0)
	{
		printf("%s simGetObjectParent\n",couldNotFind);
		return 0;
	}
	if (simGetObjectChild==0)
	{
		printf("%s simGetObjectChild\n",couldNotFind);
		return 0;
	}
	if (simSetObjectParent==0)
	{
		printf("%s simSetObjectParent\n",couldNotFind);
		return 0;
	}
	if (simGetObjectType==0)
	{
		printf("%s simGetObjectType\n",couldNotFind);
		return 0;
	}
	if (simGetJointType==0)
	{
		printf("%s simGetJointType\n",couldNotFind);
		return 0;
	}
	if (simBuildIdentityMatrix==0)
	{
		printf("%s simBuildIdentityMatrix\n",couldNotFind);
		return 0;
	}
	if (simCopyMatrix==0)
	{
		printf("%s simCopyMatrix\n",couldNotFind);
		return 0;
	}
	if (simBuildMatrix==0)
	{
		printf("%s simBuildMatrix\n",couldNotFind);
		return 0;
	}
	if (simGetEulerAnglesFromMatrix==0)
	{
		printf("%s simGetEulerAnglesFromMatrix\n",couldNotFind);
		return 0;
	}
	if (simInvertMatrix==0)
	{
		printf("%s simInvertMatrix\n",couldNotFind);
		return 0;
	}
	if (simMultiplyMatrices==0)
	{
		printf("%s simMultiplyMatrices\n",couldNotFind);
		return 0;
	}
	if (simInterpolateMatrices==0)
	{
		printf("%s simInterpolateMatrices\n",couldNotFind);
		return 0;
	}
	if (simTransformVector==0)
	{
		printf("%s simTransformVector\n",couldNotFind);
		return 0;
	}
	if (simReservedCommand==0)
	{
		printf("%s simReservedCommand\n",couldNotFind);
		return 0;
	}
	if (simGetSimulationTime==0)
	{
		printf("%s simGetSimulationTime\n",couldNotFind);
		return 0;
	}
	if (simGetSimulationState==0)
	{
		printf("%s simGetSimulationState\n",couldNotFind);
		return 0;
	}
	if (simGetSystemTime==0)
	{
		printf("%s simGetSystemTime\n",couldNotFind);
		return 0;
	}
	if (simGetSystemTimeInMilliseconds==0)
	{
		printf("%s simGetSystemTimeInMilliseconds\n",couldNotFind);
		return 0;
	}
	if (simGetSystemTimeInMs==0)
	{
		printf("%s simGetSystemTimeInMs\n",couldNotFind);
		return 0;
	}
	if (simLoadScene==0)
	{
		printf("%s simLoadScene\n",couldNotFind);
		return 0;
	}
	if (simCloseScene==0)
	{
		printf("%s simCloseScene\n",couldNotFind);
		return 0;
	}
	if (simSaveScene==0)
	{
		printf("%s simSaveScene\n",couldNotFind);
		return 0;
	}
	if (simLoadModel==0)
	{
		printf("%s simLoadModel\n",couldNotFind);
		return 0;
	}
	if (simSaveModel==0)
	{
		printf("%s simSaveModel\n",couldNotFind);
		return 0;
	}
	if (simLoadUI==0)
	{
		printf("%s simLoadUI\n",couldNotFind);
		return 0;
	}
	if (simSaveUI==0)
	{
		printf("%s simSaveUI\n",couldNotFind);
		return 0;
	}
	if (simAddStatusbarMessage==0)
	{
		printf("%s simAddStatusbarMessage\n",couldNotFind);
		return 0;
	}
	if (simAddModuleMenuEntry==0)
	{
		printf("%s simAddModuleMenuEntry\n",couldNotFind);
		return 0;
	}
	if (simSetModuleMenuItemState==0)
	{
		printf("%s simSetModuleMenuItemState\n",couldNotFind);
		return 0;
	}
	if (simDoesFileExist==0)
	{
		printf("%s simDoesFileExist\n",couldNotFind);
		return 0;
	}
	if (simIsObjectInSelection==0)
	{
		printf("%s simIsObjectInSelection\n",couldNotFind);
		return 0;
	}
	if (simAddObjectToSelection==0)
	{
		printf("%s simAddObjectToSelection\n",couldNotFind);
		return 0;
	}
	if (simRemoveObjectFromSelection==0)
	{
		printf("%s simRemoveObjectFromSelection\n",couldNotFind);
		return 0;
	}
	if (simGetObjectSelectionSize==0)
	{
		printf("%s simGetObjectSelectionSize\n",couldNotFind);
		return 0;
	}
	if (simGetObjectLastSelection==0)
	{
		printf("%s simGetObjectLastSelection\n",couldNotFind);
		return 0;
	}
	if (simGetObjectSelection==0)
	{
		printf("%s simGetObjectSelection\n",couldNotFind);
		return 0;
	}
	if (simSearchPath==0)
	{
		printf("%s simSearchPath\n",couldNotFind);
		return 0;
	}
	if (simInitializePathSearch==0)
	{
		printf("%s simInitializePathSearch\n",couldNotFind);
		return 0;
	}
	if (simPerformPathSearchStep==0)
	{
		printf("%s simPerformPathSearchStep\n",couldNotFind);
		return 0;
	}
	if (simHandleCollision==0)
	{
		printf("%s simHandleCollision\n",couldNotFind);
		return 0;
	}
	if (simReadCollision==0)
	{
		printf("%s simReadCollision\n",couldNotFind);
		return 0;
	}
	if (simHandleDistance==0)
	{
		printf("%s simHandleDistance\n",couldNotFind);
		return 0;
	}
	if (simReadDistance==0)
	{
		printf("%s simReadDistance\n",couldNotFind);
		return 0;
	}
	if (simHandleProximitySensor==0)
	{
		printf("%s simHandleProximitySensor\n",couldNotFind);
		return 0;
	}
	if (simReadProximitySensor==0)
	{
		printf("%s simReadProximitySensor\n",couldNotFind);
		return 0;
	}
	if (simHandleMill==0)
	{
		printf("%s simHandleMill\n",couldNotFind);
		return 0;
	}
	if (simHandleIkGroup==0)
	{
		printf("%s simHandleIkGroup\n",couldNotFind);
		return 0;
	}
	if (simCheckIkGroup==0)
	{
		printf("%s simCheckIkGroup\n",couldNotFind);
		return 0;
	}
	if (simHandleDynamics==0)
	{
		printf("%s simHandleDynamics\n",couldNotFind);
		return 0;
	}
	if (simGetMechanismHandle==0)
	{
		printf("%s simGetMechanismHandle\n",couldNotFind);
		return 0;
	}
	if (simGetPathPlanningHandle==0)
	{
		printf("%s simGetPathPlanningHandle\n",couldNotFind);
		return 0;
	}
	if (simHandleMechanism==0)
	{
		printf("%s simHandleMechanism\n",couldNotFind);
		return 0;
	}
	if (simGetScriptHandle==0)
	{
		printf("%s simGetScriptHandle\n",couldNotFind);
		return 0;
	}
	if (simSetScriptText==0)
	{
		printf("%s simSetScriptText\n",couldNotFind);
		return 0;
	}
	if (simGetScriptText==0)
	{
		printf("%s simGetScriptText\n",couldNotFind);
		return 0;
	}
	if (simGetScriptProperty==0)
	{
		printf("%s simGetScriptProperty\n",couldNotFind);
		return 0;
	}
	if (simAssociateScriptWithObject==0)
	{
		printf("%s simAssociateScriptWithObject\n",couldNotFind);
		return 0;
	}
	if (simGetScript==0)
	{
		printf("%s simGetScript\n",couldNotFind);
		return 0;
	}
	if (simGetScriptAssociatedWithObject==0)
	{
		printf("%s simGetScriptAssociatedWithObject\n",couldNotFind);
		return 0;
	}
	if (simGetCustomizationScriptAssociatedWithObject==0)
	{
		printf("%s simGetCustomizationScriptAssociatedWithObject\n",couldNotFind);
		return 0;
	}
	if (simGetObjectAssociatedWithScript==0)
	{
		printf("%s simGetObjectAssociatedWithScript\n",couldNotFind);
		return 0;
	}
	if (simGetScriptName==0)
	{
		printf("%s simGetScriptName\n",couldNotFind);
		return 0;
	}
	if (simHandleMainScript==0)
	{
		printf("%s simHandleMainScript\n",couldNotFind);
		return 0;
	}
	if (simHandleGeneralCallbackScript==0)
	{
		printf("%s simHandleGeneralCallbackScript\n",couldNotFind);
		return 0;
	}
	if (simResetScript==0)
	{
		printf("%s simResetScript\n",couldNotFind);
		return 0;
	}
	if (simAddScript==0)
	{
		printf("%s simAddScript\n",couldNotFind);
		return 0;
	}
	if (simRemoveScript==0)
	{
		printf("%s simRemoveScript\n",couldNotFind);
		return 0;
	}
	if (simRefreshDialogs==0)
	{
		printf("%s simRefreshDialogs\n",couldNotFind);
		return 0;
	}
	if (simGetCollisionHandle==0)
	{
		printf("%s simGetCollisionHandle\n",couldNotFind);
		return 0;
	}
	if (simGetDistanceHandle==0)
	{
		printf("%s simGetDistanceHandle\n",couldNotFind);
		return 0;
	}
	if (simGetIkGroupHandle==0)
	{
		printf("%s simGetIkGroupHandle\n",couldNotFind);
		return 0;
	}
	if (simResetCollision==0)
	{
		printf("%s simResetCollision\n",couldNotFind);
		return 0;
	}
	if (simResetDistance==0)
	{
		printf("%s simResetDistance\n",couldNotFind);
		return 0;
	}
	if (simResetProximitySensor==0)
	{
		printf("%s simResetProximitySensor\n",couldNotFind);
		return 0;
	}
	if (simResetMill==0)
	{
		printf("%s simResetMill\n",couldNotFind);
		return 0;
	}
	if (simCheckProximitySensor==0)
	{
		printf("%s simCheckProximitySensor\n",couldNotFind);
		return 0;
	}
	if (simCheckProximitySensorEx==0)
	{
		printf("%s simCheckProximitySensorEx\n",couldNotFind);
		return 0;
	}
	if (simCheckProximitySensorEx2==0)
	{
		printf("%s simCheckProximitySensorEx2\n",couldNotFind);
		return 0;
	}
	if (simAddSceneCustomData==0)
	{
		printf("%s simAddSceneCustomData\n",couldNotFind);
		return 0;
	}
	if (simGetSceneCustomDataLength==0)
	{
		printf("%s simGetSceneCustomDataLength\n",couldNotFind);
		return 0;
	}
	if (simGetSceneCustomData==0)
	{
		printf("%s simGetSceneCustomData\n",couldNotFind);
		return 0;
	}
	if (simAddObjectCustomData==0)
	{
		printf("%s simAddObjectCustomData\n",couldNotFind);
		return 0;
	}
	if (simGetObjectCustomDataLength==0)
	{
		printf("%s simGetObjectCustomDataLength\n",couldNotFind);
		return 0;
	}
	if (simGetObjectCustomData==0)
	{
		printf("%s simGetObjectCustomData\n",couldNotFind);
		return 0;
	}
	if (simCreateBuffer==0)
	{
		printf("%s simCreateBuffer\n",couldNotFind);
		return 0;
	}
	if (simReleaseBuffer==0)
	{
		printf("%s simReleaseBuffer\n",couldNotFind);
		return 0;
	}
	if (simCheckCollision==0)
	{
		printf("%s simCheckCollision\n",couldNotFind);
		return 0;
	}
	if (simCheckCollisionEx==0)
	{
		printf("%s simCheckCollisionEx\n",couldNotFind);
		return 0;
	}
	if (simCheckDistance==0)
	{
		printf("%s simCheckDistance\n",couldNotFind);
		return 0;
	}
	if (simGetObjectConfiguration==0)
	{
		printf("%s simGetObjectConfiguration\n",couldNotFind);
		return 0;
	}
	if (simSetObjectConfiguration==0)
	{
		printf("%s simSetObjectConfiguration\n",couldNotFind);
		return 0;
	}
	if (simGetConfigurationTree==0)
	{
		printf("%s simGetConfigurationTree\n",couldNotFind);
		return 0;
	}
	if (simSetConfigurationTree==0)
	{
		printf("%s simSetConfigurationTree\n",couldNotFind);
		return 0;
	}
	if (simSetSimulationTimeStep==0)
	{
		printf("%s simSetSimulationTimeStep\n",couldNotFind);
		return 0;
	}
	if (simGetSimulationTimeStep==0)
	{
		printf("%s simGetSimulationTimeStep\n",couldNotFind);
		return 0;
	}
	if (simGetRealTimeSimulation==0)
	{
		printf("%s simGetRealTimeSimulation\n",couldNotFind);
		return 0;
	}
	if (simIsRealTimeSimulationStepNeeded==0)
	{
		printf("%s simIsRealTimeSimulationStepNeeded\n",couldNotFind);
		return 0;
	}
	if (simAdjustRealTimeTimer==0)
	{
		printf("%s simAdjustRealTimeTimer\n",couldNotFind);
		return 0;
	}
	if (simGetSimulationPassesPerRenderingPass==0)
	{
		printf("%s simGetSimulationPassesPerRenderingPass\n",couldNotFind);
		return 0;
	}
	if (simAdvanceSimulationByOneStep==0)
	{
		printf("%s simAdvanceSimulationByOneStep\n",couldNotFind);
		return 0;
	}
	if (simStartSimulation==0)
	{
		printf("%s simStartSimulation\n",couldNotFind);
		return 0;
	}
	if (simStopSimulation==0)
	{
		printf("%s simStopSimulation\n",couldNotFind);
		return 0;
	}
	if (simPauseSimulation==0)
	{
		printf("%s simPauseSimulation\n",couldNotFind);
		return 0;
	}
	if (simBroadcastMessage==0)
	{
		printf("%s simBroadcastMessage\n",couldNotFind);
		return 0;
	}
	if (simGetModuleName==0)
	{
		printf("%s simGetModuleName\n",couldNotFind);
		return 0;
	}
	if (simGetScriptSimulationParameter==0)
	{
		printf("%s simGetScriptSimulationParameter\n",couldNotFind);
		return 0;
	}
	if (simSetScriptSimulationParameter==0)
	{
		printf("%s simSetScriptSimulationParameter\n",couldNotFind);
		return 0;
	}
	if (simFloatingViewAdd==0)
	{
		printf("%s simFloatingViewAdd\n",couldNotFind);
		return 0;
	}
	if (simFloatingViewRemove==0)
	{
		printf("%s simFloatingViewRemove\n",couldNotFind);
		return 0;
	}
	if (simAdjustView==0)
	{
		printf("%s simAdjustView\n",couldNotFind);
		return 0;
	}
	if (simResetPath==0)
	{
		printf("%s simResetPath\n",couldNotFind);
		return 0;
	}
	if (simHandlePath==0)
	{
		printf("%s simHandlePath\n",couldNotFind);
		return 0;
	}
	if (simResetJoint==0)
	{
		printf("%s simResetJoint\n",couldNotFind);
		return 0;
	}
	if (simHandleJoint==0)
	{
		printf("%s simHandleJoint\n",couldNotFind);
		return 0;
	}
	if (simSetLastError==0)
	{
		printf("%s simSetLastError\n",couldNotFind);
		return 0;
	}
	if (simHandleGraph==0)
	{
		printf("%s simHandleGraph\n",couldNotFind);
		return 0;
	}
	if (simResetGraph==0)
	{
		printf("%s simResetGraph\n",couldNotFind);
		return 0;
	}
	if (simSetNavigationMode==0)
	{
		printf("%s simSetNavigationMode\n",couldNotFind);
		return 0;
	}
	if (simGetNavigationMode==0)
	{
		printf("%s simGetNavigationMode\n",couldNotFind);
		return 0;
	}
	if (simSetPage==0)
	{
		printf("%s simSetPage\n",couldNotFind);
		return 0;
	}
	if (simGetPage==0)
	{
		printf("%s simGetPage\n",couldNotFind);
		return 0;
	}
	if (simDisplayDialog==0)
	{
		printf("%s simDisplayDialog\n",couldNotFind);
		return 0;
	}
	if (simGetDialogResult==0)
	{
		printf("%s simGetDialogResult\n",couldNotFind);
		return 0;
	}
	if (simGetDialogInput==0)
	{
		printf("%s simGetDialogInput\n",couldNotFind);
		return 0;
	}
	if (simEndDialog==0)
	{
		printf("%s simEndDialog\n",couldNotFind);
		return 0;
	}
	if (simRegisterCustomLuaFunction==0)
	{
		printf("%s simRegisterCustomLuaFunction\n",couldNotFind);
		return 0;
	}
	if (simRegisterCustomLuaVariable==0)
	{
		printf("%s simRegisterCustomLuaVariable\n",couldNotFind);
		return 0;
	}
	if (simSetJointTargetVelocity==0)
	{
		printf("%s simSetJointTargetVelocity\n",couldNotFind);
		return 0;
	}
	if (simGetJointTargetVelocity==0)
	{
		printf("%s simGetJointTargetVelocity\n",couldNotFind);
		return 0;
	}
	if (simSetPathTargetNominalVelocity==0)
	{
		printf("%s simSetPathTargetNominalVelocity\n",couldNotFind);
		return 0;
	}
	if (simLockInterface==0)
	{
		printf("%s simLockInterface\n",couldNotFind);
		return 0;
	}
	if (simGetScriptRawBuffer==0)
	{
		printf("%s simGetScriptRawBuffer\n",couldNotFind);
		return 0;
	}
	if (simSetScriptRawBuffer==0)
	{
		printf("%s simSetScriptRawBuffer\n",couldNotFind);
		return 0;
	}
	if (simReleaseScriptRawBuffer==0)
	{
		printf("%s simReleaseScriptRawBuffer\n",couldNotFind);
		return 0;
	}
	if (simCopyPasteSelectedObjects==0)
	{
		printf("%s simCopyPasteSelectedObjects\n",couldNotFind);
		return 0;
	}
	if (simCopyPasteObjects==0)
	{
		printf("%s simCopyPasteObjects\n",couldNotFind);
		return 0;
	}
	if (simScaleSelectedObjects==0)
	{
		printf("%s simScaleSelectedObjects\n",couldNotFind);
		return 0;
	}
	if (simScaleObjects==0)
	{
		printf("%s simScaleObjects\n",couldNotFind);
		return 0;
	}
	if (simDeleteSelectedObjects==0)
	{
		printf("%s simDeleteSelectedObjects\n",couldNotFind);
		return 0;
	}
	if (simGetObjectUniqueIdentifier==0)
	{
		printf("%s simGetObjectUniqueIdentifier\n",couldNotFind);
		return 0;
	}
	if (simGetNameSuffix==0)
	{
		printf("%s simGetNameSuffix\n",couldNotFind);
		return 0;
	}
	if (simSendData==0)
	{
		printf("%s simSendData\n",couldNotFind);
		return 0;
	}
	if (simReceiveData==0)
	{
		printf("%s simReceiveData\n",couldNotFind);
		return 0;
	}
	if (simSetGraphUserData==0)
	{
		printf("%s simSetGraphUserData\n",couldNotFind);
		return 0;
	}
	if (simSetNameSuffix==0)
	{
		printf("%s simSetNameSuffix\n",couldNotFind);
		return 0;
	}
	if (simAddDrawingObject==0)
	{
		printf("%s simAddDrawingObject\n",couldNotFind);
		return 0;
	}
	if (simRemoveDrawingObject==0)
	{
		printf("%s simRemoveDrawingObject\n",couldNotFind);
		return 0;
	}
	if (simAddDrawingObjectItem==0)
	{
		printf("%s simAddDrawingObjectItem\n",couldNotFind);
		return 0;
	}
	if (simAddParticleObject==0)
	{
		printf("%s simAddParticleObject\n",couldNotFind);
		return 0;
	}
	if (simRemoveParticleObject==0)
	{
		printf("%s simRemoveParticleObject\n",couldNotFind);
		return 0;
	}
	if (simAddParticleObjectItem==0)
	{
		printf("%s simAddParticleObjectItem\n",couldNotFind);
		return 0;
	}
	if (simGetObjectSizeFactor==0)
	{
		printf("%s simGetObjectSizeFactor\n",couldNotFind);
		return 0;
	}
	if (simAnnounceSceneContentChange==0)
	{
		printf("%s simAnnounceSceneContentChange\n",couldNotFind);
		return 0;
	}
	if (simResetMilling==0)
	{
		printf("%s simResetMilling\n",couldNotFind);
		return 0;
	}
	if (simApplyMilling==0)
	{
		printf("%s simApplyMilling\n",couldNotFind);
		return 0;
	}
	if (simSetIntegerSignal==0)
	{
		printf("%s simSetIntegerSignal\n",couldNotFind);
		return 0;
	}
	if (simGetIntegerSignal==0)
	{
		printf("%s simGetIntegerSignal\n",couldNotFind);
		return 0;
	}
	if (simClearIntegerSignal==0)
	{
		printf("%s simClearIntegerSignal\n",couldNotFind);
		return 0;
	}
	if (simSetFloatSignal==0)
	{
		printf("%s simSetFloatSignal\n",couldNotFind);
		return 0;
	}
	if (simGetFloatSignal==0)
	{
		printf("%s simGetFloatSignal\n",couldNotFind);
		return 0;
	}
	if (simClearFloatSignal==0)
	{
		printf("%s simClearFloatSignal\n",couldNotFind);
		return 0;
	}
	if (simSetStringSignal==0)
	{
		printf("%s simSetStringSignal\n",couldNotFind);
		return 0;
	}
	if (simGetStringSignal==0)
	{
		printf("%s simGetStringSignal\n",couldNotFind);
		return 0;
	}
	if (simClearStringSignal==0)
	{
		printf("%s simClearStringSignal\n",couldNotFind);
		return 0;
	}
	if (simGetSignalName==0)
	{
		printf("%s simGetSignalName\n",couldNotFind);
		return 0;
	}
	if (simSetObjectProperty==0)
	{
		printf("%s simSetObjectProperty\n",couldNotFind);
		return 0;
	}
	if (simGetObjectProperty==0)
	{
		printf("%s simGetObjectProperty\n",couldNotFind);
		return 0;
	}
	if (simSetObjectSpecialProperty==0)
	{
		printf("%s simSetObjectSpecialProperty\n",couldNotFind);
		return 0;
	}
	if (simGetObjectSpecialProperty==0)
	{
		printf("%s simGetObjectSpecialProperty\n",couldNotFind);
		return 0;
	}
	if (simGetPositionOnPath==0)
	{
		printf("%s simGetPositionOnPath\n",couldNotFind);
		return 0;
	}
	if (simGetDataOnPath==0)
	{
		printf("%s simGetDataOnPath\n",couldNotFind);
		return 0;
	}
	if (simGetOrientationOnPath==0)
	{
		printf("%s simGetOrientationOnPath\n",couldNotFind);
		return 0;
	}
	if (simGetClosestPositionOnPath==0)
	{
		printf("%s simGetClosestPositionOnPath\n",couldNotFind);
		return 0;
	}
	if (simReadForceSensor==0)
	{
		printf("%s simReadForceSensor\n",couldNotFind);
		return 0;
	}
	if (simBreakForceSensor==0)
	{
		printf("%s simBreakForceSensor\n",couldNotFind);
		return 0;
	}
	if (simGetShapeVertex==0)
	{
		printf("%s simGetShapeVertex\n",couldNotFind);
		return 0;
	}
	if (simGetShapeTriangle==0)
	{
		printf("%s simGetShapeTriangle\n",couldNotFind);
		return 0;
	}
	if (simSetLightParameters==0)
	{
		printf("%s simSetLightParameters\n",couldNotFind);
		return 0;
	}
	if (simGetLightParameters==0)
	{
		printf("%s simGetLightParameters\n",couldNotFind);
		return 0;
	}
	if (simHandleVarious==0)
	{
		printf("%s simHandleVarious\n",couldNotFind);
		return 0;
	}
	if (simGetVelocity==0)
	{
		printf("%s simGetVelocity\n",couldNotFind);
		return 0;
	}
	if (simGetObjectVelocity==0)
	{
		printf("%s simGetObjectVelocity\n",couldNotFind);
		return 0;
	}
	if (simAddForceAndTorque==0)
	{
		printf("%s simAddForceAndTorque\n",couldNotFind);
		return 0;
	}
	if (simAddForce==0)
	{
		printf("%s simAddForce\n",couldNotFind);
		return 0;
	}
	if (simSetExplicitHandling==0)
	{
		printf("%s simSetExplicitHandling\n",couldNotFind);
		return 0;
	}
	if (simGetExplicitHandling==0)
	{
		printf("%s simGetExplicitHandling\n",couldNotFind);
		return 0;
	}
	if (simGetLinkDummy==0)
	{
		printf("%s simGetLinkDummy\n",couldNotFind);
		return 0;
	}
	if (simSetLinkDummy==0)
	{
		printf("%s simSetLinkDummy\n",couldNotFind);
		return 0;
	}
	if (simSetModelProperty==0)
	{
		printf("%s simSetModelProperty\n",couldNotFind);
		return 0;
	}
	if (simGetModelProperty==0)
	{
		printf("%s simGetModelProperty\n",couldNotFind);
		return 0;
	}
	if (simSetShapeColor==0)
	{
		printf("%s simSetShapeColor\n",couldNotFind);
		return 0;
	}
	if (simGetShapeColor==0)
	{
		printf("%s simGetShapeColor\n",couldNotFind);
		return 0;
	}
	if (simResetDynamicObject==0)
	{
		printf("%s simResetDynamicObject\n",couldNotFind);
		return 0;
	}
	if (simSetJointMode==0)
	{
		printf("%s simSetJointMode\n",couldNotFind);
		return 0;
	}
	if (simGetJointMode==0)
	{
		printf("%s simGetJointMode\n",couldNotFind);
		return 0;
	}
	if (simSerialOpen==0)
	{
		printf("%s simSerialOpen\n",couldNotFind);
		return 0;
	}
	if (simSerialClose==0)
	{
		printf("%s simSerialClose\n",couldNotFind);
		return 0;
	}
	if (simSerialSend==0)
	{
		printf("%s simSerialSend\n",couldNotFind);
		return 0;
	}
	if (simSerialRead==0)
	{
		printf("%s simSerialRead\n",couldNotFind);
		return 0;
	}
	if (simSerialCheck==0)
	{
		printf("%s simSerialCheck\n",couldNotFind);
		return 0;
	}
	if (simSerialPortOpen==0)
	{
		printf("%s simSerialPortOpen\n",couldNotFind);
		return 0;
	}
	if (simSerialPortClose==0)
	{
		printf("%s simSerialPortClose\n",couldNotFind);
		return 0;
	}
	if (simSerialPortSend==0)
	{
		printf("%s simSerialPortSend\n",couldNotFind);
		return 0;
	}
	if (simSerialPortRead==0)
	{
		printf("%s simSerialPortRead\n",couldNotFind);
		return 0;
	}
	if (simGetContactInfo==0)
	{
		printf("%s simGetContactInfo\n",couldNotFind);
		return 0;
	}
	if (simSetThreadIsFree==0)
	{
		printf("%s simSetThreadIsFree\n",couldNotFind);
		return 0;
	}
	if (simTubeOpen==0)
	{
		printf("%s simTubeOpen\n",couldNotFind);
		return 0;
	}
	if (simTubeClose==0)
	{
		printf("%s simTubeClose\n",couldNotFind);
		return 0;
	}
	if (simTubeWrite==0)
	{
		printf("%s simTubeWrite\n",couldNotFind);
		return 0;
	}
	if (simTubeRead==0)
	{
		printf("%s simTubeRead\n",couldNotFind);
		return 0;
	}
	if (simTubeStatus==0)
	{
		printf("%s simTubeStatus\n",couldNotFind);
		return 0;
	}
	if (simAuxiliaryConsoleOpen==0)
	{
		printf("%s simAuxiliaryConsoleOpen\n",couldNotFind);
		return 0;
	}
	if (simAuxiliaryConsoleClose==0)
	{
		printf("%s simAuxiliaryConsoleClose\n",couldNotFind);
		return 0;
	}
	if (simAuxiliaryConsoleShow==0)
	{
		printf("%s simAuxiliaryConsoleShow\n",couldNotFind);
		return 0;
	}
	if (simAuxiliaryConsolePrint==0)
	{
		printf("%s simAuxiliaryConsolePrint\n",couldNotFind);
		return 0;
	}
	if (simImportShape==0)
	{
		printf("%s simImportShape\n",couldNotFind);
		return 0;
	}
	if (simImportMesh==0)
	{
		printf("%s simImportMesh\n",couldNotFind);
		return 0;
	}
	if (simExportMesh==0)
	{
		printf("%s simExportMesh\n",couldNotFind);
		return 0;
	}
	if (simCreateMeshShape==0)
	{
		printf("%s simCreateMeshShape\n",couldNotFind);
		return 0;
	}
	if (simCreatePureShape==0)
	{
		printf("%s simCreatePureShape\n",couldNotFind);
		return 0;
	}
	if (simCreateHeightfieldShape==0)
	{
		printf("%s simCreateHeightfieldShape\n",couldNotFind);
		return 0;
	}
	if (simGetShapeMesh==0)
	{
		printf("%s simGetShapeMesh\n",couldNotFind);
		return 0;
	}
	if (simAddBanner==0)
	{
		printf("%s simAddBanner\n",couldNotFind);
		return 0;
	}
	if (simRemoveBanner==0)
	{
		printf("%s simRemoveBanner\n",couldNotFind);
		return 0;
	}
	if (simCreateJoint==0)
	{
		printf("%s simCreateJoint\n",couldNotFind);
		return 0;
	}
	if (simCreateDummy==0)
	{
		printf("%s simCreateDummy\n",couldNotFind);
		return 0;
	}
	if (simCreateProximitySensor==0)
	{
		printf("%s simCreateProximitySensor\n",couldNotFind);
		return 0;
	}
	if (simCreatePath==0)
	{
		printf("%s simCreatePath\n",couldNotFind);
		return 0;
	}
	if (simInsertPathCtrlPoints==0)
	{
		printf("%s simInsertPathCtrlPoints\n",couldNotFind);
		return 0;
	}
	if (simCutPathCtrlPoints==0)
	{
		printf("%s simCutPathCtrlPoints\n",couldNotFind);
		return 0;
	}
	if (simCreateVisionSensor==0)
	{
		printf("%s simCreateVisionSensor\n",couldNotFind);
		return 0;
	}
	if (simCreateForceSensor==0)
	{
		printf("%s simCreateForceSensor\n",couldNotFind);
		return 0;
	}
	if (simRegisterContactCallback==0)
	{
		printf("%s simRegisterContactCallback\n",couldNotFind);
		return 0;
	}
	if (simGetObjectIntParameter==0)
	{
		printf("%s simGetObjectIntParameter\n",couldNotFind);
		return 0;
	}
	if (simSetObjectIntParameter==0)
	{
		printf("%s simSetObjectIntParameter\n",couldNotFind);
		return 0;
	}
	if (simGetObjectFloatParameter==0)
	{
		printf("%s simGetObjectFloatParameter\n",couldNotFind);
		return 0;
	}
	if (simSetObjectFloatParameter==0)
	{
		printf("%s simSetObjectFloatParameter\n",couldNotFind);
		return 0;
	}
	if (simGetObjectStringParameter==0)
	{
		printf("%s simGetObjectStringParameter\n",couldNotFind);
		return 0;
	}
	if (simSetObjectStringParameter==0)
	{
		printf("%s simSetObjectStringParameter\n",couldNotFind);
		return 0;
	}
	if (simSetSimulationPassesPerRenderingPass==0)
	{
		printf("%s simSetSimulationPassesPerRenderingPass\n",couldNotFind);
		return 0;
	}
	if (simGetRotationAxis==0)
	{
		printf("%s simGetRotationAxis\n",couldNotFind);
		return 0;
	}
	if (simRotateAroundAxis==0)
	{
		printf("%s simRotateAroundAxis\n",couldNotFind);
		return 0;
	}
	if (simJointGetForce==0)
	{
		printf("%s simJointGetForce\n",couldNotFind);
		return 0;
	}
	if (simGetJointForce==0)
	{
		printf("%s simGetJointForce\n",couldNotFind);
		return 0;
	}
	if (simSetArrayParameter==0)
	{
		printf("%s simSetArrayParameter\n",couldNotFind);
		return 0;
	}
	if (simGetArrayParameter==0)
	{
		printf("%s simGetArrayParameter\n",couldNotFind);
		return 0;
	}
	if (simSetIkGroupProperties==0)
	{
		printf("%s simSetIkGroupProperties\n",couldNotFind);
		return 0;
	}
	if (simSetIkElementProperties==0)
	{
		printf("%s simSetIkElementProperties\n",couldNotFind);
		return 0;
	}
	if (simCameraFitToView==0)
	{
		printf("%s simCameraFitToView\n",couldNotFind);
		return 0;
	}
	if (simPersistentDataWrite==0)
	{
		printf("%s simPersistentDataWrite\n",couldNotFind);
		return 0;
	}
	if (simPersistentDataRead==0)
	{
		printf("%s simPersistentDataRead\n",couldNotFind);
		return 0;
	}
	if (simIsHandleValid==0)
	{
		printf("%s simIsHandleValid\n",couldNotFind);
		return 0;
	}
	if (simHandleVisionSensor==0)
	{
		printf("%s simHandleVisionSensor\n",couldNotFind);
		return 0;
	}
	if (simReadVisionSensor==0)
	{
		printf("%s simReadVisionSensor\n",couldNotFind);
		return 0;
	}
	if (simResetVisionSensor==0)
	{
		printf("%s simResetVisionSensor\n",couldNotFind);
		return 0;
	}
	if (simCheckVisionSensor==0)
	{
		printf("%s simCheckVisionSensor\n",couldNotFind);
		return 0;
	}
	if (simCheckVisionSensorEx==0)
	{
		printf("%s simCheckVisionSensorEx\n",couldNotFind);
		return 0;
	}
	if (simGetVisionSensorResolution==0)
	{
		printf("%s simGetVisionSensorResolution\n",couldNotFind);
		return 0;
	}
	if (simGetVisionSensorImage==0)
	{
		printf("%s simGetVisionSensorImage\n",couldNotFind);
		return 0;
	}
	if (simGetVisionSensorCharImage==0)
	{
		printf("%s simGetVisionSensorCharImage\n",couldNotFind);
		return 0;
	}
	if (simSetVisionSensorImage==0)
	{
		printf("%s simSetVisionSensorImage\n",couldNotFind);
		return 0;
	}
	if (simSetVisionSensorCharImage==0)
	{
		printf("%s simSetVisionSensorCharImage\n",couldNotFind);
		return 0;
	}
	if (simGetVisionSensorDepthBuffer==0)
	{
		printf("%s simGetVisionSensorDepthBuffer\n",couldNotFind);
		return 0;
	}
	if (simCreateUI==0)
	{
		printf("%s simCreateUI\n",couldNotFind);
		return 0;
	}
	if (simCreateUIButton==0)
	{
		printf("%s simCreateUIButton\n",couldNotFind);
		return 0;
	}
	if (simGetUIHandle==0)
	{
		printf("%s simGetUIHandle\n",couldNotFind);
		return 0;
	}
	if (simGetUIProperty==0)
	{
		printf("%s simGetUIProperty\n",couldNotFind);
		return 0;
	}
	if (simGetUIEventButton==0)
	{
		printf("%s simGetUIEventButton\n",couldNotFind);
		return 0;
	}
	if (simSetUIProperty==0)
	{
		printf("%s simSetUIProperty\n",couldNotFind);
		return 0;
	}
	if (simGetUIButtonProperty==0)
	{
		printf("%s simGetUIButtonProperty\n",couldNotFind);
		return 0;
	}
	if (simSetUIButtonProperty==0)
	{
		printf("%s simSetUIButtonProperty\n",couldNotFind);
		return 0;
	}
	if (simGetUIButtonSize==0)
	{
		printf("%s simGetUIButtonSize\n",couldNotFind);
		return 0;
	}
	if (simSetUIButtonLabel==0)
	{
		printf("%s simSetUIButtonLabel\n",couldNotFind);
		return 0;
	}
	if (simGetUIButtonLabel==0)
	{
		printf("%s simGetUIButtonLabel\n",couldNotFind);
		return 0;
	}
	if (simSetUISlider==0)
	{
		printf("%s simSetUISlider\n",couldNotFind);
		return 0;
	}
	if (simGetUISlider==0)
	{
		printf("%s simGetUISlider\n",couldNotFind);
		return 0;
	}
	if (simSetUIButtonColor==0)
	{
		printf("%s simSetUIButtonColor\n",couldNotFind);
		return 0;
	}
	if (simSetUIButtonTexture==0)
	{
		printf("%s simSetUIButtonTexture\n",couldNotFind);
		return 0;
	}
	if (simCreateUIButtonArray==0)
	{
		printf("%s simCreateUIButtonArray\n",couldNotFind);
		return 0;
	}
	if (simSetUIButtonArrayColor==0)
	{
		printf("%s simSetUIButtonArrayColor\n",couldNotFind);
		return 0;
	}
	if (simDeleteUIButtonArray==0)
	{
		printf("%s simDeleteUIButtonArray\n",couldNotFind);
		return 0;
	}
	if (simRemoveUI==0)
	{
		printf("%s simRemoveUI\n",couldNotFind);
		return 0;
	}
	if (simSetUIPosition==0)
	{
		printf("%s simSetUIPosition\n",couldNotFind);
		return 0;
	}
	if (simGetUIPosition==0)
	{
		printf("%s simGetUIPosition\n",couldNotFind);
		return 0;
	}
	if (simGetObjectQuaternion==0)
	{
		printf("%s simGetObjectQuaternion\n",couldNotFind);
		return 0;
	}
	if (simSetObjectQuaternion==0)
	{
		printf("%s simSetObjectQuaternion\n",couldNotFind);
		return 0;
	}
	if (simRMLPosition==0)
	{
		printf("%s simRMLPosition\n",couldNotFind);
		return 0;
	}
	if (simRMLVelocity==0)
	{
		printf("%s simRMLVelocity\n",couldNotFind);
		return 0;
	}
	if (simRMLPos==0)
	{
		printf("%s simRMLPos\n",couldNotFind);
		return 0;
	}
	if (simRMLVel==0)
	{
		printf("%s simRMLVel\n",couldNotFind);
		return 0;
	}
	if (simRMLStep==0)
	{
		printf("%s simRMLStep\n",couldNotFind);
		return 0;
	}
	if (simRMLRemove==0)
	{
		printf("%s simRMLRemove\n",couldNotFind);
		return 0;
	}
	if (simBuildMatrixQ==0)
	{
		printf("%s simBuildMatrixQ\n",couldNotFind);
		return 0;
	}
	if (simGetQuaternionFromMatrix==0)
	{
		printf("%s simGetQuaternionFromMatrix\n",couldNotFind);
		return 0;
	}
	if (simFileDialog==0)
	{
		printf("%s simFileDialog\n",couldNotFind);
		return 0;
	}
	if (simMsgBox==0)
	{
		printf("%s simMsgBox\n",couldNotFind);
		return 0;
	}
	if (simSetShapeMassAndInertia==0)
	{
		printf("%s simSetShapeMassAndInertia\n",couldNotFind);
		return 0;
	}
	if (simGetShapeMassAndInertia==0)
	{
		printf("%s simGetShapeMassAndInertia\n",couldNotFind);
		return 0;
	}
	if (simGroupShapes==0)
	{
		printf("%s simGroupShapes\n",couldNotFind);
		return 0;
	}
	if (simUngroupShape==0)
	{
		printf("%s simUngroupShape\n",couldNotFind);
		return 0;
	}
	if (simConvexDecompose==0)
	{
		printf("%s simConvexDecompose\n",couldNotFind);
		return 0;
	}
	if (simGetIkGroupMatrix==0)
	{
		printf("%s simGetIkGroupMatrix\n",couldNotFind);
		return 0;
	}
	if (simGetMotionPlanningHandle==0)
	{
		printf("%s simGetMotionPlanningHandle\n",couldNotFind);
		return 0;
	}
	if (simGetMpConfigForTipPose==0)
	{
		printf("%s simGetMpConfigForTipPose\n",couldNotFind);
		return 0;
	}
	if (simFindMpPath==0)
	{
		printf("%s simFindMpPath\n",couldNotFind);
		return 0;
	}
	if (simSimplifyMpPath==0)
	{
		printf("%s simSimplifyMpPath\n",couldNotFind);
		return 0;
	}
	if (simGetMpConfigTransition==0)
	{
		printf("%s simGetMpConfigTransition\n",couldNotFind);
		return 0;
	}
	if (simAddGhost==0)
	{
		printf("%s simAddGhost\n",couldNotFind);
		return 0;
	}
	if (simModifyGhost==0)
	{
		printf("%s simModifyGhost\n",couldNotFind);
		return 0;
	}
	if (simQuitSimulator==0)
	{
		printf("%s simQuitSimulator\n",couldNotFind);
		return 0;
	}
	if (simGetThreadId==0)
	{
		printf("%s simGetThreadId\n",couldNotFind);
		return 0;
	}
	if (simLockResources==0)
	{
		printf("%s simLockResources\n",couldNotFind);
		return 0;
	}
	if (simUnlockResources==0)
	{
		printf("%s simUnlockResources\n",couldNotFind);
		return 0;
	}
	if (simEnableEventCallback==0)
	{
		printf("%s simEnableEventCallback\n",couldNotFind);
		return 0;
	}
	if (simGetMaterialId==0)
	{
		printf("%s simGetMaterialId\n",couldNotFind);
		return 0;
	}
	if (simSetShapeMaterial==0)
	{
		printf("%s simSetShapeMaterial\n",couldNotFind);
		return 0;
	}
	if (simGetShapeMaterial==0)
	{
		printf("%s simGetShapeMaterial\n",couldNotFind);
		return 0;
	}
	if (simFindIkPath==0)
	{
		printf("%s simFindIkPath\n",couldNotFind);
		return 0;
	}
	if (simGetTextureId==0)
	{
		printf("%s simGetTextureId\n",couldNotFind);
		return 0;
	}
	if (simReadTexture==0)
	{
		printf("%s simReadTexture\n",couldNotFind);
		return 0;
	}
	if (simWriteTexture==0)
	{
		printf("%s simWriteTexture\n",couldNotFind);
		return 0;
	}
	if (simCreateTexture==0)
	{
		printf("%s simCreateTexture\n",couldNotFind);
		return 0;
	}
	if (simWriteCustomDataBlock==0)
	{
		printf("%s simWriteCustomDataBlock\n",couldNotFind);
		return 0;
	}
	if (simReadCustomDataBlock==0)
	{
		printf("%s simReadCustomDataBlock\n",couldNotFind);
		return 0;
	}
	if (simAddPointCloud==0)
	{
		printf("%s simAddPointCloud\n",couldNotFind);
		return 0;
	}
	if (simModifyPointCloud==0)
	{
		printf("%s simModifyPointCloud\n",couldNotFind);
		return 0;
	}
	if (simGetShapeGeomInfo==0)
	{
		printf("%s simGetShapeGeomInfo\n",couldNotFind);
		return 0;
	}
	if (simGetObjectsInTree==0)
	{
		printf("%s simGetObjectsInTree\n",couldNotFind);
		return 0;
	}
	if (simSetObjectSizeValues==0)
	{
		printf("%s simSetObjectSizeValues\n",couldNotFind);
		return 0;
	}
	if (simGetObjectSizeValues==0)
	{
		printf("%s simGetObjectSizeValues\n",couldNotFind);
		return 0;
	}
	if (simScaleObject==0)
	{
		printf("%s simScaleObject\n",couldNotFind);
		return 0;
	}
	if (simSetShapeTexture==0)
	{
		printf("%s simSetShapeTexture\n",couldNotFind);
		return 0;
	}
	if (simGetShapeTextureId==0)
	{
		printf("%s simGetShapeTextureId\n",couldNotFind);
		return 0;
	}
	if (simGetCollectionObjects==0)
	{
		printf("%s simGetCollectionObjects\n",couldNotFind);
		return 0;
	}
	if (simHandleCustomizationScripts==0)
	{
		printf("%s simHandleCustomizationScripts\n",couldNotFind);
		return 0;
	}
	if (simSetScriptAttribute==0)
	{
		printf("%s simSetScriptAttribute\n",couldNotFind);
		return 0;
	}
	if (simGetScriptAttribute==0)
	{
		printf("%s simGetScriptAttribute\n",couldNotFind);
		return 0;
	}
	if (simReorientShapeBoundingBox==0)
	{
		printf("%s simReorientShapeBoundingBox\n",couldNotFind);
		return 0;
	}
	if (simSwitchThread==0)
	{
		printf("%s simSwitchThread\n",couldNotFind);
		return 0;
	}
	if (simCreateIkGroup==0)
	{
		printf("%s simCreateIkGroup\n",couldNotFind);
		return 0;
	}
	if (simRemoveIkGroup==0)
	{
		printf("%s simRemoveIkGroup\n",couldNotFind);
		return 0;
	}
	if (simCreateIkElement==0)
	{
		printf("%s simCreateIkElement\n",couldNotFind);
		return 0;
	}
	if (simCreateMotionPlanning==0)
	{
		printf("%s simCreateMotionPlanning\n",couldNotFind);
		return 0;
	}
	if (simRemoveMotionPlanning==0)
	{
		printf("%s simRemoveMotionPlanning\n",couldNotFind);
		return 0;
	}
	if (simCreateCollection==0)
	{
		printf("%s simCreateCollection\n",couldNotFind);
		return 0;
	}
	if (simAddObjectToCollection==0)
	{
		printf("%s simAddObjectToCollection\n",couldNotFind);
		return 0;
	}
	if (simSaveImage==0)
	{
		printf("%s simSaveImage\n",couldNotFind);
		return 0;
	}




	if (_simGetContactCallbackCount==0)
	{
		printf("%s _simGetContactCallbackCount\n",couldNotFind);
		return 0;
	}
	if (_simGetContactCallback==0)
	{
		printf("%s _simGetContactCallback\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicSimulationIconCode==0)
	{
		printf("%s _simSetDynamicSimulationIconCode\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicObjectFlagForVisualization==0)
	{
		printf("%s _simSetDynamicObjectFlagForVisualization\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectListSize==0)
	{
		printf("%s _simGetObjectListSize\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectFromIndex==0)
	{
		printf("%s _simGetObjectFromIndex\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectID==0)
	{
		printf("%s _simGetObjectID\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectType==0)
	{
		printf("%s _simGetObjectType\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectChildren==0)
	{
		printf("%s _simGetObjectChildren\n",couldNotFind);
		return 0;
	}
	if (_simGetGeomProxyFromShape==0)
	{
		printf("%s _simGetGeomProxyFromShape\n",couldNotFind);
		return 0;
	}
	if (_simGetParentObject==0)
	{
		printf("%s _simGetParentObject\n",couldNotFind);
		return 0;
	}
	if (_simGetObject==0)
	{
		printf("%s _simGetObject\n",couldNotFind);
		return 0;
	}
	if (_simGetIkGroupObject==0)
	{
		printf("%s _simGetIkGroupObject\n",couldNotFind);
		return 0;
	}
	if (_simMpHandleIkGroupObject==0)
	{
		printf("%s _simMpHandleIkGroupObject\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectLocalTransformation==0)
	{
		printf("%s _simGetObjectLocalTransformation\n",couldNotFind);
		return 0;
	}
	if (_simSetObjectLocalTransformation==0)
	{
		printf("%s _simSetObjectLocalTransformation\n",couldNotFind);
		return 0;
	}
	if (_simSetObjectCumulativeTransformation==0)
	{
		printf("%s _simSetObjectCumulativeTransformation\n",couldNotFind);
		return 0;
	}
	if (_simGetObjectCumulativeTransformation==0)
	{
		printf("%s _simGetObjectCumulativeTransformation\n",couldNotFind);
		return 0;
	}
	if (_simIsShapeDynamicallyStatic==0)
	{
		printf("%s _simIsShapeDynamicallyStatic\n",couldNotFind);
		return 0;
	}
	if (_simGetTreeDynamicProperty==0)
	{
		printf("%s _simGetTreeDynamicProperty\n",couldNotFind);
		return 0;
	}
	if (_simGetDummyLinkType==0)
	{
		printf("%s _simGetDummyLinkType\n",couldNotFind);
		return 0;
	}
	if (_simGetJointMode==0)
	{
		printf("%s _simGetJointMode\n",couldNotFind);
		return 0;
	}
	if (_simIsJointInHybridOperation==0)
	{
		printf("%s _simIsJointInHybridOperation\n",couldNotFind);
		return 0;
	}
	if (_simDisableDynamicTreeForManipulation==0)
	{
		printf("%s _simDisableDynamicTreeForManipulation\n",couldNotFind);
		return 0;
	}
	if (_simIsShapeDynamicallyRespondable==0)
	{
		printf("%s _simIsShapeDynamicallyRespondable\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicCollisionMask==0)
	{
		printf("%s _simGetDynamicCollisionMask\n",couldNotFind);
		return 0;
	}
	if (_simGetLastParentForLocalGlobalCollidable==0)
	{
		printf("%s _simGetLastParentForLocalGlobalCollidable\n",couldNotFind);
		return 0;
	}
	if (_simSetShapeIsStaticAndNotRespondableButDynamicTag==0)
	{
		printf("%s _simSetShapeIsStaticAndNotRespondableButDynamicTag\n",couldNotFind);
		return 0;
	}
	if (_simGetShapeIsStaticAndNotRespondableButDynamicTag==0)
	{
		printf("%s _simGetShapeIsStaticAndNotRespondableButDynamicTag\n",couldNotFind);
		return 0;
	}
	if (_simSetJointPosition==0)
	{
		printf("%s _simSetJointPosition\n",couldNotFind);
		return 0;
	}
	if (_simGetJointPosition==0)
	{
		printf("%s _simGetJointPosition\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicMotorPositionControlTargetPosition==0)
	{
		printf("%s _simSetDynamicMotorPositionControlTargetPosition\n",couldNotFind);
		return 0;
	}
	if (_simGetInitialDynamicVelocity==0)
	{
		printf("%s _simGetInitialDynamicVelocity\n",couldNotFind);
		return 0;
	}
	if (_simSetInitialDynamicVelocity==0)
	{
		printf("%s _simSetInitialDynamicVelocity\n",couldNotFind);
		return 0;
	}
	if (_simGetInitialDynamicAngVelocity==0)
	{
		printf("%s _simGetInitialDynamicAngVelocity\n",couldNotFind);
		return 0;
	}
	if (_simSetInitialDynamicAngVelocity==0)
	{
		printf("%s _simSetInitialDynamicAngVelocity\n",couldNotFind);
		return 0;
	}
	if (_simGetStartSleeping==0)
	{
		printf("%s _simGetStartSleeping\n",couldNotFind);
		return 0;
	}
	if (_simGetWasPutToSleepOnce==0)
	{
		printf("%s _simGetWasPutToSleepOnce\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicsFullRefreshFlag==0)
	{
		printf("%s _simGetDynamicsFullRefreshFlag\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicsFullRefreshFlag==0)
	{
		printf("%s _simSetDynamicsFullRefreshFlag\n",couldNotFind);
		return 0;
	}
	if (_simSetGeomProxyDynamicsFullRefreshFlag==0)
	{
		printf("%s _simSetGeomProxyDynamicsFullRefreshFlag\n",couldNotFind);
		return 0;
	}
	if (_simGetGeomProxyDynamicsFullRefreshFlag==0)
	{
		printf("%s _simGetGeomProxyDynamicsFullRefreshFlag\n",couldNotFind);
		return 0;
	}
	if (_simGetParentFollowsDynamic==0)
	{
		printf("%s _simGetParentFollowsDynamic\n",couldNotFind);
		return 0;
	}
	if (_simSetShapeDynamicVelocity==0)
	{
		printf("%s _simSetShapeDynamicVelocity\n",couldNotFind);
		return 0;
	}
	if (_simGetAdditionalForceAndTorque==0)
	{
		printf("%s _simGetAdditionalForceAndTorque\n",couldNotFind);
		return 0;
	}
	if (_simClearAdditionalForceAndTorque==0)
	{
		printf("%s _simClearAdditionalForceAndTorque\n",couldNotFind);
		return 0;
	}
	if (_simGetJointPositionInterval==0)
	{
		printf("%s _simGetJointPositionInterval\n",couldNotFind);
		return 0;
	}
	if (_simGetJointType==0)
	{
		printf("%s _simGetJointType\n",couldNotFind);
		return 0;
	}
	if (_simGetJointOdeParameters==0)
	{
		printf("%s _simGetJointOdeParameters\n",couldNotFind);
		return 0;
	}
	if (_simGetJointBulletParameters==0)
	{
		printf("%s _simGetJointBulletParameters\n",couldNotFind);
		return 0;
	}
	if (_simIsForceSensorBroken==0)
	{
		printf("%s _simIsForceSensorBroken\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicForceSensorLocalTransformationPart2==0)
	{
		printf("%s _simGetDynamicForceSensorLocalTransformationPart2\n",couldNotFind);
		return 0;
	}
	if (_simIsDynamicMotorEnabled==0)
	{
		printf("%s _simIsDynamicMotorEnabled\n",couldNotFind);
		return 0;
	}
	if (_simIsDynamicMotorPositionCtrlEnabled==0)
	{
		printf("%s _simIsDynamicMotorPositionCtrlEnabled\n",couldNotFind);
		return 0;
	}
	if (_simIsDynamicMotorTorqueModulationEnabled==0)
	{
		printf("%s _simIsDynamicMotorTorqueModulationEnabled\n",couldNotFind);
		return 0;
	}
	if (_simGetMotorPid==0)
	{
		printf("%s _simGetMotorPid\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicMotorTargetPosition==0)
	{
		printf("%s _simGetDynamicMotorTargetPosition\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicMotorTargetVelocity==0)
	{
		printf("%s _simGetDynamicMotorTargetVelocity\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicMotorMaxForce==0)
	{
		printf("%s _simGetDynamicMotorMaxForce\n",couldNotFind);
		return 0;
	}
	if (_simGetDynamicMotorUpperLimitVelocity==0)
	{
		printf("%s _simGetDynamicMotorUpperLimitVelocity\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicMotorReflectedPositionFromDynamicEngine==0)
	{
		printf("%s _simSetDynamicMotorReflectedPositionFromDynamicEngine\n",couldNotFind);
		return 0;
	}
	if (_simSetJointSphericalTransformation==0)
	{
		printf("%s _simSetJointSphericalTransformation\n",couldNotFind);
		return 0;
	}
	if (_simAddForceSensorCumulativeForcesAndTorques==0)
	{
		printf("%s _simAddForceSensorCumulativeForcesAndTorques\n",couldNotFind);
		return 0;
	}
	if (_simAddJointCumulativeForcesOrTorques==0)
	{
		printf("%s _simAddJointCumulativeForcesOrTorques\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicJointLocalTransformationPart2==0)
	{
		printf("%s _simSetDynamicJointLocalTransformationPart2\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicForceSensorLocalTransformationPart2==0)
	{
		printf("%s _simSetDynamicForceSensorLocalTransformationPart2\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicJointLocalTransformationPart2IsValid==0)
	{
		printf("%s _simSetDynamicJointLocalTransformationPart2IsValid\n",couldNotFind);
		return 0;
	}
	if (_simSetDynamicForceSensorLocalTransformationPart2IsValid==0)
	{
		printf("%s _simSetDynamicForceSensorLocalTransformationPart2IsValid\n",couldNotFind);
		return 0;
	}
	if (_simGetGeomWrapFromGeomProxy==0)
	{
		printf("%s _simGetGeomWrapFromGeomProxy\n",couldNotFind);
		return 0;
	}
	if (_simGetLocalInertiaFrame==0)
	{
		printf("%s _simGetLocalInertiaFrame\n",couldNotFind);
		return 0;
	}
	if (_simGetPurePrimitiveType==0)
	{
		printf("%s _simGetPurePrimitiveType\n",couldNotFind);
		return 0;
	}
	if (_simIsGeomWrapGeometric==0)
	{
		printf("%s _simIsGeomWrapGeometric\n",couldNotFind);
		return 0;
	}
	if (_simIsGeomWrapConvex==0)
	{
		printf("%s _simIsGeomWrapConvex\n",couldNotFind);
		return 0;
	}
	if (_simGetGeometricCount==0)
	{
		printf("%s _simGetGeometricCount\n",couldNotFind);
		return 0;
	}
	if (_simGetAllGeometrics==0)
	{
		printf("%s _simGetAllGeometrics\n",couldNotFind);
		return 0;
	}
	if (_simGetPurePrimitiveSizes==0)
	{
		printf("%s _simGetPurePrimitiveSizes\n",couldNotFind);
		return 0;
	}
	if (_simMakeDynamicAnnouncement==0)
	{
		printf("%s _simMakeDynamicAnnouncement\n",couldNotFind);
		return 0;
	}
	if (_simGetVerticesLocalFrame==0)
	{
		printf("%s _simGetVerticesLocalFrame\n",couldNotFind);
		return 0;
	}
	if (_simGetHeightfieldData==0)
	{
		printf("%s _simGetHeightfieldData\n",couldNotFind);
		return 0;
	}
	if (_simGetCumulativeMeshes==0)
	{
		printf("%s _simGetCumulativeMeshes\n",couldNotFind);
		return 0;
	}
	if (_simGetOdeMaxContactFrictionCFMandERP==0)
	{
		printf("%s _simGetOdeMaxContactFrictionCFMandERP\n",couldNotFind);
		return 0;
	}
	if (_simGetBulletCollisionMargin==0)
	{
		printf("%s _simGetBulletCollisionMargin\n",couldNotFind);
		return 0;
	}
	if (_simGetBulletStickyContact==0)
	{
		printf("%s _simGetBulletStickyContact\n",couldNotFind);
		return 0;
	}
	if (_simGetBulletRestitution==0)
	{
		printf("%s _simGetBulletRestitution\n",couldNotFind);
		return 0;
	}
	if (_simGetMass==0)
	{
		printf("%s _simGetMass\n",couldNotFind);
		return 0;
	}
	if (_simGetPrincipalMomentOfInertia==0)
	{
		printf("%s _simGetPrincipalMomentOfInertia\n",couldNotFind);
		return 0;
	}
	if (_simGetDamping==0)
	{
		printf("%s _simGetDamping\n",couldNotFind);
		return 0;
	}
	if (_simGetFriction==0)
	{
		printf("%s _simGetFriction\n",couldNotFind);
		return 0;
	}
	if (_simGetGravity==0)
	{
		printf("%s _simGetGravity\n",couldNotFind);
		return 0;
	}
	if (_simGetTimeDiffInMs==0)
	{
		printf("%s _simGetTimeDiffInMs\n",couldNotFind);
		return 0;
	}
	if (_simDoEntitiesCollide==0)
	{
		printf("%s _simDoEntitiesCollide\n",couldNotFind);
		return 0;
	}
	if (_simGetDistanceBetweenEntitiesIfSmaller==0)
	{
		printf("%s _simGetDistanceBetweenEntitiesIfSmaller\n",couldNotFind);
		return 0;
	}
	if (_simHandleJointControl==0)
	{
		printf("%s _simHandleJointControl\n",couldNotFind);
		return 0;
	}
	if (_simHandleCustomContact==0)
	{
		printf("%s _simHandleCustomContact\n",couldNotFind);
		return 0;
	}
	if (_simGetPureHollowScaling==0)
	{
		printf("%s _simGetPureHollowScaling\n",couldNotFind);
		return 0;
	}
	if (_simGetVortexParameters==0)
	{
		printf("%s _simGetVortexParameters\n",couldNotFind);
		return 0;
	}
	if (_simGetJointCallbackCallOrder==0)
	{
		printf("%s _simGetJointCallbackCallOrder\n",couldNotFind);
		return 0;
	}
	if (_simGetNewtonParameters==0)
	{
		printf("%s _simGetNewtonParameters\n",couldNotFind);
		return 0;
	}
	return 1;
}

#endif // V_REP_LIBRARY
