------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim_childscriptcall_initialization) then 
 
end 
if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 
if (sim_call_type==sim_childscriptcall_sensing) then 
    if not firstTimeHere93846738 then 
        firstTimeHere93846738=0 
    end 
    simSetScriptAttribute(sim_handle_self,sim_scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738+1 
 
------------------------------------------------------------------------------ 
robot_name = simGetScriptSimulationParameter(sim_handle_self,'name')
function toString(name)
    return robot_name..'/'..name
end

function prefix(name)
    return robot_name..'_'..name
end
 
if (simGetScriptExecutionCount()==0) then
        visionSensor1Handle=simGetObjectHandle("SICK_S300_sensor1")
        visionSensor2Handle=simGetObjectHandle("SICK_S300_sensor2")
        joint1Handle=simGetObjectHandle("SICK_S300_joint1")
        joint2Handle=simGetObjectHandle("SICK_S300_joint2")
        sensorRefHandle=simGetObjectHandle("laser") -- laser_ref
    
        maxScanDistance=simGetScriptSimulationParameter(sim_handle_self,'maxScanDistance')
        if maxScanDistance>1000 then maxScanDistance=1000 end
        if maxScanDistance<0.1 then maxScanDistance=0.1 end
        simSetObjectFloatParameter(visionSensor1Handle,1001,maxScanDistance)
        simSetObjectFloatParameter(visionSensor2Handle,1001,maxScanDistance)
        maxScanDistance_=maxScanDistance*0.9999
    
        scanningAngle=simGetScriptSimulationParameter(sim_handle_self,'scanAngle')
        if scanningAngle>270 then scanningAngle=270 end
        if scanningAngle<2 then scanningAngle=2 end
        scanningAngle=scanningAngle*math.pi/180
        simSetObjectFloatParameter(visionSensor1Handle,1004,scanningAngle/2)
        simSetObjectFloatParameter(visionSensor2Handle,1004,scanningAngle/2)
    
        simSetJointPosition(joint1Handle,-scanningAngle/4)
        simSetJointPosition(joint2Handle,scanningAngle/4)
        red={1,0,0}
        lines=simAddDrawingObject(sim_drawing_lines,1,0,-1,1000,nil,nil,nil,red)
    
        if (simGetInt32Parameter(sim_intparam_program_version)<30004) then
            simDisplayDialog("ERROR","This version of the SICK sensor is only supported from V-REP V3.0.4 and upwards.&&nMake sure to update your V-REP.",sim_dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
    end
    
    measuredData={}
    
    if (simGetScriptExecutionCount()~=0) then
        -- We skip the very first reading
        simAddDrawingObjectItem(lines,nil)
        showLines=simGetScriptSimulationParameter(sim_handle_self,'showLaserSegments')
        r,t1,u1=simReadVisionSensor(visionSensor1Handle)
        r,t2,u2=simReadVisionSensor(visionSensor2Handle)
    
        m1=simGetObjectMatrix(visionSensor1Handle,-1)
        m01=simGetInvertedMatrix(simGetObjectMatrix(sensorRefHandle,-1))
        m01=simMultiplyMatrices(m01,m1)
        m2=simGetObjectMatrix(visionSensor2Handle,-1)
        m02=simGetInvertedMatrix(simGetObjectMatrix(sensorRefHandle,-1))
        m02=simMultiplyMatrices(m02,m2)
        if u1 then
            p={0,0,0}
            p=simMultiplyVector(m1,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u1[2]-1,1 do
                for i=0,u1[1]-1,1 do
                    w=2+4*(j*u1[1]+i)
                    v1=u1[w+1]
                    v2=u1[w+2]
                    v3=u1[w+3]
                    v4=u1[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=simMultiplyVector(m01,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                    else
                        p={v1,v2,v3}
                        p=simMultiplyVector(m01,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])                        
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=simMultiplyVector(m1,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        simAddDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        if u2 then
            p={0,0,0}
            p=simMultiplyVector(m2,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u2[2]-1,1 do
                for i=0,u2[1]-1,1 do
                    w=2+4*(j*u2[1]+i)
                    v1=u2[w+1]
                    v2=u2[w+2]
                    v3=u2[w+3]
                    v4=u2[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=simMultiplyVector(m02,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])
                    else
                        p={v1,v2,v3}
                        p=simMultiplyVector(m02,p)
                        table.insert(measuredData,p[1])
                        table.insert(measuredData,p[2])
                        table.insert(measuredData,p[3])                        
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=simMultiplyVector(m2,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        simAddDrawingObjectItem(lines,t)
                    end
                end
            end
        end
    end
    
    -- measuredData now contains all the points that are closer than the sensor range
    -- For each point there is the x, y and z coordinate (i.e. 3 number for each point)
    -- Coordinates are expressed relative to the sensor frame.
    -- You can access this data from outside via various mechanisms. The best is to first
    -- pack the data, then to send it as a string. For example:
    --
    -- 
    data=simPackFloats(measuredData)
    simSetStringSignal(prefix('laser_data'), data)
    --
    -- Then in a different location:
    -- data=simGetStringSignal("measuredDataAtThisTime")
    -- measuredData=simUnpackFloats(data)
    --
    --
    -- Of course you can also send the data via tubes, wireless (simTubeOpen, etc., simSendData, etc.)
    --
    -- Also, if you send the data via string signals, if you you cannot read the data in each simulation
    -- step, then always append the data to an already existing signal data, e.g.
    --
    -- 
    --data=simPackFloats(measuredData)
    --existingData=simGetStringSignal("laser_data")
    --if existingData then
    --    data=existingData..data
    --end
    --simSetStringSignal("laser_data",data)
    
    
    
    
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 

