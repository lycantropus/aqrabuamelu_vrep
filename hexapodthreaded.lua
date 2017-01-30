setStepMode=function(stepVelocity,stepAmplitude,stepHeight,movementDirection,rotationMode,movementStrength)
    simSetScriptSimulationParameter(sim_handle_tree,'stepVelocity',stepVelocity)
    simSetScriptSimulationParameter(sim_handle_tree,'stepAmplitude',stepAmplitude)
    simSetScriptSimulationParameter(sim_handle_tree,'stepHeight',stepHeight)
    simSetScriptSimulationParameter(sim_handle_tree,'movementDirection',movementDirection)
    simSetScriptSimulationParameter(sim_handle_tree,'rotationMode',rotationMode)
    simSetScriptSimulationParameter(sim_handle_tree,'movementStrength',movementStrength)
end

moveBody=function(index)
    local p={initialP[1],initialP[2],initialP[3]}
    local o={initialO[1],initialO[2],initialO[3]}
    simMoveToPosition(legBase,antBase,p,o,vel,accel)
    if (index==0) then
        -- up/down
        p[3]=p[3]-0.03*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel*2,accel)
        p[3]=p[3]+0.03*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel*2,accel)
    end
    if (index==1) then
        -- 4x twisting
        o[1]=o[1]+5*math.pi/180
        o[2]=o[2]-05*math.pi/180
        o[3]=o[3]-15*math.pi/180
        p[1]=p[1]-0.03*sizeFactor
        p[2]=p[2]+0.015*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[1]=o[1]-10*math.pi/180
        o[3]=o[3]+30*math.pi/180
        p[2]=p[2]-0.04*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[1]=o[1]+10*math.pi/180
        o[2]=o[2]+10*math.pi/180
        p[2]=p[2]+0.03*sizeFactor
        p[1]=p[1]+0.06*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[1]=o[1]-10*math.pi/180
        o[3]=o[3]-30*math.pi/180
        p[2]=p[2]-0.03*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
    end
    if (index==2) then
        -- rolling
        p[3]=p[3]-0.0*sizeFactor
        o[1]=o[1]+17*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[1]=o[1]-34*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[1]=o[1]+17*math.pi/180
        p[3]=p[3]+0.0*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
    end
    if (index==3) then
        -- pitching
        p[3]=p[3]-0.0*sizeFactor
        o[2]=o[2]+15*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[2]=o[2]-30*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[2]=o[2]+15*math.pi/180
        p[3]=p[3]+0.0*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
    end
    if (index==4) then
        -- yawing
        p[3]=p[3]+0.0*sizeFactor
        o[3]=o[3]+30*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[3]=o[3]-60*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        o[3]=o[3]+30*math.pi/180
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
        p[3]=p[3]-0.0*sizeFactor
        simMoveToPosition(legBase,antBase,p,o,vel,accel)
    end
    if (index==5) then
    -- adapt
        --p[3]=p[3]-0.01*sizeFactor
        o[1]=o[1]+-15*math.pi/180
        --o[1]=o[1]+-15*math.pi/180
        simMoveToPosition(legBase,antBase,nill,o,vel,accel)
    end
end

readFrontalSensors=function()
    proxResult0, proxDistance0, proxDetectedPoint0, proxDetectedObjectHandle0, proxDetectedSurfaceNV0 = simHandleProximitySensor(proxSensor0)
    proxResult1, proxDistance1, proxDetectedPoint1, proxDetectedObjectHandle1, proxDetectedSurfaceNV1 = simHandleProximitySensor(proxSensor1)
end

readBaseSensors=function()
    proxResultFR, proxDistanceFR, proxDetectedPointFR, proxDetectedObjectHandleFR, proxDetectedSurfaceNVFR = simHandleProximitySensor(baseSensorFR)
end

antBase=simGetObjectHandle('hexa_base')
legBase=simGetObjectHandle('hexa_legBase')
sizeFactor=simGetObjectSizeFactor(antBase)
proxSensor0=simGetObjectHandle('Proximity_sensor0')
proxSensor1=simGetObjectHandle('Proximity_sensor1')
baseSensorFR= simGetObjectHandle('Body_sensorFR')
vel=0.05
accel=0.05
initialP={0,0,0}
initialO={0,0,0}
initialP[3]=initialP[3]-0.03*sizeFactor
simMoveToPosition(legBase,antBase,initialP,initialO,vel,accel)

stepHeight=0.02*sizeFactor
maxWalkingStepSize=0.11*sizeFactor
walkingVel=0.1



--footTip=simGetObjectHandle('hexa_footTip0')
proxResult0=1
proxResult1=1
readFrontalSensors()
readBaseSensors()
oldProxDetected0 = proxDetectedPoint0[3]
oldProxDetected1 = proxDetectedPoint1[3]
oldDiffR=0
oldError=0
counter=0;
reference=0.09

moveBody(5)
while proxResult0 == 1 or proxResult1 == 1 do
    readFrontalSensors()
    readBaseSensors()
    
    if proxDetectedPoint0 ~= nil and proxDetectedPoint1 ~= nil then
        oldProxDetected0 = proxDetectedPoint0[3]
        oldProxDetected1 = proxDetectedPoint1[3]
    end
        
    if proxDetectedPointFR ~= nil then
        --height = proxDetectedPointFR[3]
        --print ('x: ' .. proxDetectedSurfaceNVFR[1] .. ' y: ' .. proxDetectedSurfaceNVFR[2] .. ' z: ' .. proxDetectedSurfaceNVFR[3])
        normalVectorFront={proxDetectedSurfaceNVFR[1], proxDetectedSurfaceNVFR[3]}
        normalVectorSide={proxDetectedSurfaceNVFR[2], proxDetectedSurfaceNVFR[3]}
        normNVFront=math.sqrt((normalVectorFront[1]* normalVectorFront[1]) + (normalVectorFront[2]*normalVectorFront[2]))
        normNVSide=math.sqrt((normalVectorSide[1]* normalVectorSide[1]) + (normalVectorSide[2]*normalVectorSide[2]))
        
        if normNVFront > 1 then
            normNVFront=1
        end
        if normNVSide > 1 then
            normNVSide=1
        end
        
        
        angleFront= math.acos(normNVFront)
        angleSide= math.acos(normNVSide)

        Pk=1

        valueFront=Pk*angleFront
        valueSide=Pk*angleSide
        print('value side: ' .. valueSide)
        o={initialO[1]+0.2*angleSide,initialO[2]+valueFront,initialO[3]}
        
        simMoveToPosition(legBase,antBase,nil,o,vel,accel)

    end
    counter=counter+1

    
    setStepMode(walkingVel,maxWalkingStepSize*0.5,stepHeight*5  ,0,0,1)
    simWait(1)
    
    
    
    
end
setStepMode(walkingVel*0.0,maxWalkingStepSize*0.2,stepHeight,0,0,0)
            
