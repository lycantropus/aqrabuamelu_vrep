-- This script is in charge of moving the legs in order to walk

if (sim_call_type==sim_childscriptcall_initialization) then 
    antBase=simGetObjectHandle('hexa_legBase')
    legTips={-1,-1,-1,-1,-1,-1}
    legTargets={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        legTips[i]=simGetObjectHandle('hexa_footTip'..i-1)
        legTargets[i]=simGetObjectHandle('hexa_footTarget'..i-1)
    end
    initialPos={nil,nil,nil,nil,nil,nil}
    for i=1,6,1 do
        initialPos[i]=simGetObjectPosition(legTips[i],antBase)
    end
    legMovementIndex={1,4,2,6,3,5}
    stepProgression=0
    stepVelocity=0.5
    stepAmplitude=0.16
    stepHeight=0.04
    movementStrength=1
    realMovementStrength=0
    movementDirection=0*math.pi/180
    rotation=0    
end 

if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 

if (sim_call_type==sim_childscriptcall_actuation) then 
    dt=simGetSimulationTimeStep()
    
    stepVelocity=simGetScriptSimulationParameter(sim_handle_self,'stepVelocity')
    stepAmplitude=simGetScriptSimulationParameter(sim_handle_self,'stepAmplitude')
    stepHeight=simGetScriptSimulationParameter(sim_handle_self,'stepHeight')
    movementDirection=math.pi*simGetScriptSimulationParameter(sim_handle_self,'movementDirection')/180
    rotation=simGetScriptSimulationParameter(sim_handle_self,'rotationMode')
    movementStrength=simGetScriptSimulationParameter(sim_handle_self,'movementStrength')
    dx=movementStrength-realMovementStrength
    if (math.abs(dx)>dt*0.1) then
        dx=math.abs(dx)*dt*0.5/dx
    end
    realMovementStrength=realMovementStrength+dx
    
    
    for leg=1,6,1 do
        sp=(stepProgression+(legMovementIndex[leg]-1)/6) % 1
        offset={0,0,0}
        if (sp<(1/3)) then
            offset[1]=sp*3*stepAmplitude/2
        else
            if (sp<(1/3+1/6)) then
                s=sp-1/3
                offset[1]=stepAmplitude/2-stepAmplitude*s*6/2
                offset[3]=s*6*stepHeight
            else
                if (sp<(2/3)) then
                    s=sp-1/3-1/6
                    offset[1]=-stepAmplitude*s*6/2
                    offset[3]=(1-s*6)*stepHeight
                else
                    s=sp-2/3
                    offset[1]=-stepAmplitude*(1-s*3)/2
                end
            end
        end
        md=movementDirection+math.abs(rotation)*math.atan2(initialPos[leg][1]*rotation,-initialPos[leg][2]*rotation)
        offset2={offset[1]*math.cos(md)*realMovementStrength,offset[1]*math.sin(md)*realMovementStrength,offset[3]*realMovementStrength}
        p={initialPos[leg][1]+offset2[1],initialPos[leg][2]+offset2[2],initialPos[leg][3]+offset2[3]}
        simSetObjectPosition(legTargets[leg],antBase,p) 
    end
    
    stepProgression=stepProgression+dt*stepVelocity
end 
