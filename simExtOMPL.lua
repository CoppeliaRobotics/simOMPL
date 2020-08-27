local simOMPL={}

function simOMPL.getPathStateCount(taskHandle,path)
    local n=simOMPL.getStateSpaceDimension(taskHandle)
    return #path/n
end

function simOMPL.getPathState(taskHandle,path,index)
    local n=simOMPL.getStateSpaceDimension(taskHandle)
    local s={}
    for i=(index-1)*n+1,(index)*n do table.insert(s,path[i]) end
    return s
end

function simOMPL.drawPath(taskHandle,path,lineSize,parentObjectHandle,color,extraAttributes)
    lineSize=lineSize or 2
    parentObjectHandle=parentObjectHandle or -1
    color=color or {1,0,0}
    extraAttributes=extraAttributes or 0
    sim.setThreadAutomaticSwitch(false)
    local dwoPath=sim.addDrawingObject(sim.drawing_lines+extraAttributes,lineSize,0,parentObjectHandle,99999,{1,0,0})
    local pathProjection=simOMPL.projectStates(taskHandle,path)
    for i=4,#pathProjection,3 do
        local d={}
        for j=-3,2 do table.insert(d, pathProjection[i+j]) end
        sim.addDrawingObjectItem(dwoPath,d)
    end
    sim.setThreadAutomaticSwitch(true)
    return dwoPath
end

function simOMPL.drawPlannerData(task,pointSize,lineSize,color,startColor,goalColor)
    local states1,tags,tagsReal,edges,edgeWeights,startVertices,goalVertices=simOMPL.getPlannerData(task)
    local states=simOMPL.projectStates(task,states1)
    pointSize=pointSize or 0.02
    lineSize=lineSize or 2
    color=color or {0.5,0.5,0.5}
    startColor=startColor or {0.5,0.5,0.5}
    goalColor=goalColor or {0.5,0.5,0.5}
    local dupTol=0
    local parentHandle=-1
    local maxItemCnt=999999
    sim.setThreadAutomaticSwitch(false)
    local dwoPoints=sim.addDrawingObject(sim.drawing_spherepoints,pointSize,dupTol,parentHandle,maxItemCnt,color)
    local dwoLines=sim.addDrawingObject(sim.drawing_lines,lineSize,dupTol,parentHandle,maxItemCnt,color)
    local dwoStart=sim.addDrawingObject(sim.drawing_spherepoints,pointSize*1.5,dupTol,parentHandle,maxItemCnt,startColor)
    local dwoGoal=sim.addDrawingObject(sim.drawing_spherepoints,pointSize*1.5,dupTol,parentHandle,maxItemCnt,goalColor)
    for i=1,#states,3 do
        local p={states[i+0],states[i+1],states[i+2]}
        sim.addDrawingObjectItem(dwoPoints,p)
    end
    for i=1,#edges,2 do
        local l={states[3*edges[i+0]+1],states[3*edges[i+0]+2],states[3*edges[i+0]+3],
                 states[3*edges[i+1]+1],states[3*edges[i+1]+2],states[3*edges[i+1]+3]}
        sim.addDrawingObjectItem(dwoLines,l)
    end
    for i=1,#startVertices do
        local p={states[3*startVertices[i]+1],states[3*startVertices[i]+2],states[3*startVertices[i]+3]}
        sim.addDrawingObjectItem(dwoStart,p)
    end
    for i=1,#goalVertices do
        local p={states[3*goalVertices[i]+1],states[3*goalVertices[i]+2],states[3*goalVertices[i]+3]}
        sim.addDrawingObjectItem(dwoGoal,p)
    end
    sim.setThreadAutomaticSwitch(true)
    return dwoPoints,dwoLines,dwoStart,dwoGoal
end

return simOMPL
