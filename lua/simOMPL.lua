local simOMPL = loadPlugin 'simOMPL';

-- @fun setGoalStates set multiple goal states at once, equivalent to calling simOMPL.setGoalState, simOMPL.addGoalState, simOMPL.addGoalState...
-- @arg string taskHandle the handle of the task
-- @arg {type='table',size='1..*'} states a table of tables, one element for each goal state
-- @cats task, goal
function simOMPL.setGoalStates(taskHandle, states)
    simOMPL.setGoalState(taskHandle, states[1])
    for i = 2, #states do simOMPL.addGoalState(taskHandle, states[i]) end
end

-- @fun getPathStateCount get the number of states in the given path
-- @arg string taskHandle the handle of the task
-- @arg table.float path the path, as returned by simOMPL.getPath
-- @ret int count the number of states in the path
-- @cats path, state
function simOMPL.getPathStateCount(taskHandle, path)
    local n = simOMPL.getStateSpaceDimension(taskHandle)
    return #path / n
end

-- @fun getPathState extract the state at specified index from the given path
-- @arg string taskHandle the handle of the task
-- @arg table.float path the path, as returned by simOMPL.getPath
-- @arg int index the index, starting from 1
-- @ret table.float state a state extracted from the path
-- @cats path, state
function simOMPL.getPathState(taskHandle, path, index)
    if index == 0 then error('invalid index') end
    if index < 0 then index = simOMPL.getPathStateCount(taskHandle, path) + index + 1 end
    local n = simOMPL.getStateSpaceDimension(taskHandle)
    local s = {}
    for i = (index - 1) * n + 1, (index) * n do table.insert(s, path[i]) end
    return s
end

-- @fun getProjectedPathLength get the length of the path projected onto the default projection
-- @arg string taskHandle the handle of the task
-- @arg table.float path the path, as returned by simOMPL.getPath
-- @cats path, projection
function simOMPL.getProjectedPathLength(taskHandle, path)
    local m = simOMPL.projectionSize(taskHandle)
    local pathProjection = simOMPL.projectStates(taskHandle, path)
    local length = 0
    for i = 2, #pathProjection / m do
        local s = 0
        for j = 1, m do s = s + math.pow(path[i * m + j] - path[(i - 1) * m + j], 2) end
        length = length + math.sqrt(s)
    end
    return length
end

-- @fun getReversedPath reverse the given path
-- @arg string taskHandle the handle of the task
-- @arg table.float path the path, as returned by simOMPL.getPath
-- @ret table.float reversedPath the reversed path
-- @cats path
function simOMPL.getReversedPath(taskHandle, path)
    local n = simOMPL.getStateSpaceDimension(taskHandle)
    local p = {}
    for i = 1, #path / n do
        local ii = #path / n - i + 1
        for j = 1, n do table.insert(p, path[(ii - 1) * n + j]) end
    end
    return p
end

-- @fun projectionSize return the dimension of the projection
-- @arg string taskHandle the handle of the task
-- @ret int size of the projection
-- @cats projection
function simOMPL.projectionSize(taskHandle)
    local s = simOMPL.readState(taskHandle)
    local p = simOMPL.projectStates(taskHandle, s)
    return #p
end

function simOMPL.__projectionMustBe3D(taskHandle)
    if simOMPL.projectionSize(taskHandle) ~= 3 then
        error(
            'this functions works only with 3D projections (pass useForProjection=1 to createStateSpace wherever appropriate, or otherwise use setProjectionEvaluationCallback to specify a custom projection to map the state into a 3D point)'
        )
    end
end

-- @fun drawPath draw a solution path for the specified motion planning task (as lines)
-- @arg string taskHandle the handle of the task
-- @arg table.float path the path, as returned by simOMPL.getPath
-- @arg float lineSize size of the line (in pixels)
-- @arg {type='table',item_type='float',size=3} color color of the lines
-- @arg int extraAttributes extra attributes to pass to sim.addDrawingObject
-- @ret table.int dwos a table of handles of new drawing objects
-- @cats path, drawing
function simOMPL.drawPath(taskHandle, path, lineSize, color, extraAttributes)
    simOMPL.__projectionMustBe3D(taskHandle)
    lineSize = lineSize or 2
    parentObjectHandle = -1
    color = color or {1, 0, 0}
    extraAttributes = extraAttributes or 0
    sim.setStepping(true)
    local dwoPath = sim.addDrawingObject(
                        sim.drawing_lines + extraAttributes, lineSize, 0, parentObjectHandle, 99999,
                        color
                    )
    local pathProjection = simOMPL.projectStates(taskHandle, path)
    for i = 4, #pathProjection, 3 do
        local d = {}
        for j = -3, 2 do table.insert(d, pathProjection[i + j]) end
        sim.addDrawingObjectItem(dwoPath, d)
    end
    sim.setStepping(false)
    return {dwoPath}
end

-- @fun drawPlannerData draw planner data (graph) extracted from the specified motion planning task
-- @arg string taskHandle handle of the task
-- @arg float pointSize size of nodes (in meters)
-- @arg float lineSize size of lines (in pixels)
-- @arg {type='table',item_type='float',size=3} color color of nodes and lines
-- @arg {type='table',item_type='float',size=3} startColor color of start nodes
-- @arg {type='table',item_type='float',size=3} goalColor color of goal nodes
-- @ret table.int dwos a table of handles of new drawing objects
-- @cats drawing
function simOMPL.drawPlannerData(taskHandle, pointSize, lineSize, color, startColor, goalColor)
    simOMPL.__projectionMustBe3D(taskHandle)
    local states1, tags, tagsReal, edges, edgeWeights, startVertices, goalVertices =
        simOMPL.getPlannerData(
            taskHandle
        )
    local states = simOMPL.projectStates(taskHandle, states1)
    pointSize = pointSize or 0.02
    lineSize = lineSize or 2
    color = color or {0.5, 0.5, 0.5}
    startColor = startColor or {0.5, 0.5, 0.5}
    goalColor = goalColor or {0.5, 0.5, 0.5}
    local dupTol = 0
    local parentHandle = -1
    local maxItemCnt = 999999
    sim.setStepping(true)
    local dwoPoints = sim.addDrawingObject(
                          sim.drawing_spherepoints, pointSize, dupTol, parentHandle, maxItemCnt,
                          color
                      )
    local dwoLines = sim.addDrawingObject(
                         sim.drawing_lines, lineSize, dupTol, parentHandle, maxItemCnt, color
                     )
    local dwoStart = sim.addDrawingObject(
                         sim.drawing_spherepoints, pointSize * 1.5, dupTol, parentHandle,
                         maxItemCnt, startColor
                     )
    local dwoGoal = sim.addDrawingObject(
                        sim.drawing_spherepoints, pointSize * 1.5, dupTol, parentHandle, maxItemCnt,
                        goalColor
                    )
    for i = 1, #states, 3 do
        local p = {states[i + 0], states[i + 1], states[i + 2]}
        sim.addDrawingObjectItem(dwoPoints, p)
    end
    for i = 1, #edges, 2 do
        local l = {
            states[3 * edges[i + 0] + 1], states[3 * edges[i + 0] + 2],
            states[3 * edges[i + 0] + 3], states[3 * edges[i + 1] + 1],
            states[3 * edges[i + 1] + 2], states[3 * edges[i + 1] + 3],
        }
        sim.addDrawingObjectItem(dwoLines, l)
    end
    for i = 1, #startVertices do
        local p = {
            states[3 * startVertices[i] + 1], states[3 * startVertices[i] + 2],
            states[3 * startVertices[i] + 3],
        }
        sim.addDrawingObjectItem(dwoStart, p)
    end
    for i = 1, #goalVertices do
        local p = {
            states[3 * goalVertices[i] + 1], states[3 * goalVertices[i] + 2],
            states[3 * goalVertices[i] + 3],
        }
        sim.addDrawingObjectItem(dwoGoal, p)
    end
    sim.setStepping(false)
    return {dwoPoints, dwoLines, dwoStart, dwoGoal}
end

-- @fun removeDrawingObjects remove the drawing objects created with related functions
-- @arg string taskHandle handle of the task
-- @arg table.int dwos table of handles to drawing objects, as returned by the functions
-- @cats drawing
function simOMPL.removeDrawingObjects(taskHandle, dwos)
    for i, ob in pairs(dwos) do sim.removeDrawingObject(ob) end
end

-- @fun createStateSpaceForJoint convenience function that wraps simOMPL.createStateSpace
-- @arg string name name of the state space
-- @arg int jointHandle handle of the joint
-- @arg {type='int',default=0} useForProjection use for projection
-- @arg {type='float',default=1} weight weight
-- @ret int ssHandle handle of the state space
-- @cats state
function simOMPL.createStateSpaceForJoint(name, jointHandle, useForProjection, weight)
    local cyclic, interval = sim.getJointInterval(jointHandle)
    return simOMPL.createStateSpace(
               name, cyclic and simOMPL.StateSpaceType.cyclic_joint_position or
                   simOMPL.StateSpaceType.joint_position, jointHandle,
               cyclic and {} or {interval[1]}, cyclic and {} or {interval[1] + interval[2]},
               useForProjection, weight
           )
end

-- @fun setStateSpaceForJoints convenience function that wraps simOMPL.setStateSpace
-- @arg string taskHandle the handle of the task
-- @arg table.int jointHandles handles of the joints
-- @arg {type='table',item_type='int',default='{}'} useForProjection use for projection, same size as jointHandles
-- @arg {type='table',item_type='float',default='{}'} weight weights, same size as jointHandles
-- @cats state
function simOMPL.setStateSpaceForJoints(taskHandle, jointHandles, useForProjection, weights)
    local ss = {}
    for i, jointHandle in ipairs(jointHandles) do
        table.insert(
            ss, simOMPL.createStateSpaceForJoint(
                table.unpack {
                    sim.getObjectAlias(jointHandle, 4), jointHandle, useForProjection[i],
                    weights[i],
                }
            )
        )
    end
    simOMPL.setStateSpace(taskHandle, ss)
end

(require 'simOMPL-typecheck')(simOMPL)

return simOMPL
