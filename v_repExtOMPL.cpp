// Copyright (c) 2015, Federico Ferri
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// -------------------------------------------------------------------
// Authors:
// Federico Ferri <federico.ferri.it at gmail dot com>
// -------------------------------------------------------------------

#include "v_repExtOMPL.h"
#include "luaFunctionData.h"
#include "v_repLib.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#ifdef _WIN32
    #ifdef QT_COMPIL
        #include <direct.h>
    #else
        #include <shlwapi.h>
        #pragma comment(lib, "Shlwapi.lib")
    #endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#define _stricmp strcasecmp
#endif /* __linux || __APPLE__ */

#define CONCAT(x, y, z) x y z
#define strConCat(x, y, z)    CONCAT(x, y, z)

#define PLUGIN_VERSION 2 // 2 since version 3.2.1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#include <ompl/base/Goal.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/StateSpace.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/geometric/SimpleSetup.h>

#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
//#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/TRRT.h>

std::string luaTypeToString(simInt x)
{
    switch(x)
    {
    case sim_lua_arg_bool: return "sim_lua_arg_bool";
    case sim_lua_arg_int: return "sim_lua_arg_int";
    case sim_lua_arg_float: return "sim_lua_arg_float";
    case sim_lua_arg_double: return "sim_lua_arg_double";
    case sim_lua_arg_string: return "sim_lua_arg_string";
    case sim_lua_arg_charbuff: return "sim_lua_arg_charbuff";
    case sim_lua_arg_nil: return "sim_lua_arg_nil";
    case sim_lua_arg_table: return "sim_lua_arg_table";
    case sim_lua_arg_invalid: return "sim_lua_arg_invalid";
    }
    if(x & sim_lua_arg_nil)
        return luaTypeToString(x & ~sim_lua_arg_nil) + "|" + luaTypeToString(sim_lua_arg_nil);
    if(x & sim_lua_arg_table)
        return luaTypeToString(x & ~sim_lua_arg_table) + "|" + luaTypeToString(sim_lua_arg_table);
    return "???";
}

std::string luaCallbackToString(SLuaCallBack *c)
{
    std::stringstream ss;
    ss << "{inputArgsTypeAndSize: [";
    for(int i = 0; i < c->inputArgCount; i++)
    {
        ss << (i ? ", " : "") << luaTypeToString(c->inputArgTypeAndSize[2*i]);
        if(c->inputArgTypeAndSize[1+2*i]) ss << "_" << c->inputArgTypeAndSize[1+2*i];
    }
    ss << "], outputArgsTypeAndSize: [";
    for(int i = 0; i < c->outputArgCount; i++)
    {
        ss << (i ? ", " : "") << luaTypeToString(c->outputArgTypeAndSize[2*i]);
        if(c->outputArgTypeAndSize[1+2*i]) ss << "_" << c->outputArgTypeAndSize[1+2*i];
    }
    ss << "]}";
    return ss.str();
}

namespace ob = ompl::base;
namespace og = ompl::geometric;

enum StateSpaceType
{
    simx_ompl_statespacetype_position2d = 50001,
    simx_ompl_statespacetype_pose2d,
    simx_ompl_statespacetype_position3d,
    simx_ompl_statespacetype_pose3d,
    simx_ompl_statespacetype_joint_position
};

enum Algorithm
{
    simx_ompl_algorithm_BiTRRT = 30001,
    simx_ompl_algorithm_BITstar,
    simx_ompl_algorithm_BKPIECE1,
    simx_ompl_algorithm_CForest,
    simx_ompl_algorithm_EST,
    simx_ompl_algorithm_FMT,
    simx_ompl_algorithm_KPIECE1,
    simx_ompl_algorithm_LazyPRM,
    simx_ompl_algorithm_LazyPRMstar,
    simx_ompl_algorithm_LazyRRT,
    simx_ompl_algorithm_LBKPIECE1,
    simx_ompl_algorithm_LBTRRT,
    //simx_ompl_algorithm_LightningRetrieveRepair,
    simx_ompl_algorithm_PDST,
    simx_ompl_algorithm_PRM,
    simx_ompl_algorithm_PRMstar,
    simx_ompl_algorithm_pRRT,
    simx_ompl_algorithm_pSBL,
    simx_ompl_algorithm_RRT,
    simx_ompl_algorithm_RRTConnect,
    simx_ompl_algorithm_RRTstar,
    simx_ompl_algorithm_SBL,
    simx_ompl_algorithm_SPARS,
    simx_ompl_algorithm_SPARStwo,
    simx_ompl_algorithm_STRIDE,
    simx_ompl_algorithm_TRRT
};

struct ObjectDefHeader
{
    // internal handle of this object (used by the plugin):
    simInt handle;
    // name of this object:
    std::string name;
    // objects created during simulation will be destroyed when simulation terminates:
    bool destroyAfterSimulationStop;
};

struct StateSpaceDef
{
    ObjectDefHeader header;
    // type of this state space:
    StateSpaceType type;
    // V-REP handle of the object (objects, or joint if type = joint_position):
    simInt objectHandle;
    // lower bounds of search space:
    std::vector<simFloat> boundsLow;
    // upper bounds of search space:
    std::vector<simFloat> boundsHigh;
    // use this state space as the default projection:
    bool defaultProjection;
};

struct RobotDef
{
    ObjectDefHeader header;
    // handles of object that will be checked for collision:
    std::vector<simInt> collisionHandles;
    // state space is a composition of elementary state spaces (internal handles to StateSpaceDef objects):
    std::vector<simInt> stateSpaces;
};

struct TaskDef
{
    ObjectDefHeader header;
    // internal handle of the robot object associated with the task:
    simInt robotHandle;
    // handle of the obstacles that will be checked for collision:
    std::vector<simInt> obstacleHandles;
    // start state:
    std::vector<simFloat> startState;
    // goal can be specified in different ways:
    struct Goal
    {
        enum {STATE, DUMMY_PAIR, CALLBACK} type;
        // goal state:
        std::vector<simFloat> state;
        // goal dummy pair:
        struct {simInt goalDummy, robotDummy;} dummyPair;
        // goal callback:
        struct {std::string function; simInt scriptId;} callback;
    } goal;
    // state validation:
    struct StateValidation
    {
        enum {DEFAULT, CALLBACK} type;
        // state validation callback:
        struct {std::string function; simInt scriptId;} callback;
    } stateValidation;
    // projection evaluation:
    struct ProjectionEvaluation
    {
        enum {DEFAULT, CALLBACK} type;
        // projection evaluation callback:
        struct {std::string function; simInt scriptId; int dim;} callback;
    } projectionEvaluation;
    // search algorithm to use:
    Algorithm algorithm;
    // pointer to OMPL state space. will be valid only during planning (i.e. only valid for Lua callbacks)
    ob::StateSpacePtr stateSpacePtr;
};

std::map<simInt, TaskDef *> tasks;
std::map<simInt, RobotDef *> robots;
std::map<simInt, StateSpaceDef *> statespaces;
simInt nextTaskHandle = 1000;
simInt nextRobotHandle = 4000;
simInt nextStateSpaceHandle = 9000;

// this function will be called at simulation end to destroy objects that
// were created during simulation, which otherwise would leak indefinitely:
template<typename T>
void destroyTransientObjects(std::map<simInt, T *>& c)
{
    std::vector<simInt> transientObjects;

    for(typename std::map<simInt, T *>::const_iterator it = c.begin(); it != c.end(); ++it)
    {
        if(it->second->header.destroyAfterSimulationStop)
            transientObjects.push_back(it->first);
    }

    for(size_t i = 0; i < transientObjects.size(); i++)
    {
        simInt key = transientObjects[i];
        T *t = c[key];
        c.erase(key);
        delete t;
    }
}

void destroyTransientObjects()
{
    destroyTransientObjects(tasks);
    destroyTransientObjects(robots);
    destroyTransientObjects(statespaces);
}

class ProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    ProjectionEvaluator(const ob::StateSpacePtr& space, TaskDef *task)
        : ob::ProjectionEvaluator(space), statespace(space)
    {
        this->task = task;

        dim = 0;

        RobotDef *robot = robots[task->robotHandle];

        switch(task->projectionEvaluation.type)
        {
            case TaskDef::ProjectionEvaluation::DEFAULT:
                switch(task->goal.type)
                {
                    case TaskDef::Goal::STATE:
                    case TaskDef::Goal::CALLBACK:
                        dim = defaultProjectionSize();
                        break;
                    case TaskDef::Goal::DUMMY_PAIR:
                        dim = dummyPairProjectionSize();
                        break;
                    default:
                        // this will never happen
                        dim = 0;
                        break;
                }
                break;
            case TaskDef::ProjectionEvaluation::CALLBACK:
                dim = luaProjectCallbackSize();
                break;
            default:
                // this will never happen
                dim = 0;
                break;
        }
    }

    virtual unsigned int getDimension(void) const
    {
        return dim;
    }

    virtual void defaultCellSizes(void)
    {
        // TODO: handle this in the plugin API
        cellSizes_.resize(dim);
        for(int i = 0; i < dim; i++)
            cellSizes_[i] = 0.05;
    }

    virtual void project(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        for(int i = 0; i < dim; i++)
            projection(i) = 0.0;

        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        RobotDef *robot = robots[task->robotHandle];

        switch(task->projectionEvaluation.type)
        {
            case TaskDef::ProjectionEvaluation::DEFAULT:
                switch(task->goal.type)
                {
                    case TaskDef::Goal::STATE:
                    case TaskDef::Goal::CALLBACK:
                        defaultProjection(state, projection);
                        break;
                    case TaskDef::Goal::DUMMY_PAIR:
                        dummyPairProjection(state, projection);
                        break;
                }
                break;
            case TaskDef::ProjectionEvaluation::CALLBACK:
                luaProjectCallback(state, projection);
                break;
        }
    }

protected:
    virtual int defaultProjectionSize() const
    {
        RobotDef *robot = robots[task->robotHandle];

        for(int i = 0; i < robot->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[robot->stateSpaces[i]];

            if(!stateSpace->defaultProjection) continue;

            switch(stateSpace->type)
            {
                case simx_ompl_statespacetype_pose2d:
                case simx_ompl_statespacetype_position2d:
                    return 2;
                case simx_ompl_statespacetype_pose3d:
                case simx_ompl_statespacetype_position3d:
                    return 3;
                case simx_ompl_statespacetype_joint_position:
                    return 1;
            }
        }

        return 0;
    }

    virtual void defaultProjection(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        RobotDef *robot = robots[task->robotHandle];
        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        for(int i = 0; i < robot->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[robot->stateSpaces[i]];

            if(!stateSpace->defaultProjection) continue;

            switch(stateSpace->type)
            {
                case simx_ompl_statespacetype_pose2d:
                    projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                    projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                    break;
                case simx_ompl_statespacetype_pose3d:
                    projection(0) = s->as<ob::SE3StateSpace::StateType>(i)->getX();
                    projection(1) = s->as<ob::SE3StateSpace::StateType>(i)->getY();
                    projection(2) = s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                    break;
                case simx_ompl_statespacetype_position2d:
                    projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    break;
                case simx_ompl_statespacetype_position3d:
                    projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    projection(2) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                    break;
                case simx_ompl_statespacetype_joint_position:
                    projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    break;
            }

            break;
        }
    }

    virtual int dummyPairProjectionSize() const
    {
        return 3;
    }

    virtual void dummyPairProjection(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, -1, &pos[0]);
        projection(0) = pos[0];
        projection(1) = pos[1];
        projection(2) = pos[2];
    }

    virtual int luaProjectCallbackSize() const
    {
        return task->projectionEvaluation.callback.dim;
    }

    virtual void luaProjectCallback(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        RobotDef *robot = robots[task->robotHandle];
        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        // The expected return arguments (2):
        const int outArgs[]={1, sim_lua_arg_float|sim_lua_arg_table, luaProjectCallbackSize()};

        SLuaCallBack c;
        CLuaFunctionData D;

        // Prepare the input arguments:
        std::vector<float> stateVecf;
        for(int i = 0; i < stateVec.size(); i++)
            stateVecf.push_back(stateVec[i]);
        D.pushOutData_luaFunctionCall(CLuaFunctionDataItem(stateVecf));
        D.writeDataToLua_luaFunctionCall(&c, outArgs);

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->projectionEvaluation.callback.scriptId, task->projectionEvaluation.callback.function.c_str(), &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->projectionEvaluation.callback.function.c_str()))
            {
                std::cout << "ProjectionEvaluator::luaProjectCallback - Lua callback " << task->projectionEvaluation.callback.function << " returned ";
                std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                for(int i = 0; i < outData->at(0).floatData.size(); i++)
                {
                    projection(i) = outData->at(0).floatData[i];
                    std::cout << (i ? ", " : "") << outData->at(0).floatData[i];
                }
                std::cout << std::endl;
            }
            else
            {
                throw ompl::Exception("Projection evaliator callback " + task->projectionEvaluation.callback.function + " return value size and/or type is incorrect");
            }
        }
        else
        {
            throw ompl::Exception("Projection evaliator callback " + task->projectionEvaluation.callback.function + " returned an error");
        }

        // Release the data:
        D.releaseBuffers_luaFunctionCall(&c);
    }

    TaskDef *task;
    const ob::StateSpacePtr& statespace;
    int dim;
};

class StateSpace : public ob::CompoundStateSpace
{
public:
    StateSpace(TaskDef *task)
        : ob::CompoundStateSpace(), task(task)
    {
        setName("VREPCompoundStateSpace");
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;

        RobotDef *robot = robots[task->robotHandle];

        for(int i = 0; i < robot->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[robot->stateSpaces[i]];

            ob::StateSpacePtr subSpace;

            switch(stateSpace->type)
            {
                case simx_ompl_statespacetype_pose2d:
                    subSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                    break;
                case simx_ompl_statespacetype_pose3d:
                    subSpace = ob::StateSpacePtr(new ob::SE3StateSpace());
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                    break;
                case simx_ompl_statespacetype_position2d:
                    subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
                    break;
                case simx_ompl_statespacetype_position3d:
                    subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
                    break;
                case simx_ompl_statespacetype_joint_position:
                    subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
                    break;
            }

            subSpace->setName(stateSpace->header.name);
            addSubspace(subSpace, 1.0);

            // set bounds:

            ob::RealVectorBounds bounds(stateSpace->boundsLow.size());;
            for(int j = 0; j < stateSpace->boundsLow.size(); j++)
                bounds.setLow(j, stateSpace->boundsLow[j]);
            for(int j = 0; j < stateSpace->boundsHigh.size(); j++)
                bounds.setHigh(j, stateSpace->boundsHigh[j]);

            switch(stateSpace->type)
            {
                case simx_ompl_statespacetype_pose2d:
                    as<ob::SE2StateSpace>(i)->setBounds(bounds);
                    break;
                case simx_ompl_statespacetype_pose3d:
                    as<ob::SE3StateSpace>(i)->setBounds(bounds);
                    break;
                case simx_ompl_statespacetype_position2d:
                    as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                    break;
                case simx_ompl_statespacetype_position3d:
                    as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                    break;
                case simx_ompl_statespacetype_joint_position:
                    as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                    break;
            }
        }
    }

    // writes state s to V-REP:
    void writeState(const ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        RobotDef *robot = robots[task->robotHandle];

        int j = 0;
        simFloat pos[3], orient[4], value;

        for(int i = 0; i < robot->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[robot->stateSpaces[i]];

            switch(stateSpace->type)
            {
                case simx_ompl_statespacetype_pose2d:
                    simGetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    simGetObjectQuaternion(stateSpace->objectHandle, -1, &orient[0]);
                    pos[0] = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                    pos[1] = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                    orient[0] = s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                    // FIXME: make correct quaternion for 2d orientation!
                    simSetObjectQuaternion(stateSpace->objectHandle, -1, &orient[0]);
                    simSetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    break;
                case simx_ompl_statespacetype_pose3d:
                    pos[0] = s->as<ob::SE3StateSpace::StateType>(i)->getX();
                    pos[1] = s->as<ob::SE3StateSpace::StateType>(i)->getY();
                    pos[2] = s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                    orient[0] = s->as<ob::SE3StateSpace::StateType>(i)->rotation().x;
                    orient[1] = s->as<ob::SE3StateSpace::StateType>(i)->rotation().y;
                    orient[2] = s->as<ob::SE3StateSpace::StateType>(i)->rotation().z;
                    orient[3] = s->as<ob::SE3StateSpace::StateType>(i)->rotation().w;
                    simSetObjectQuaternion(stateSpace->objectHandle, -1, &orient[0]);
                    simSetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    break;
                case simx_ompl_statespacetype_position2d:
                    simGetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    pos[0] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    pos[1] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    simSetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    break;
                case simx_ompl_statespacetype_position3d:
                    pos[0] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    pos[1] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    pos[2] = s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                    simSetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    break;
                case simx_ompl_statespacetype_joint_position:
                    value = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    simSetJointPosition(stateSpace->objectHandle, value);
                    break;
            }
        }
    }

    // reads state s from V-REP:
    void readState(ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        RobotDef *robot = robots[task->robotHandle];

        simFloat pos[3], orient[4], value;

        for(int i = 0; i < robot->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[robot->stateSpaces[i]];

            switch(stateSpace->type)
            {
                case simx_ompl_statespacetype_pose2d:
                    simGetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    simGetObjectQuaternion(stateSpace->objectHandle, -1, &orient[0]);
                    s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                    s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[0]);
                    // FIXME: make correct quaternion for 2d orientation!
                    break;
                case simx_ompl_statespacetype_pose3d:
                    simGetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    simGetObjectQuaternion(stateSpace->objectHandle, -1, &orient[0]);
                    s->as<ob::SE3StateSpace::StateType>(i)->setXYZ(pos[0], pos[1], pos[2]);
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().x = orient[0];
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().y = orient[1];
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().z = orient[2];
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().w = orient[3];
                    break;
                case simx_ompl_statespacetype_position2d:
                    simGetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                    break;
                case simx_ompl_statespacetype_position3d:
                    simGetObjectPosition(stateSpace->objectHandle, -1, &pos[0]);
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[2] = pos[2];
                    break;
                case simx_ompl_statespacetype_joint_position:
                    simGetJointPosition(stateSpace->objectHandle, &value);
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = value;
                    break;
            }
        }
    }

protected:
    TaskDef *task;
};

class StateValidityChecker : public ob::StateValidityChecker
{
public:
    StateValidityChecker(const ob::SpaceInformationPtr &si, TaskDef *task)
        : ob::StateValidityChecker(si), statespace(si->getStateSpace()), task(task)
    {
    }

    virtual ~StateValidityChecker()
    {
    }

    virtual bool isValid(const ob::State *state) const
    {
        switch(task->stateValidation.type)
        {
        case TaskDef::StateValidation::DEFAULT:
            return checkDefault(state);
        case TaskDef::StateValidation::CALLBACK:
            return checkCallback(state);
        }
        return false;
    }

protected:
    virtual bool checkDefault(const ob::State *state) const
    {
        RobotDef *robot = robots[task->robotHandle];

        //ob::CompoundStateSpace *ss = statespace->as<ob::CompoundStateSpace>();
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        // check collisions:
        bool inCollision = false;
        for(int i = 0; i < task->obstacleHandles.size(); i++)
        {
            for(int j = 0; j < robot->collisionHandles.size(); j++)
            {
                int r = simCheckCollision(robot->collisionHandles[j], task->obstacleHandles[i]);
                if(r == 1)
                {
                    inCollision = true;
                    break;
                }
            }
            if(inCollision) break;
        }

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);

        return !inCollision;
    }

    virtual bool checkCallback(const ob::State *state) const
    {
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        bool ret = false;

        // The expected return arguments (2):
        const int outArgs[]={1, sim_lua_arg_bool, 0};

        SLuaCallBack c;
        CLuaFunctionData D;

        // Prepare the input arguments:
        std::vector<float> stateVecf;
        for(int i = 0; i < stateVec.size(); i++)
            stateVecf.push_back(stateVec[i]);
        D.pushOutData_luaFunctionCall(CLuaFunctionDataItem(stateVecf));
        D.writeDataToLua_luaFunctionCall(&c, outArgs);

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->stateValidation.callback.scriptId, task->stateValidation.callback.function.c_str(), &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->stateValidation.callback.function.c_str()))
            {
                std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                ret = outData->at(0).boolData[0];
                std::cout << "StateValidityChecker::checkCallback - Lua callback " << task->stateValidation.callback.function << " returned " << ret << std::endl;
            }
            else
            {
                throw ompl::Exception("State validation callback " + task->stateValidation.callback.function + " return value size and/or type is incorrect");
            }
        }
        else
        {
            throw ompl::Exception("State validation callback " + task->stateValidation.callback.function + " returned an error");
        }

        // Release the data:
        D.releaseBuffers_luaFunctionCall(&c);

        return ret;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
};

class Goal : public ob::Goal
{
public:
    Goal(const ob::SpaceInformationPtr &si, TaskDef *task, double tolerance = 1e-3)
        : ob::Goal(si), statespace(si->getStateSpace()), task(task), tolerance(tolerance)
    {
    }

    virtual bool isSatisfied(const ob::State *state) const
    {
        double distance = 0.0;
        return isSatisfied(state, &distance);
    }

    virtual bool isSatisfied(const ob::State *state, double *distance) const
    {
        switch(task->goal.type)
        {
        case TaskDef::Goal::STATE:
            // silence -Wswitch warning
            // if really type is STATE we are not using this class for goal check
            return false;
        case TaskDef::Goal::DUMMY_PAIR:
            return checkDummyPair(state, distance);
        case TaskDef::Goal::CALLBACK:
            return checkCallback(state, distance);
        }

        return false;
    }

protected:
    virtual bool checkDummyPair(const ob::State *state, double *distance) const
    {
        ob::ScopedState<ob::CompoundStateSpace> s(statespace);
        s = state;

        // save old state:
        ob::ScopedState<ob::CompoundStateSpace> s_old(statespace);
        statespace->as<StateSpace>()->readState(s_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        simFloat goalPos[3], robotPos[3];
        simGetObjectPosition(task->goal.dummyPair.goalDummy, -1, &goalPos[0]);
        simGetObjectPosition(task->goal.dummyPair.robotDummy, -1, &robotPos[0]);
        *distance = sqrt(pow(goalPos[0] - robotPos[0], 2) + pow(goalPos[1] - robotPos[1], 2) + pow(goalPos[2] - robotPos[2], 2));
        bool satisfied = *distance <= tolerance;

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);

        return satisfied;
    }

    virtual bool checkCallback(const ob::State *state, double *distance) const
    {
        std::cout << "***** thread id inside lua function caller: " << simGetThreadId() << std::endl;

        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        double dist = std::numeric_limits<double>::infinity();
        bool ret = false;

        // The expected return arguments (2):
        const int outArgs[]={2, sim_lua_arg_bool, 0, sim_lua_arg_float, 0};

        SLuaCallBack c;
        CLuaFunctionData D;

        // Prepare the input arguments:
        std::vector<float> stateVecf;
        for(int i = 0; i < stateVec.size(); i++)
            stateVecf.push_back(stateVec[i]);
        D.pushOutData_luaFunctionCall(CLuaFunctionDataItem(stateVecf));
        D.writeDataToLua_luaFunctionCall(&c, outArgs);

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->goal.callback.scriptId, task->goal.callback.function.c_str(), &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->goal.callback.function.c_str()))
            {
                std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                ret = outData->at(0).boolData[0];
                dist = outData->at(1).floatData[0];
                std::cout << "Goal::checkCallback - Lua callback " << task->goal.callback.function << " returned " << ret << ", " << dist << std::endl;
            }
            else
            {
                throw ompl::Exception("Goal callback " + task->goal.callback.function + " return value size and/or type is incorrect (callback: " + luaCallbackToString(&c) + ")");
            }
        }
        else
        {
            throw ompl::Exception("Goal callback " + task->goal.callback.function + " returned an error");
        }

        // Release the data:
        D.releaseBuffers_luaFunctionCall(&c);

        *distance = dist;
        return ret;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
    double tolerance;
};

#define LUA_CREATE_STATE_SPACE_DESCR "Create a component of the state space for the motion planning problem."
#define LUA_CREATE_STATE_SPACE_PARAMS "name: a name for this state space" \
    "|type: the type of this state space component (must be one of simx_ompl_statespacetype_position2d, simx_ompl_statespacetype_pose2d, simx_ompl_statespacetype_position3d, simx_ompl_statespacetype_pose3d, simx_ompl_statespacetype_joint_position)" \
    "|objectHandle: the object handle (a joint object if type is simx_ompl_statespacetype_joint_position, otherwise a shape)" \
    "|boundsLow: lower bounds (if type is pose, specify only the 3 position components)" \
    "|boundsHigh: upper bounds (if type is pose, specify only the 3 position components)" \
    "|useForProjection: if true, this object position or joint value will be used for computing a default projection"
#define LUA_CREATE_STATE_SPACE_RET "stateSpaceHandle: a handle to the created state space component"
#define LUA_CREATE_STATE_SPACE_COMMAND "simExtOMPL_createStateSpace"
#define LUA_CREATE_STATE_SPACE_APIHELP "number stateSpaceHandle=" LUA_CREATE_STATE_SPACE_COMMAND "(string name, number type, number objectHandle, table boundsLow, table boundsHigh, number useForProjection)"
const int inArgs_CREATE_STATE_SPACE[]={6, sim_lua_arg_string, 0, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 0, sim_lua_arg_float|sim_lua_arg_table, 0, sim_lua_arg_int, 0};

void LUA_CREATE_STATE_SPACE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_CREATE_STATE_SPACE, inArgs_CREATE_STATE_SPACE[0], LUA_CREATE_STATE_SPACE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        if(inData->at(3).floatData.size() != inData->at(4).floatData.size())
        {
            simSetLastError(LUA_CREATE_STATE_SPACE_COMMAND, "Lower and upper bounds must have the same length.");
            break;
        }

        std::string name = inData->at(0).stringData[0];
        simInt type = inData->at(1).intData[0];
        simInt objectHandle = inData->at(2).intData[0];
        StateSpaceDef *statespace = new StateSpaceDef();
        statespace->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
        statespace->header.handle = nextStateSpaceHandle++;
        statespace->header.name = name;
        statespace->type = static_cast<StateSpaceType>(type);
        statespace->objectHandle = objectHandle;
        for(int i = 0; i < inData->at(3).floatData.size(); i++)
            statespace->boundsLow.push_back(inData->at(3).floatData[i]);
        for(int i = 0; i < inData->at(4).floatData.size(); i++)
            statespace->boundsHigh.push_back(inData->at(4).floatData[i]);
        statespace->defaultProjection = inData->at(5).intData[0] > 0;
        statespaces[statespace->header.handle] = statespace;
        returnResult = statespace->header.handle;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_DESTROY_STATE_SPACE_DESCR "Destroy the spacified state space component.<br /><br />" \
    "Note: state space components created during simulation are automatically destroyed when simulation ends."
#define LUA_DESTROY_STATE_SPACE_PARAMS "stateSpaceHandle: handle to state space component"
#define LUA_DESTROY_STATE_SPACE_RET ""
#define LUA_DESTROY_STATE_SPACE_COMMAND "simExtOMPL_destroyStateSpace"
#define LUA_DESTROY_STATE_SPACE_APIHELP "number result=" LUA_DESTROY_STATE_SPACE_COMMAND "(number stateSpaceHandle)"
const int inArgs_DESTROY_STATE_SPACE[]={1, sim_lua_arg_int, 0};

void LUA_DESTROY_STATE_SPACE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_DESTROY_STATE_SPACE, inArgs_DESTROY_STATE_SPACE[0], LUA_DESTROY_STATE_SPACE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();
        simInt stateSpaceHandle = inData->at(0).intData[0];

        if(statespaces.find(stateSpaceHandle) == statespaces.end())
        {
            simSetLastError(LUA_DESTROY_STATE_SPACE_COMMAND, "Invalid state space handle handle.");
            break;
        }

        StateSpaceDef *statespace = statespaces[stateSpaceHandle];
        statespaces.erase(stateSpaceHandle);
        delete statespace;
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_CREATE_ROBOT_DESCR "Create a robot object. A robot object contains informations about: <ul>" \
    "<li> the state space components (created with <a href=\"#" LUA_CREATE_STATE_SPACE_COMMAND "\">" LUA_CREATE_STATE_SPACE_COMMAND "</a>)</li>" \
    "<li> the collision objects of the robot</li>" \
    "</ul>"
#define LUA_CREATE_ROBOT_PARAMS "name: a name for this robot object"
#define LUA_CREATE_ROBOT_RET "robotHandle: a handle to the created robot object"
#define LUA_CREATE_ROBOT_COMMAND "simExtOMPL_createRobot"
#define LUA_CREATE_ROBOT_APIHELP "number robotHandle=" LUA_CREATE_ROBOT_COMMAND "(string name)"
const int inArgs_CREATE_ROBOT[]={1, sim_lua_arg_string, 0};

void LUA_CREATE_ROBOT_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_CREATE_ROBOT, inArgs_CREATE_ROBOT[0], LUA_CREATE_ROBOT_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();
        std::string name = inData->at(0).stringData[0];
        RobotDef *robot = new RobotDef();
        robot->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
        robot->header.handle = nextRobotHandle++;
        robot->header.name = name;
        robots[robot->header.handle] = robot;
        returnResult = robot->header.handle;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_DESTROY_ROBOT_DESCR "Destroy the spacified robot object.<br /><br />" \
    "Note: robot objects created during simulation are automatically destroyed when simulation ends."
#define LUA_DESTROY_ROBOT_PARAMS "robotHandle: handle to the robot object to destroy"
#define LUA_DESTROY_ROBOT_RET ""
#define LUA_DESTROY_ROBOT_COMMAND "simExtOMPL_destroyRobot"
#define LUA_DESTROY_ROBOT_APIHELP "number result=" LUA_DESTROY_ROBOT_COMMAND "(number robotHandle)"
const int inArgs_DESTROY_ROBOT[]={1, sim_lua_arg_int, 0};

void LUA_DESTROY_ROBOT_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_DESTROY_ROBOT, inArgs_DESTROY_ROBOT[0], LUA_DESTROY_ROBOT_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();
        simInt robotHandle = inData->at(0).intData[0];

        if(robots.find(robotHandle) == robots.end())
        {
            simSetLastError(LUA_DESTROY_ROBOT_COMMAND, "Invalid robot handle.");
            break;
        }

        RobotDef *robot = robots[robotHandle];
        robots.erase(robotHandle);
        delete robot;
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_PARAM_ROBOT_HANDLE "handle to a robot object created with <a href=\"#" LUA_CREATE_ROBOT_COMMAND "\">" LUA_CREATE_ROBOT_COMMAND "</a>"
#define LUA_SET_STATE_SPACE_DESCR "Set the state space of this robot object."
#define LUA_SET_STATE_SPACE_PARAMS "robotHandle: " LUA_PARAM_ROBOT_HANDLE "" \
    "|stateSpaceHandles: a table of handles to state space components, created with <a href=\"#" LUA_CREATE_STATE_SPACE_COMMAND "\">" LUA_CREATE_STATE_SPACE_COMMAND "</a>"
#define LUA_SET_STATE_SPACE_RET ""
#define LUA_SET_STATE_SPACE_COMMAND "simExtOMPL_setStateSpace"
#define LUA_SET_STATE_SPACE_APIHELP "number result=" LUA_SET_STATE_SPACE_COMMAND "(number robotHandle, table stateSpaceHandles)"
const int inArgs_SET_STATE_SPACE[]={2, sim_lua_arg_int, 0, sim_lua_arg_int|sim_lua_arg_table, 0};

void LUA_SET_STATE_SPACE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_STATE_SPACE, inArgs_SET_STATE_SPACE[0], LUA_SET_STATE_SPACE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();
        simInt robotHandle = inData->at(0).intData[0];

        if(robots.find(robotHandle) == robots.end())
        {
            simSetLastError(LUA_SET_STATE_SPACE_COMMAND, "Invalid robot handle.");
            break;
        }

        bool valid_statespace_handles = true;

        for(int i = 0; i < inData->at(1).intData.size(); i++)
        {
            simInt stateSpaceHandle = inData->at(1).intData[i];

            if(statespaces.find(stateSpaceHandle) == statespaces.end())
            {
                valid_statespace_handles = false;
                break;
            }
        }

        if(!valid_statespace_handles)
        {
            simSetLastError(LUA_SET_STATE_SPACE_COMMAND, "Invalid state space handle.");
            break;
        }

        RobotDef *robot = robots[robotHandle];
        robot->stateSpaces.clear();
        for(int i = 0; i < inData->at(1).intData.size(); i++)
            robot->stateSpaces.push_back(inData->at(1).intData[i]);
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_COLLISION_OBJECTS_DESCR "Set the collision objects for this robot object. Collision object are used to compute collisions between robot and environment in the default state validity checker function."
#define LUA_SET_COLLISION_OBJECTS_PARAMS "robotHandle: " LUA_PARAM_ROBOT_HANDLE "" \
    "|objectHandles: a table of handle to V-REP objects (shapes)"
#define LUA_SET_COLLISION_OBJECTS_RET ""
#define LUA_SET_COLLISION_OBJECTS_COMMAND "simExtOMPL_setCollisionObjects"
#define LUA_SET_COLLISION_OBJECTS_APIHELP "number result=" LUA_SET_COLLISION_OBJECTS_COMMAND "(number robotHandle, table objectHandles)"
const int inArgs_SET_COLLISION_OBJECTS[]={2, sim_lua_arg_int, 0, sim_lua_arg_int|sim_lua_arg_table, 0};

void LUA_SET_COLLISION_OBJECTS_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_COLLISION_OBJECTS, inArgs_SET_COLLISION_OBJECTS[0], LUA_SET_COLLISION_OBJECTS_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();
        simInt robotHandle = inData->at(0).intData[0];

        if(robots.find(robotHandle) == robots.end())
        {
            simSetLastError(LUA_SET_COLLISION_OBJECTS_COMMAND, "Invalid robot handle.");
            break;
        }

        RobotDef *robot = robots[robotHandle];
        robot->collisionHandles.clear();
        for(int i = 0; i < inData->at(1).intData.size(); i++)
            robot->collisionHandles.push_back(inData->at(1).intData[i]);
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_CREATE_TASK_DESCR "Create a task object, used to represent the motion planning task. A task object contains informations about: <ul>" \
    "<li>environment (i.e. which shapes are to be considered obstacles by the default state validity checker)</li>" \
    "<li>robot object (created with <a href=\"#" LUA_CREATE_ROBOT_COMMAND "\">" LUA_CREATE_ROBOT_COMMAND "</a>)</li>" \
    "<li>start state</li>" \
    "<li>goal state, or goal specification (e.g. pair of dummies, Lua callback, ...)</li>" \
    "<li>various Lua callbacks (projection evaluation, state validation, goal satisfaction)</li>" \
    "</ul>"
#define LUA_CREATE_TASK_PARAMS "name: a name for this task object"
#define LUA_CREATE_TASK_RET "taskHandle: a handle to the created task object"
#define LUA_CREATE_TASK_COMMAND "simExtOMPL_createTask"
#define LUA_CREATE_TASK_APIHELP "number taskHandle=" LUA_CREATE_TASK_COMMAND "(string name)"
const int inArgs_CREATE_TASK[]={1, sim_lua_arg_string, 0};

void LUA_CREATE_TASK_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_CREATE_TASK, inArgs_CREATE_TASK[0], LUA_CREATE_TASK_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        std::string name = inData->at(0).stringData[0];
        TaskDef *task = new TaskDef();
        task->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
        task->header.handle = nextTaskHandle++;
        task->header.name = name;
        task->goal.type = TaskDef::Goal::STATE;
        task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
        task->algorithm = simx_ompl_algorithm_KPIECE1;
        tasks[task->header.handle] = task;
        returnResult = task->header.handle;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_PARAM_TASK_HANDLE "a handle to a task object created with <a href=\"#" LUA_CREATE_TASK_COMMAND "\">" LUA_CREATE_TASK_COMMAND "</a>"
#define LUA_DESTROY_TASK_DESCR "Destroy the specified task object.<br /><br />" \
    "NOTE: task objects created during simulation are automatically destroyed when simulation ends"
#define LUA_DESTROY_TASK_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE
#define LUA_DESTROY_TASK_RET ""
#define LUA_DESTROY_TASK_COMMAND "simExtOMPL_destroyTask"
#define LUA_DESTROY_TASK_APIHELP "number result=" LUA_DESTROY_TASK_COMMAND "(number taskHandle)"
const int inArgs_DESTROY_TASK[]={1, sim_lua_arg_int, 0};

void LUA_DESTROY_TASK_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_DESTROY_TASK, inArgs_DESTROY_TASK[0], LUA_DESTROY_TASK_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_DESTROY_TASK_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        tasks.erase(taskHandle);
        delete task;
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

const char * state_space_type_string(StateSpaceType type)
{
    switch(type)
    {
        case simx_ompl_statespacetype_position2d: return "simx_ompl_statespacetype_position2d";
        case simx_ompl_statespacetype_pose2d: return "simx_ompl_statespacetype_pose2d";
        case simx_ompl_statespacetype_position3d: return "simx_ompl_statespacetype_position3d";
        case simx_ompl_statespacetype_pose3d: return "simx_ompl_statespacetype_pose3d";
        case simx_ompl_statespacetype_joint_position: return "simx_ompl_statespacetype_joint_position";
        default: return "???";
    }
};

const char * algorithm_string(Algorithm alg)
{
    switch(alg)
    {
        case simx_ompl_algorithm_BiTRRT: return "simx_ompl_algorithm_BiTRRT";
        case simx_ompl_algorithm_BITstar: return "simx_ompl_algorithm_BITstar";
        case simx_ompl_algorithm_BKPIECE1: return "simx_ompl_algorithm_BKPIECE1";
        case simx_ompl_algorithm_CForest: return "simx_ompl_algorithm_CForest";
        case simx_ompl_algorithm_EST: return "simx_ompl_algorithm_EST";
        case simx_ompl_algorithm_FMT: return "simx_ompl_algorithm_FMT";
        case simx_ompl_algorithm_KPIECE1: return "simx_ompl_algorithm_KPIECE1";
        case simx_ompl_algorithm_LazyPRM: return "simx_ompl_algorithm_LazyPRM";
        case simx_ompl_algorithm_LazyPRMstar: return "simx_ompl_algorithm_LazyPRMstar";
        case simx_ompl_algorithm_LazyRRT: return "simx_ompl_algorithm_LazyRRT";
        case simx_ompl_algorithm_LBKPIECE1: return "simx_ompl_algorithm_LBKPIECE1";
        case simx_ompl_algorithm_LBTRRT: return "simx_ompl_algorithm_LBTRRT";
        //case simx_ompl_algorithm_LightningRetrieveRepair: return "simx_ompl_algorithm_LightningRetrieveRepair";
        case simx_ompl_algorithm_PDST: return "simx_ompl_algorithm_PDST";
        case simx_ompl_algorithm_PRM: return "simx_ompl_algorithm_PRM";
        case simx_ompl_algorithm_PRMstar: return "simx_ompl_algorithm_PRMstar";
        case simx_ompl_algorithm_pRRT: return "simx_ompl_algorithm_pRRT";
        case simx_ompl_algorithm_pSBL: return "simx_ompl_algorithm_pSBL";
        case simx_ompl_algorithm_RRT: return "simx_ompl_algorithm_RRT";
        case simx_ompl_algorithm_RRTConnect: return "simx_ompl_algorithm_RRTConnect";
        case simx_ompl_algorithm_RRTstar: return "simx_ompl_algorithm_RRTstar";
        case simx_ompl_algorithm_SBL: return "simx_ompl_algorithm_SBL";
        case simx_ompl_algorithm_SPARS: return "simx_ompl_algorithm_SPARS";
        case simx_ompl_algorithm_SPARStwo: return "simx_ompl_algorithm_SPARStwo";
        case simx_ompl_algorithm_STRIDE: return "simx_ompl_algorithm_STRIDE";
        case simx_ompl_algorithm_TRRT: return "simx_ompl_algorithm_TRRT";
        default: return "???";
    }
};

#define LUA_PRINT_TASK_INFO_DESCR "Print a summary of the specified task object, including information about robot and its state spaces."
#define LUA_PRINT_TASK_INFO_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE
#define LUA_PRINT_TASK_INFO_RET ""
#define LUA_PRINT_TASK_INFO_COMMAND "simExtOMPL_printTaskInfo"
#define LUA_PRINT_TASK_INFO_APIHELP "number result=" LUA_PRINT_TASK_INFO_COMMAND "(number taskHandle)"
const int inArgs_PRINT_TASK_INFO[]={1, sim_lua_arg_int, 0};

void LUA_PRINT_TASK_INFO_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_PRINT_TASK_INFO, inArgs_PRINT_TASK_INFO[0], LUA_PRINT_TASK_INFO_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_PRINT_TASK_INFO_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        RobotDef *robot = robots[task->robotHandle];

        std::stringstream s;
        std::string prefix = "OMPL: ";
        s << prefix << "task name: " << task->header.name << std::endl;
        s << prefix << "robot: " << task->robotHandle << std::endl;
        s << prefix << "    name: " << robot->header.name << std::endl;
        s << prefix << "    collidables: {";
        for(int i = 0; i < robot->collisionHandles.size(); i++)
            s << (i ? ", " : "") << robot->collisionHandles[i];
        s << "}" << std::endl;
        s << prefix << "    state spaces:" << std::endl;
        for(int i = 0; i < robot->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[robot->stateSpaces[i]];
            s << prefix << "        state space: " << stateSpace->header.handle << std::endl;
            s << prefix << "            name: " << stateSpace->header.name << std::endl;
            s << prefix << "            type: " << state_space_type_string(stateSpace->type) << std::endl;
            s << prefix << "            object handle: " << stateSpace->objectHandle << std::endl;
            s << prefix << "            bounds low: {";
            for(int i = 0; i < stateSpace->boundsLow.size(); i++)
                s << (i ? ", " : "") << stateSpace->boundsLow[i];
            s << "}" << std::endl;
            s << prefix << "            bounds high: {";
            for(int i = 0; i < stateSpace->boundsHigh.size(); i++)
                s << (i ? ", " : "") << stateSpace->boundsHigh[i];
            s << "}" << std::endl;
            s << prefix << "            default projection: " << (stateSpace->defaultProjection ? "true" : "false") << std::endl;
        }
        s << prefix << "obstacles: {";
        for(int i = 0; i < task->obstacleHandles.size(); i++)
            s << (i ? ", " : "") << task->obstacleHandles[i];
        s << "}" << std::endl;
        s << prefix << "start state: {";
        for(int i = 0; i < task->startState.size(); i++)
            s << (i ? ", " : "") << task->startState[i];
        s << "}" << std::endl;
        s << prefix << "goal:";
        switch(task->goal.type)
        {
        case TaskDef::Goal::STATE:
            s << std::endl;
            s << prefix << "    goal state: {";
            for(int i = 0; i < task->goal.state.size(); i++)
                s << (i ? ", " : "") << task->goal.state[i];
            s << "}" << std::endl;
            break;
        case TaskDef::Goal::DUMMY_PAIR:
            s << std::endl;
            s << prefix << "    robot dummy:" << task->goal.dummyPair.robotDummy << std::endl;
            s << prefix << "    goal dummy:" << task->goal.dummyPair.goalDummy << std::endl;
            break;
        case TaskDef::Goal::CALLBACK:
            s << std::endl;
            s << prefix << "    callback:" << std::endl;
            s << prefix << "        scriptId: " << task->goal.callback.scriptId << std::endl;
            s << prefix << "        function: " << task->goal.callback.function << std::endl;
            break;
        default:
            s << " ???" << std::endl;
            break;
        }
        s << prefix << "state validation:";
        switch(task->stateValidation.type)
        {
        case TaskDef::StateValidation::DEFAULT:
            s << " default" << std::endl;
            break;
        case TaskDef::StateValidation::CALLBACK:
            s << std::endl;
            s << prefix << "    callback:" << std::endl;
            s << prefix << "        scriptId: " << task->stateValidation.callback.scriptId << std::endl;
            s << prefix << "        function: " << task->stateValidation.callback.function << std::endl;
            break;
        default:
            s << " ???" << std::endl;
            break;
        }
        s << prefix << "projection evaluation:";
        switch(task->projectionEvaluation.type)
        {
        case TaskDef::ProjectionEvaluation::DEFAULT:
            s << " default" << std::endl;
            break;
        case TaskDef::ProjectionEvaluation::CALLBACK:
            s << std::endl;
            s << prefix << "    callback:" << std::endl;
            s << prefix << "        scriptId:" << task->projectionEvaluation.callback.scriptId << std::endl;
            s << prefix << "        function:" << task->projectionEvaluation.callback.function << std::endl;
            break;
        default:
            s << " ???" << std::endl;
            break;
        }

        simAddStatusbarMessage(s.str().c_str());
        std::cout << s.str();

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_ALGORITHM_DESCR "Set the search algorithm for the specified task. Default algorithm used is KPIECE1."
#define LUA_SET_ALGORITHM_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|algorithm: one of simx_ompl_algorithm_BiTRRT, simx_ompl_algorithm_BITstar, simx_ompl_algorithm_BKPIECE1, simx_ompl_algorithm_CForest, simx_ompl_algorithm_EST, simx_ompl_algorithm_FMT, simx_ompl_algorithm_KPIECE1, simx_ompl_algorithm_LazyPRM, simx_ompl_algorithm_LazyPRMstar, simx_ompl_algorithm_LazyRRT, simx_ompl_algorithm_LBKPIECE1, simx_ompl_algorithm_LBTRRT, simx_ompl_algorithm_PDST, simx_ompl_algorithm_PRM, simx_ompl_algorithm_PRMstar, simx_ompl_algorithm_pRRT, simx_ompl_algorithm_pSBL, simx_ompl_algorithm_RRT, simx_ompl_algorithm_RRTConnect, simx_ompl_algorithm_RRTstar, simx_ompl_algorithm_SBL, simx_ompl_algorithm_SPARS, simx_ompl_algorithm_SPARStwo, simx_ompl_algorithm_STRIDE, simx_ompl_algorithm_TRRT"
#define LUA_SET_ALGORITHM_RET ""
#define LUA_SET_ALGORITHM_COMMAND "simExtOMPL_setAlgorithm"
#define LUA_SET_ALGORITHM_APIHELP "number result=" LUA_SET_ALGORITHM_COMMAND "(number taskHandle, number algorithm)"
const int inArgs_SET_ALGORITHM[]={2, sim_lua_arg_int, 0, sim_lua_arg_int, 0};

void LUA_SET_ALGORITHM_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_ALGORITHM, inArgs_SET_ALGORITHM[0], LUA_SET_ALGORITHM_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        Algorithm algorithm = static_cast<Algorithm>(inData->at(1).intData[0]);

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_ALGORITHM_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        task->algorithm = algorithm;
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_ROBOT_DESCR "Set the robot object for the specified task."
#define LUA_SET_ROBOT_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|robotHandle: " LUA_PARAM_ROBOT_HANDLE
#define LUA_SET_ROBOT_RET ""
#define LUA_SET_ROBOT_COMMAND "simExtOMPL_setRobot"
#define LUA_SET_ROBOT_APIHELP "number result=" LUA_SET_ROBOT_COMMAND "(number taskHandle, number robotHandle)"
const int inArgs_SET_ROBOT[]={2, sim_lua_arg_int, 0, sim_lua_arg_int, 0};

void LUA_SET_ROBOT_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_ROBOT, inArgs_SET_ROBOT[0], LUA_SET_ROBOT_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        simInt robotHandle = inData->at(1).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_ROBOT_COMMAND, "Invalid task handle.");
            break;
        }

        if(robots.find(robotHandle) == robots.end())
        {
            simSetLastError(LUA_SET_ROBOT_COMMAND, "Invalid robot handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        task->robotHandle = robotHandle;
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_ENVIRONMENT_DESCR "Set the environment specification for the specified task object."
#define LUA_SET_ENVIRONMENT_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|obstacleHandles: a table of handles to V-REP objects (shapes)"
#define LUA_SET_ENVIRONMENT_RET ""
#define LUA_SET_ENVIRONMENT_COMMAND "simExtOMPL_setEnvironment"
#define LUA_SET_ENVIRONMENT_APIHELP "number result=" LUA_SET_ENVIRONMENT_COMMAND "(number taskHandle, table obstacleHandles)"
const int inArgs_SET_ENVIRONMENT[]={2, sim_lua_arg_int, 0, sim_lua_arg_int|sim_lua_arg_table, 0};

void LUA_SET_ENVIRONMENT_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_ENVIRONMENT, inArgs_SET_ENVIRONMENT[0], LUA_SET_ENVIRONMENT_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        int numHandles = inData->at(1).intData.size();
        std::vector<simInt>& obstacleHandles = inData->at(1).intData;

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_ENVIRONMENT_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        task->obstacleHandles.clear();
        for(int i = 0; i < numHandles; i++)
            task->obstacleHandles.push_back(obstacleHandles[i]);
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_PARAM_ROBOT_STATE "a table of numbers, whose size must be consistent with the robot's state space specified in this task object"
#define LUA_SET_START_STATE_DESCR "Set the start state for the specified task object."
#define LUA_SET_START_STATE_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|state: " LUA_PARAM_ROBOT_STATE
#define LUA_SET_START_STATE_RET ""
#define LUA_SET_START_STATE_COMMAND "simExtOMPL_setStartState"
#define LUA_SET_START_STATE_APIHELP "number result=" LUA_SET_START_STATE_COMMAND "(number taskHandle, table state)"
const int inArgs_SET_START_STATE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 3};

void LUA_SET_START_STATE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_START_STATE, inArgs_SET_START_STATE[0], LUA_SET_START_STATE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_START_STATE_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        task->startState.clear();
        for(int i = 0; i < inData->at(1).floatData.size(); i++)
            task->startState.push_back(inData->at(1).floatData[i]);
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_GOAL_STATE_DESCR "Set the goal state for the specified task object."
#define LUA_SET_GOAL_STATE_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|state: " LUA_PARAM_ROBOT_STATE
#define LUA_SET_GOAL_STATE_RET ""
#define LUA_SET_GOAL_STATE_COMMAND "simExtOMPL_setGoalState"
#define LUA_SET_GOAL_STATE_APIHELP "number result=" LUA_SET_GOAL_STATE_COMMAND "(number taskHandle, table state)"
const int inArgs_SET_GOAL_STATE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 3};

void LUA_SET_GOAL_STATE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_GOAL_STATE, inArgs_SET_GOAL_STATE[0], LUA_SET_GOAL_STATE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_GOAL_STATE_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        task->goal.type = TaskDef::Goal::STATE;
        task->goal.state.clear();
        for(int i = 0; i < inData->at(1).floatData.size(); i++)
            task->goal.state.push_back(inData->at(1).floatData[i]);
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_GOAL_DESCR "Set the goal for the specificed task object by a dummy pair. One of the two dummies is part of the robot. The other dummy is fixed in the environment. When the task is solved, the position or pose of the two dummies will (approximatively) be the same."
#define LUA_SET_GOAL_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|robotDummy: a dummy attached to the robot" \
    "|goalDummy: a dummy fixed in the environment, representing the goal pose/position"
#define LUA_SET_GOAL_RET ""
#define LUA_SET_GOAL_COMMAND "simExtOMPL_setGoal"
#define LUA_SET_GOAL_APIHELP "number result=" LUA_SET_GOAL_COMMAND "(number taskHandle, number robotDummy, number goalDummy)"
const int inArgs_SET_GOAL[]={3, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_int, 0};

void LUA_SET_GOAL_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_GOAL, inArgs_SET_GOAL[0], LUA_SET_GOAL_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_GOAL_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];
        task->goal.type = TaskDef::Goal::DUMMY_PAIR;
        task->goal.dummyPair.goalDummy = inData->at(1).intData[0];
        task->goal.dummyPair.robotDummy = inData->at(2).intData[0];
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_COMPUTE_DESCR "Use OMPL to find a solution for this motion planning task."
#define LUA_COMPUTE_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|maxTime: maximum time to use in seconds"
#define LUA_COMPUTE_RET "states: a table of states, representing the solution, from start to goal. States are specified linearly."
#define LUA_COMPUTE_COMMAND "simExtOMPL_compute"
#define LUA_COMPUTE_APIHELP "number result, table states=" LUA_COMPUTE_COMMAND "(number taskHandle, number maxTime)"
const int inArgs_COMPUTE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float, 0};

void LUA_COMPUTE_CALLBACK(SLuaCallBack* p)
{
    std::cout << "***** thread id inside lua compute callback: " << simGetThreadId() << std::endl;

    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;
    std::vector<simFloat> pathOut;

    do
    {
        if(!D.readDataFromLua(p, inArgs_COMPUTE, inArgs_COMPUTE[0], LUA_COMPUTE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        simFloat maxTime = inData->at(1).floatData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_COMPUTE_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(robots.find(task->robotHandle) == robots.end())
        {
            simSetLastError(LUA_COMPUTE_COMMAND, "Invalid robot handle.");
            break;
        }

        RobotDef *robot = robots[task->robotHandle];

        try
        {
            ob::StateSpacePtr space(new StateSpace(task));
            task->stateSpacePtr = space;
            ob::ProjectionEvaluatorPtr projectionEval(new ProjectionEvaluator(space, task));
            space->registerDefaultProjection(projectionEval);
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(setup.getSpaceInformation(), task)));

            ob::ScopedState<> start(space);
            // TODO: check if task->startState is set and valid
            for(int i = 0; i < task->startState.size(); i++)
                start[i] = task->startState[i];
            setup.setStartState(start);

            if(task->goal.type == TaskDef::Goal::STATE)
            {
                ob::ScopedState<> goal(space);
                // TODO: check if task->goal.state is set and valid
                for(int i = 0; i < task->goal.state.size(); i++)
                    goal[i] = task->goal.state[i];
                setup.setGoalState(goal);
            }
            else if(task->goal.type == TaskDef::Goal::DUMMY_PAIR || task->goal.type == TaskDef::Goal::CALLBACK)
            {
                ob::GoalPtr goal(new Goal(setup.getSpaceInformation(), task));
                setup.setGoal(goal);
            }

            ob::SpaceInformationPtr si = setup.getSpaceInformation();
            ob::PlannerPtr planner;
            bool validAlgorithm = true;
            switch(task->algorithm)
            {
                case simx_ompl_algorithm_BiTRRT:
                    planner = ob::PlannerPtr(new og::BiTRRT(si));
                    break;
                case simx_ompl_algorithm_BITstar:
                    planner = ob::PlannerPtr(new og::BITstar(si));
                    break;
                case simx_ompl_algorithm_BKPIECE1:
                    planner = ob::PlannerPtr(new og::BKPIECE1(si));
                    break;
                case simx_ompl_algorithm_CForest:
                    planner = ob::PlannerPtr(new og::CForest(si));
                    break;
                case simx_ompl_algorithm_EST:
                    planner = ob::PlannerPtr(new og::EST(si)); // needs projection
                    break;
                case simx_ompl_algorithm_FMT:
                    planner = ob::PlannerPtr(new og::FMT(si));
                    break;
                case simx_ompl_algorithm_KPIECE1:
                    planner = ob::PlannerPtr(new og::KPIECE1(si)); // needs projection
                    break;
                case simx_ompl_algorithm_LazyPRM:
                    planner = ob::PlannerPtr(new og::LazyPRM(si));
                    break;
                case simx_ompl_algorithm_LazyPRMstar:
                    planner = ob::PlannerPtr(new og::LazyPRMstar(si));
                    break;
                case simx_ompl_algorithm_LazyRRT:
                    planner = ob::PlannerPtr(new og::LazyRRT(si));
                    break;
                case simx_ompl_algorithm_LBKPIECE1:
                    planner = ob::PlannerPtr(new og::LBKPIECE1(si));
                    break;
                case simx_ompl_algorithm_LBTRRT:
                    planner = ob::PlannerPtr(new og::LBTRRT(si));
                    break;
                //case simx_ompl_algorithm_LightningRetrieveRepair:
                    //planner = ob::PlannerPtr(new og::LightningRetrieveRepair(si));
                    //break;
                case simx_ompl_algorithm_PDST:
                    planner = ob::PlannerPtr(new og::PDST(si)); // needs projection
                    break;
                case simx_ompl_algorithm_PRM:
                    planner = ob::PlannerPtr(new og::PRM(si));
                    break;
                case simx_ompl_algorithm_PRMstar:
                    planner = ob::PlannerPtr(new og::PRMstar(si));
                    break;
                case simx_ompl_algorithm_pRRT:
                    planner = ob::PlannerPtr(new og::pRRT(si));
                    break;
                case simx_ompl_algorithm_pSBL:
                    planner = ob::PlannerPtr(new og::pSBL(si));
                    break;
                case simx_ompl_algorithm_RRT:
                    planner = ob::PlannerPtr(new og::RRT(si));
                    break;
                case simx_ompl_algorithm_RRTConnect:
                    planner = ob::PlannerPtr(new og::RRTConnect(si));
                    break;
                case simx_ompl_algorithm_RRTstar:
                    planner = ob::PlannerPtr(new og::RRTstar(si));
                    break;
                case simx_ompl_algorithm_SBL:
                    planner = ob::PlannerPtr(new og::SBL(si)); // needs projection
                    break;
                case simx_ompl_algorithm_SPARS:
                    planner = ob::PlannerPtr(new og::SPARS(si));
                    break;
                case simx_ompl_algorithm_SPARStwo:
                    planner = ob::PlannerPtr(new og::SPARStwo(si));
                    break;
                case simx_ompl_algorithm_STRIDE:
                    planner = ob::PlannerPtr(new og::STRIDE(si));
                    break;
                case simx_ompl_algorithm_TRRT:
                    planner = ob::PlannerPtr(new og::TRRT(si));
                    break;
                default:
                    simSetLastError(LUA_COMPUTE_COMMAND, "Invalid motion planning algorithm.");
                    validAlgorithm = false;
                    break;
            }
            if(!validAlgorithm)
                break;
            setup.setPlanner(planner);
            std::cout << "COMPUTE: p->scriptID = " << p->scriptID << std::endl;
            std::cout << "COMPUTE: task->goal.callback.scriptId = " << task->goal.callback.scriptId << std::endl;
            ob::PlannerStatus solved = setup.solve(maxTime);
            if(solved)
            {
                //simAddStatusbarMessage("OMPL: found solution:");
                setup.simplifySolution();
                //std::stringstream s;
                og::PathGeometric& path = setup.getSolutionPath();
                //path.print(s);
                //simAddStatusbarMessage(s.str().c_str());
                //simAddStatusbarMessage("OMPL: interpolated:");
                path.interpolate();
                //path.print(s);
                //simAddStatusbarMessage(s.str().c_str());
                for(int i = 0; i < path.getStateCount(); i++)
                {
                    const ob::StateSpace::StateType *s = path.getState(i);
                    std::vector<double> v;
                    space->copyToReals(v, s);
                    for(int i = 0; i < v.size(); i++)
                        pathOut.push_back(v[i]);
                }
            }
            else
            {
                //simAddStatusbarMessage("OMPL: could not find solution.");
            }
            returnResult = 1;
        }
        catch(ompl::Exception& ex)
        {
            std::string s = "OMPL exception: ";
            s += ex.what();
            std::cout << s << std::endl;
            simSetLastError(LUA_COMPUTE_COMMAND, s.c_str());
        }
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.pushOutData(CLuaFunctionDataItem(pathOut));
    D.writeDataToLua(p);
}

#define LUA_READ_STATE_DESCR ""
#define LUA_READ_STATE_PARAMS ""
#define LUA_READ_STATE_RET ""
#define LUA_READ_STATE_COMMAND "simExtOMPL_readState"
#define LUA_READ_STATE_APIHELP "number result, table state=" LUA_READ_STATE_COMMAND "(number taskHandle)"
const int inArgs_READ_STATE[]={1, sim_lua_arg_int, 0};

void LUA_READ_STATE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;
    std::vector<simFloat> stateOut;

    do
    {
        if(!D.readDataFromLua(p, inArgs_READ_STATE, inArgs_READ_STATE[0], LUA_READ_STATE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_READ_STATE_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(robots.find(task->robotHandle) == robots.end())
        {
            simSetLastError(LUA_READ_STATE_COMMAND, "Invalid robot handle.");
            break;
        }

        RobotDef *robot = robots[task->robotHandle];

        simSetLastError(LUA_READ_STATE_COMMAND, "Method not implemented.");
        // this would need a pointer to the OMPL state space, which is not yet available at this point.

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.pushOutData(CLuaFunctionDataItem(stateOut));
    D.writeDataToLua(p);
}

#define LUA_WRITE_STATE_DESCR ""
#define LUA_WRITE_STATE_PARAMS ""
#define LUA_WRITE_STATE_RET ""
#define LUA_WRITE_STATE_COMMAND "simExtOMPL_writeState"
#define LUA_WRITE_STATE_APIHELP "number result=" LUA_WRITE_STATE_COMMAND "(number taskHandle, table state)"
const int inArgs_WRITE_STATE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 0};

void LUA_WRITE_STATE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_WRITE_STATE, inArgs_WRITE_STATE[0], LUA_WRITE_STATE_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_WRITE_STATE_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(robots.find(task->robotHandle) == robots.end())
        {
            simSetLastError(LUA_WRITE_STATE_COMMAND, "Invalid robot handle.");
            break;
        }

        RobotDef *robot = robots[task->robotHandle];

        simSetLastError(LUA_WRITE_STATE_COMMAND, "Method not implemented.");
        // this would need a pointer to the OMPL state space, which is not yet available at this point.

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_PROJ_EVAL_CB_DESCR "Set a custom projection evaluation. The argument of the callback will be a state, and the return value must be a table of numbers, with a size equal to the projectionSize argument, i.e.<br /><br />table projection=evaluateProjection(table state)"
#define LUA_SET_PROJ_EVAL_CB_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|callback: name of the Lua callback" \
    "|projectionSize: size of the projection (usually 2 or 3)"
#define LUA_SET_PROJ_EVAL_CB_RET ""
#define LUA_SET_PROJ_EVAL_CB_COMMAND "simExtOMPL_setProjectionEvaluationCallback"
#define LUA_SET_PROJ_EVAL_CB_APIHELP "number result=" LUA_SET_PROJ_EVAL_CB_COMMAND "(number taskHandle, string callback, number projectionSize)"
const int inArgs_SET_PROJ_EVAL_CB[]={3, sim_lua_arg_int, 0, sim_lua_arg_string, 0, sim_lua_arg_int, 0};

void LUA_SET_PROJ_EVAL_CB_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_PROJ_EVAL_CB, inArgs_SET_PROJ_EVAL_CB[0], LUA_SET_PROJ_EVAL_CB_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        std::string callback = inData->at(1).stringData[0];
        simInt projectionSize = inData->at(2).intData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_PROJ_EVAL_CB_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(projectionSize < 1)
        {
            simSetLastError(LUA_SET_PROJ_EVAL_CB_COMMAND, "Projection size must be positive.");
            break;
        }

        // TODO: validate scriptId argument

        if(callback == "")
        {
            task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
            task->projectionEvaluation.callback.dim = 0;
            task->projectionEvaluation.callback.scriptId = 0;
            task->projectionEvaluation.callback.function = "";
        }
        else
        {
            task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::CALLBACK;
            task->projectionEvaluation.callback.dim = projectionSize;
            task->projectionEvaluation.callback.scriptId = p->scriptID;
            task->projectionEvaluation.callback.function = callback;
        }

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_STATE_VAL_CB_DESCR "Set a custom state validation. By default state validation is performed by collision checking, between robot's collision objects and environment's objects. By specifying a custom state validation, it is possible to perform any arbitrary check on a state to determine wether it is valid or not. Argument to the callback is the state to validate, and return value must be a boolean indicating the validity of the state, i.e.:<br /><br />boolean valid=stateValidator(table state)"
#define LUA_SET_STATE_VAL_CB_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|callback: name of the Lua calback"
#define LUA_SET_STATE_VAL_CB_RET ""
#define LUA_SET_STATE_VAL_CB_COMMAND "simExtOMPL_setStateValidationCallback"
#define LUA_SET_STATE_VAL_CB_APIHELP "number result=" LUA_SET_STATE_VAL_CB_COMMAND "(number taskHandle, string callback)"
const int inArgs_SET_STATE_VAL_CB[]={2, sim_lua_arg_int, 0, sim_lua_arg_string, 0};

void LUA_SET_STATE_VAL_CB_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_STATE_VAL_CB, inArgs_SET_STATE_VAL_CB[0], LUA_SET_STATE_VAL_CB_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        std::string callback = inData->at(1).stringData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_STATE_VAL_CB_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(callback == "")
        {
            task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
            task->stateValidation.callback.scriptId = 0;
            task->stateValidation.callback.function = "";
        }
        else
        {
            task->stateValidation.type = TaskDef::StateValidation::CALLBACK;
            task->stateValidation.callback.scriptId = p->scriptID;
            task->stateValidation.callback.function = callback;
        }

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_GOAL_CB_DESCR "Set a custom goal callback for the specified task. The argument passed to the callback is the state to test for goal satisfaction. The return values must be a boolean indicating wether the goal is satisfied, and a float indicating the distance to the goal, i.e.:<br /><br />boolean satisfied, number distance=goalSatisfied(table state)<br /><br />If a distance to the goal is not known, a constant value can be used, but the performance of the algorithm will be worse."
#define LUA_SET_GOAL_CB_PARAMS "taskHandle: " LUA_PARAM_TASK_HANDLE "" \
    "|callback: the name of the Lua callback"
#define LUA_SET_GOAL_CB_RET ""
#define LUA_SET_GOAL_CB_COMMAND "simExtOMPL_setGoalCallback"
#define LUA_SET_GOAL_CB_APIHELP "number result=" LUA_SET_GOAL_CB_COMMAND "(number taskHandle, string callback)"
const int inArgs_SET_GOAL_CB[]={2, sim_lua_arg_int, 0, sim_lua_arg_string, 0};

void LUA_SET_GOAL_CB_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_GOAL_CB, inArgs_SET_GOAL_CB[0], LUA_SET_GOAL_CB_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        simInt taskHandle = inData->at(0).intData[0];
        std::string callback = inData->at(1).stringData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
            simSetLastError(LUA_SET_GOAL_CB_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(callback == "")
        {
            simSetLastError(LUA_SET_GOAL_CB_COMMAND, "Invalid callback name.");
            break;
        }
        else
        {
            task->goal.type = TaskDef::Goal::CALLBACK;
            task->goal.callback.scriptId = p->scriptID;
            task->goal.callback.function = callback;
        }

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_TEST_LUA_CB_DESCR ""
#define LUA_TEST_LUA_CB_PARAMS ""
#define LUA_TEST_LUA_CB_RET ""
#define LUA_TEST_LUA_CB_COMMAND "simExtOMPL_testLuaCallback"
#define LUA_TEST_LUA_CB_APIHELP "number result=" LUA_TEST_LUA_CB_COMMAND "(string callback, number arg)"
const int inArgs_TEST_LUA_CB[]={2, sim_lua_arg_string, 0, sim_lua_arg_float, 0};

void LUA_TEST_LUA_CB_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simFloat returnResult = -1;

    do
    {
        if(!D.readDataFromLua(p, inArgs_TEST_LUA_CB, inArgs_TEST_LUA_CB[0], LUA_TEST_LUA_CB_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();
        std::string callback = inData->at(0).stringData[0];
        simFloat arg = inData->at(1).floatData[0];

        SLuaCallBack c;
        CLuaFunctionData D1;
        D1.pushOutData_luaFunctionCall(CLuaFunctionDataItem(arg));
        const int outArgs[]={1, sim_lua_arg_float, 0};
        D1.writeDataToLua_luaFunctionCall(&c, outArgs);

        if(simCallScriptFunction(p->scriptID, callback.c_str(), &c, NULL) != -1 &&
                D1.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], callback.c_str()))
        {
            std::vector<CLuaFunctionDataItem> *outData = D1.getOutDataPtr_luaFunctionCall();
            returnResult = outData->at(0).floatData[0];
            std::cout << "Test Lua callback " << callback << " returned " << returnResult << std::endl;
        }
        else
        {
            std::cout << "Test Lua callback error" << std::endl;
        }

        D1.releaseBuffers_luaFunctionCall(&c);
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#ifdef GENERATE_DOC
#define REGISTER_LUA_COMMAND(NAME) \
    std::cout << "    <command name=\"" LUA_##NAME##_COMMAND "\">" << std::endl \
        << "        <synopsis>" << LUA_##NAME##_APIHELP << "</synopsis>" << std::endl \
        << "        <description>" << LUA_##NAME##_DESCR << "</description>" << std::endl \
        << "        <params>" << LUA_##NAME##_PARAMS << "</params>" << std::endl \
        << "        <return>" << LUA_##NAME##_RET << "</return>" << std::endl \
        << "    </command>" << std::endl;
#define REGISTER_LUA_VARIABLE(NAME) \
    std::cout << "";
#include "v_repLib.cpp"
void registerLuaCommands();
int main(int argc, char ** argv)
{
    std::cout << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
    std::cout << "<?xml-stylesheet type=\"text/xsl\" href=\"reference.xsl\"?>" << std::endl;
    std::cout << "<doc>" << std::endl;
    registerLuaCommands();
    std::cout << "</doc>" << std::endl;
}
#else
#define REGISTER_LUA_COMMAND(NAME) { \
    std::vector<int> inArgs; \
    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_##NAME, inArgs); \
    simRegisterCustomLuaFunction(LUA_##NAME##_COMMAND, LUA_##NAME##_APIHELP, &inArgs[0], LUA_##NAME##_CALLBACK); \
}
#define REGISTER_LUA_VARIABLE(NAME) simRegisterCustomLuaVariable(#NAME, (boost::lexical_cast<std::string>(NAME)).c_str())
#endif // GENERATE_DOC

void registerLuaCommands()
{
    REGISTER_LUA_COMMAND(CREATE_STATE_SPACE);
    REGISTER_LUA_COMMAND(DESTROY_STATE_SPACE);
    REGISTER_LUA_COMMAND(CREATE_ROBOT);
    REGISTER_LUA_COMMAND(DESTROY_ROBOT);
    REGISTER_LUA_COMMAND(SET_STATE_SPACE);
    REGISTER_LUA_COMMAND(SET_COLLISION_OBJECTS);
    REGISTER_LUA_COMMAND(CREATE_TASK);
    REGISTER_LUA_COMMAND(DESTROY_TASK);
    REGISTER_LUA_COMMAND(PRINT_TASK_INFO);
    REGISTER_LUA_COMMAND(SET_ROBOT);
    REGISTER_LUA_COMMAND(SET_ENVIRONMENT);
    REGISTER_LUA_COMMAND(SET_START_STATE);
    REGISTER_LUA_COMMAND(SET_GOAL_STATE);
    REGISTER_LUA_COMMAND(SET_GOAL);
    REGISTER_LUA_COMMAND(COMPUTE);
    REGISTER_LUA_COMMAND(READ_STATE);
    REGISTER_LUA_COMMAND(WRITE_STATE);
    REGISTER_LUA_COMMAND(SET_PROJ_EVAL_CB);
    REGISTER_LUA_COMMAND(SET_STATE_VAL_CB);
    REGISTER_LUA_COMMAND(SET_GOAL_CB);
    REGISTER_LUA_COMMAND(TEST_LUA_CB);

    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_position2d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_pose2d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_position3d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_pose3d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_joint_position);

    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_BiTRRT);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_BITstar);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_BKPIECE1);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_CForest);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_EST);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_FMT);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_KPIECE1);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_LazyPRM);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_LazyPRMstar);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_LazyRRT);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_LBKPIECE1);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_LBTRRT);
    //REGISTER_LUA_VARIABLE(simx_ompl_algorithm_LightningRetrieveRepair);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_PDST);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_PRM);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_PRMstar);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_pRRT);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_pSBL);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_RRT);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_RRTConnect);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_RRTstar);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_SBL);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_SPARS);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_SPARStwo);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_STRIDE);
    REGISTER_LUA_VARIABLE(simx_ompl_algorithm_TRRT);
}

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt)
{
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL, curDirAndFile, 1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp+="\\v_rep.dll";
#elif defined (__linux)
    temp+="/libv_rep.so";
#elif defined (__APPLE__)
    temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL)
    {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'OMPL' plugin.\n";
        return(0);
    }
    if (getVrepProcAddresses(vrepLib)==0)
    {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'OMPL' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 30200) // if V-REP version is smaller than 3.02.00
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'OMPL' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

    registerLuaCommands();

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal=NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message == sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message == sim_message_eventcallback_menuitemselected)
    { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message == sim_message_eventcallback_instancepass)
    {    // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags = auxiliaryData[0];
        bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
        bool instanceSwitched=((flags&64)!=0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message == sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
        
    }

    if (message == sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start

    }

    if (message == sim_message_eventcallback_simulationended)
    { // Simulation just ended
        destroyTransientObjects();

    }

    if (message == sim_message_eventcallback_moduleopen)
    { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ( (customData == NULL)||(_stricmp("OMPL", (char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message == sim_message_eventcallback_modulehandle)
    { // A script called simHandleModule (by default the main script). Is only called during simulation.
        if ( (customData == NULL)||(_stricmp("OMPL", (char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message == sim_message_eventcallback_moduleclose)
    { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ( (customData == NULL)||(_stricmp("OMPL", (char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message == sim_message_eventcallback_instanceswitch)
    { // We switched to a different scene. Such a switch can only happen while simulation is not running

    }

    if (message == sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message == sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message == sim_message_eventcallback_guipass)&&refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag = false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    return(retVal);
}

