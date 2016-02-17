// Copyright 2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
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

#define PLUGIN_VERSION 3 // 3 since V3.3.0, 2 since V3.3.0Beta.

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

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
    sim_ompl_statespacetype_position2d = 50001,
    sim_ompl_statespacetype_pose2d,
    sim_ompl_statespacetype_position3d,
    sim_ompl_statespacetype_pose3d,
    sim_ompl_statespacetype_joint_position
};

enum Algorithm
{
    sim_ompl_algorithm_BiTRRT = 30001,
    sim_ompl_algorithm_BITstar,
    sim_ompl_algorithm_BKPIECE1,
    sim_ompl_algorithm_CForest,
    sim_ompl_algorithm_EST,
    sim_ompl_algorithm_FMT,
    sim_ompl_algorithm_KPIECE1,
    sim_ompl_algorithm_LazyPRM,
    sim_ompl_algorithm_LazyPRMstar,
    sim_ompl_algorithm_LazyRRT,
    sim_ompl_algorithm_LBKPIECE1,
    sim_ompl_algorithm_LBTRRT,
    //sim_ompl_algorithm_LightningRetrieveRepair,
    sim_ompl_algorithm_PDST,
    sim_ompl_algorithm_PRM,
    sim_ompl_algorithm_PRMstar,
    sim_ompl_algorithm_pRRT,
    sim_ompl_algorithm_pSBL,
    sim_ompl_algorithm_RRT,
    sim_ompl_algorithm_RRTConnect,
    sim_ompl_algorithm_RRTstar,
    sim_ompl_algorithm_SBL,
    sim_ompl_algorithm_SPARS,
    sim_ompl_algorithm_SPARStwo,
    sim_ompl_algorithm_STRIDE,
    sim_ompl_algorithm_TRRT
};

struct LuaCallbackFunction
{
    // name of the Lua function
    std::string function;
    // id of the V-REP script where the function is defined in
    simInt scriptId;
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
    // object handle in order to specify optional reference frame that is not absolute
    // for sim_ompl_statespace_pose2d, etc.
    simInt refFrameHandle;
    // weight of this state space component (used for state distance calculation):
    simFloat weight;
    // lower bounds of search space:
    std::vector<simFloat> boundsLow;
    // upper bounds of search space:
    std::vector<simFloat> boundsHigh;
    // use this state space as the default projection:
    bool defaultProjection;
};

struct TaskDef
{
    ObjectDefHeader header;
    // state space is a composition of elementary state spaces (internal handles to StateSpaceDef objects):
    std::vector<simInt> stateSpaces;
    // handle of the collision pairs:
    std::vector<simInt> collisionPairHandles;
    // start state:
    std::vector<simFloat> startState;
    // goal can be specified in different ways:
    struct Goal
    {
        enum {STATE, DUMMY_PAIR, CLLBACK} type;
		// goal ref. dummy:
		int refDummy;
		// goal metric:
		float metric[4]; // x,y,z,angle(orientation), relative to refDummy
		// goal tolerance:
		float tolerance;
        // goal state:
        std::vector<simFloat> state;
        // goal dummy pair:
        struct {simInt goalDummy, robotDummy;} dummyPair;
        // goal callback:
        LuaCallbackFunction callback;
    } goal;
    // state validation:
    struct StateValidation
    {
        enum {DEFAULT, CLLBACK} type;
        // state validation callback:
        LuaCallbackFunction callback;
    } stateValidation;
    // resolution at which state validity needs to be verified in order for a
    // motion between two states to be considered valid (specified as a
    // fraction of the space's extent)
    float stateValidityCheckingResolution;
    // state sampling:
    struct ValidStateSampling
    {
        enum {DEFAULT, CLLBACK} type;
        // state sampling callback:
        LuaCallbackFunction callback;
        // "near" state sampling callback:
        LuaCallbackFunction callbackNear;
    } validStateSampling;
    // projection evaluation:
    struct ProjectionEvaluation
    {
        enum {DEFAULT, CLLBACK} type;
        // projection evaluation callback:
        LuaCallbackFunction callback;
        // size of the projection (for callback)
        int dim;
    } projectionEvaluation;
    // search algorithm to use:
    Algorithm algorithm;
    // pointer to OMPL state space. will be valid only during planning (i.e. only valid for Lua callbacks)
    ob::StateSpacePtr stateSpacePtr;
    // pointer to OMPL space information. will be valid only during planning (i.e. only valid for Lua callbacks)
    ob::SpaceInformationPtr spaceInformationPtr;
    // state space dimension:
    int dim;
    // how many things we should say in the V-REP console? (0 = stay silent)
    int verboseLevel;
};

std::map<simInt, TaskDef *> tasks;
std::map<simInt, StateSpaceDef *> statespaces;
simInt nextTaskHandle = 1000;
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

        switch(task->projectionEvaluation.type)
        {
            case TaskDef::ProjectionEvaluation::DEFAULT:
                switch(task->goal.type)
                {
                    case TaskDef::Goal::STATE:
                    case TaskDef::Goal::CLLBACK:
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
            case TaskDef::ProjectionEvaluation::CLLBACK:
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

        switch(task->projectionEvaluation.type)
        {
            case TaskDef::ProjectionEvaluation::DEFAULT:
                switch(task->goal.type)
                {
                    case TaskDef::Goal::STATE:
                    case TaskDef::Goal::CLLBACK:
                        defaultProjection(state, projection);
                        break;
                    case TaskDef::Goal::DUMMY_PAIR:
                        dummyPairProjection(state, projection);
                        break;
                }
                break;
            case TaskDef::ProjectionEvaluation::CLLBACK:
                luaProjectCallback(state, projection);
                break;
        }
    }

protected:
    virtual int defaultProjectionSize() const
    {
        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            if(!stateSpace->defaultProjection) continue;

            switch(stateSpace->type)
            {
                case sim_ompl_statespacetype_pose2d:
                case sim_ompl_statespacetype_position2d:
                    return 2;
                case sim_ompl_statespacetype_pose3d:
                case sim_ompl_statespacetype_position3d:
                    return 3;
                case sim_ompl_statespacetype_joint_position:
                    return 1;
            }
        }

        return 0;
    }

    virtual void defaultProjection(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            if(!stateSpace->defaultProjection) continue;

            switch(stateSpace->type)
            {
                case sim_ompl_statespacetype_pose2d:
                    projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                    projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
                    break;
                case sim_ompl_statespacetype_pose3d:
                    projection(0) = s->as<ob::SE3StateSpace::StateType>(i)->getX();
                    projection(1) = s->as<ob::SE3StateSpace::StateType>(i)->getY();
                    projection(2) = s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                    break;
                case sim_ompl_statespacetype_position2d:
                    projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    break;
                case sim_ompl_statespacetype_position3d:
                    projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    projection(1) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    projection(2) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                    break;
                case sim_ompl_statespacetype_joint_position:
                    projection(0) = s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    break;
            }

            break;
        }
    }

    virtual int dummyPairProjectionSize() const
    {
        /*
        return 3;
        */
        int s=0;
        for (int i=0;i<3;i++)
        {
            if (task->goal.metric[i]!=0.0)
                s++;
        }
        if (s==0)
            s=1; // if X/Y/Z are ignored
        return(s);
    }

    virtual void dummyPairProjection(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        /*
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, -1, &pos[0]);
        projection(0) = pos[0];
        projection(1) = pos[1];
        projection(2) = pos[2];
        */

        // TODO: don't we need to apply the provided state to the robot, read the tip dummy's position, then project it?

        // do projection, only for axis that should not be ignored:
        simFloat pos[3];
        simGetObjectPosition(task->goal.dummyPair.robotDummy, task->goal.refDummy, &pos[0]);
        int ind=0;
        for (int i=0;i<3;i++)
        {
            if (task->goal.metric[i]!=0.0)
                projection(ind++) = pos[i];
        }
        if (ind==0)
            projection(0)=0.0; // if X/Y/Z are ignored 

        // TODO: restore original state, no?
    }

    virtual int luaProjectCallbackSize() const
    {
        return task->projectionEvaluation.dim;
    }

    virtual void luaProjectCallback(const ob::State *state, ob::EuclideanProjection& projection) const
    {
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        // The expected return arguments (2):
        const int outArgs[]={1, sim_lua_arg_float|sim_lua_arg_table, luaProjectCallbackSize()};

        SLuaCallBack c;
        CLuaFunctionData D;

        // Prepare the input arguments:
        std::vector<float> stateVecf;
        for(size_t i = 0; i < stateVec.size(); i++)
            stateVecf.push_back((float)stateVec[i]);
        D.pushOutData_luaFunctionCall(CLuaFunctionDataItem(stateVecf));
        D.writeDataToLua_luaFunctionCall(&c, outArgs);

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->projectionEvaluation.callback.scriptId, task->projectionEvaluation.callback.function.c_str(), &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->projectionEvaluation.callback.function.c_str()))
            {
                std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                for(size_t i = 0; i < outData->at(0).floatData.size(); i++)
                {
                    projection(i) = outData->at(0).floatData[i];
                    std::cout << (i ? ", " : "") << outData->at(0).floatData[i];
                }
                std::cout << std::endl;
            }
            else
            {
                throw ompl::Exception("Projection evaluator callback " + task->projectionEvaluation.callback.function + " return value size and/or type is incorrect");
            }
        }
        else
        {
            throw ompl::Exception("Projection evaluator callback " + task->projectionEvaluation.callback.function + " returned an error");
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

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            ob::StateSpacePtr subSpace;

            switch(stateSpace->type)
            {
                case sim_ompl_statespacetype_pose2d:
                    subSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                    break;
                case sim_ompl_statespacetype_pose3d:
                    subSpace = ob::StateSpacePtr(new ob::SE3StateSpace());
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->header.name + ".position");
                    subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->header.name + ".orientation");
                    break;
                case sim_ompl_statespacetype_position2d:
                    subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
                    break;
                case sim_ompl_statespacetype_position3d:
                    subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
                    break;
                case sim_ompl_statespacetype_joint_position:
                    subSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(1));
                    break;
            }

            subSpace->setName(stateSpace->header.name);
            addSubspace(subSpace, stateSpace->weight);

            // set bounds:

            ob::RealVectorBounds bounds(stateSpace->boundsLow.size());;
            for(size_t j = 0; j < stateSpace->boundsLow.size(); j++)
                bounds.setLow(j, stateSpace->boundsLow[j]);
            for(size_t j = 0; j < stateSpace->boundsHigh.size(); j++)
                bounds.setHigh(j, stateSpace->boundsHigh[j]);

            switch(stateSpace->type)
            {
                case sim_ompl_statespacetype_pose2d:
                    as<ob::SE2StateSpace>(i)->setBounds(bounds);
                    break;
                case sim_ompl_statespacetype_pose3d:
                    as<ob::SE3StateSpace>(i)->setBounds(bounds);
                    break;
                case sim_ompl_statespacetype_position2d:
                    as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                    break;
                case sim_ompl_statespacetype_position3d:
                    as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                    break;
                case sim_ompl_statespacetype_joint_position:
                    as<ob::RealVectorStateSpace>(i)->setBounds(bounds);
                    break;
            }
        }
    }

    // writes state s to V-REP:
    void writeState(const ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        int j = 0;
        simFloat pos[3], orient[4], value;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            switch(stateSpace->type)
            {
                case sim_ompl_statespacetype_pose2d:
                    simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                    pos[0] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getX();
                    pos[1] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getY();
                    orient[2] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                    simSetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                    simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    break;
                case sim_ompl_statespacetype_pose3d:
                    pos[0] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getX();
                    pos[1] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getY();
                    pos[2] = (float)s->as<ob::SE3StateSpace::StateType>(i)->getZ();
                    orient[0] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().x;
                    orient[1] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().y;
                    orient[2] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().z;
                    orient[3] = (float)s->as<ob::SE3StateSpace::StateType>(i)->rotation().w;
                    simSetObjectQuaternion(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                    simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    break;
                case sim_ompl_statespacetype_position2d:
                    simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    pos[0] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    pos[1] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    break;
                case sim_ompl_statespacetype_position3d:
                    pos[0] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    pos[1] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[1];
                    pos[2] = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[2];
                    simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    break;
                case sim_ompl_statespacetype_joint_position:
                    value = (float)s->as<ob::RealVectorStateSpace::StateType>(i)->values[0];
                    simSetJointPosition(stateSpace->objectHandle, value);
                    break;
            }
        }
    }

    // reads state s from V-REP:
    void readState(ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        simFloat pos[3], orient[4], value;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];

            switch(stateSpace->type)
            {
                case sim_ompl_statespacetype_pose2d:
                    simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                    s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                    s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[2]);
                    break;
                case sim_ompl_statespacetype_pose3d:
                    simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    simGetObjectQuaternion(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                    s->as<ob::SE3StateSpace::StateType>(i)->setXYZ(pos[0], pos[1], pos[2]);
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().x = orient[0];
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().y = orient[1];
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().z = orient[2];
                    s->as<ob::SE3StateSpace::StateType>(i)->rotation().w = orient[3];
                    break;
                case sim_ompl_statespacetype_position2d:
                    simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                    break;
                case sim_ompl_statespacetype_position3d:
                    simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[0] = pos[0];
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[1] = pos[1];
                    s->as<ob::RealVectorStateSpace::StateType>(i)->values[2] = pos[2];
                    break;
                case sim_ompl_statespacetype_joint_position:
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
        case TaskDef::StateValidation::CLLBACK:
            return checkCallback(state);
        }
        return false;
    }

protected:
    virtual bool checkDefault(const ob::State *state) const
    {
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
        for(size_t i = 0; i < task->collisionPairHandles.size()/2; i++)
        {
            if(task->collisionPairHandles[2*i+0] >= 0)
            {
                int r = simCheckCollision(task->collisionPairHandles[2*i+0],task->collisionPairHandles[2*i+1]);
                if(r > 0)
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
        for(size_t i = 0; i < stateVec.size(); i++)
            stateVecf.push_back((float)stateVec[i]);
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
        case TaskDef::Goal::CLLBACK:
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

        if (task->goal.metric[3]==0.0)
        { // ignore orientation
            float goalPos[3];
            float robotPos[3];
            simGetObjectPosition(task->goal.dummyPair.goalDummy,task->goal.refDummy, &goalPos[0]);
            simGetObjectPosition(task->goal.dummyPair.robotDummy,task->goal.refDummy, &robotPos[0]);
            *distance = sqrt(pow((goalPos[0] - robotPos[0])*task->goal.metric[0], 2) + pow((goalPos[1] - robotPos[1])*task->goal.metric[1], 2) + pow((goalPos[2] - robotPos[2])*task->goal.metric[2], 2));
        }
        else
        { // do not ignore orientation
            float goalM[12];
            float robotM[12];
            simGetObjectMatrix(task->goal.dummyPair.goalDummy,task->goal.refDummy,goalM);
            simGetObjectMatrix(task->goal.dummyPair.robotDummy,task->goal.refDummy,robotM);
            float axis[3];
            float angle;
            simGetRotationAxis(robotM,goalM,axis,&angle);
            *distance = sqrt(pow((goalM[3] - robotM[3])*task->goal.metric[0], 2) + pow((goalM[7] - robotM[7])*task->goal.metric[1], 2) + pow((goalM[11] - robotM[11])*task->goal.metric[2], 2) + pow(angle*task->goal.metric[3], 2));
        }

        bool satisfied = *distance <= tolerance;

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);

        return satisfied;
    }

    virtual bool checkCallback(const ob::State *state, double *distance) const
    {
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
        for(size_t i = 0; i < stateVec.size(); i++)
            stateVecf.push_back((float)stateVec[i]);
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

class ValidStateSampler : public ob::UniformValidStateSampler
{
public:
    ValidStateSampler(const ob::SpaceInformation *si, TaskDef *task)
        : ob::UniformValidStateSampler(si), task(task)
    {
        name_ = "VREPValidStateSampler";
    }

    bool sample(ob::State *state)
    {
        if(task->validStateSampling.type == TaskDef::ValidStateSampling::CLLBACK)
        {
            if(task->validStateSampling.callback.function == "")
            {
                throw ompl::Exception("Specified empty callback for valid state sampling");
            }

            bool ret = false;

            // The expected return arguments (1):
            const int outArgs[]={1, sim_lua_arg_float|sim_lua_arg_table, 0};

            SLuaCallBack c;
            CLuaFunctionData D;

            // no input arguments

            // Call the function in the calling script:
            if(simCallScriptFunction(task->goal.callback.scriptId, task->validStateSampling.callback.function.c_str(), &c, NULL) != -1)
            {
                // the call succeeded

                // Now check the return arguments:
                if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->validStateSampling.callback.function.c_str()))
                {
                    std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                    std::vector<double> stateVec;
                    for(size_t i = 0; i < outData->at(0).floatData.size(); i++)
                        stateVec.push_back((double)outData->at(0).floatData[i]);
                    task->stateSpacePtr->copyFromReals(state, stateVec);
                    ret = true;
                }
                else
                {
                    throw ompl::Exception("Valid state sampling callback " + task->validStateSampling.callback.function + " return value size and/or type is incorrect (callback: " + luaCallbackToString(&c) + ")");
                }
            }
            else
            {
                throw ompl::Exception("Valid state sampling callback " + task->validStateSampling.callback.function + " returned an error");
            }

            // Release the data:
            D.releaseBuffers_luaFunctionCall(&c);

            return ret;
        }
        else
        {
            return ob::UniformValidStateSampler::sample(state);
        }
    }

    bool sampleNear(ob::State *state, const ob::State *nearState, const double distance)
    {
        if(task->validStateSampling.type == TaskDef::ValidStateSampling::CLLBACK)
        {
            if(task->validStateSampling.callbackNear.function == "")
            {
                throw ompl::Exception("Specified empty callback for \"near\" valid state sampling");
            }

            std::vector<double> nearStateVec;
            task->stateSpacePtr->copyToReals(nearStateVec, nearState);

            bool ret = false;

            // The expected return arguments (1):
            const int outArgs[]={1, sim_lua_arg_float|sim_lua_arg_table, 0};

            SLuaCallBack c;
            CLuaFunctionData D;

            // Prepare the input arguments:
            std::vector<float> nearStateVecf;
            for(size_t i = 0; i < nearStateVec.size(); i++)
                nearStateVecf.push_back((float)nearStateVec[i]);
            D.pushOutData_luaFunctionCall(CLuaFunctionDataItem(nearStateVecf));
            D.pushOutData_luaFunctionCall(CLuaFunctionDataItem((float)distance));
            D.writeDataToLua_luaFunctionCall(&c, outArgs);

            // Call the function in the calling script:
            if(simCallScriptFunction(task->goal.callback.scriptId, task->validStateSampling.callbackNear.function.c_str(), &c, NULL) != -1)
            {
                // the call succeeded

                // Now check the return arguments:
                if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->validStateSampling.callbackNear.function.c_str()))
                {
                    std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                    std::vector<double> stateVec;
                    for(size_t i = 0; i < outData->at(0).floatData.size(); i++)
                        stateVec.push_back((double)outData->at(0).floatData[i]);
                    task->stateSpacePtr->copyFromReals(state, stateVec);
                    ret = true;
                }
                else
                {
                    throw ompl::Exception("Near valid state sampling callback " + task->validStateSampling.callbackNear.function + " return value size and/or type is incorrect (callback: " + luaCallbackToString(&c) + ")");
                }
            }
            else
            {
                throw ompl::Exception("Near valid state sampling callback " + task->validStateSampling.callbackNear.function + " returned an error");
            }

            // Release the data:
            D.releaseBuffers_luaFunctionCall(&c);

            return ret;
        }
        else
        {
            return ob::UniformValidStateSampler::sampleNear(state, nearState, distance);
        }
    }

protected:
    TaskDef *task;
};

typedef boost::shared_ptr<ValidStateSampler> ValidStateSamplerPtr;

ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si, TaskDef *task)
{
    return ValidStateSamplerPtr(new ValidStateSampler(si, task));
}

#define HYPERLINK(function) "<a href=\"#" function "\">" function "</a>"
#define PARAM(name,description) "<param name=\"" name "\">" description "</param>"

#define LUA_CREATE_STATE_SPACE_DESCR "Create a component of the state space for the motion planning problem."
#define LUA_CREATE_STATE_SPACE_PARAMS \
    PARAM("name", "a name for this state space") \
    PARAM("type", "the type of this state space component (must be one of sim_ompl_statespacetype_position2d, sim_ompl_statespacetype_pose2d, sim_ompl_statespacetype_position3d, sim_ompl_statespacetype_pose3d, sim_ompl_statespacetype_joint_position)") \
    PARAM("objectHandle", "the object handle (a joint object if type is sim_ompl_statespacetype_joint_position, otherwise a shape)") \
    PARAM("boundsLow", "lower bounds (if type is pose, specify only the 3 position components)") \
    PARAM("boundsHigh", "upper bounds (if type is pose, specify only the 3 position components)") \
    PARAM("useForProjection", "if true, this object position or joint value will be used for computing a default projection") \
    PARAM("weight", "(optional) the weight of this state space component, used for computing distance between states. Default value is 1.0") \
    PARAM("refObjectHandle", "(optional) an object handle relative to which reference frame position/orientations will be evaluated. Default value is -1, for the absolute reference frame")
#define LUA_CREATE_STATE_SPACE_RET \
    PARAM("stateSpaceHandle", "a handle to the created state space component")
#define LUA_CREATE_STATE_SPACE_COMMAND "simExtOMPL_createStateSpace"
#define LUA_CREATE_STATE_SPACE_APIHELP "number stateSpaceHandle=" LUA_CREATE_STATE_SPACE_COMMAND "(string name, number type, number objectHandle, table boundsLow, table boundsHigh, number useForProjection, number weight, number refObjectHandle)"
const int inArgs_CREATE_STATE_SPACE[]={8, sim_lua_arg_string, 0, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 0, sim_lua_arg_float|sim_lua_arg_table, 0, sim_lua_arg_int, 0, sim_lua_arg_float, 0, sim_lua_arg_int, 0};

void LUA_CREATE_STATE_SPACE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_CREATE_STATE_SPACE, inArgs_CREATE_STATE_SPACE[0]-2, LUA_CREATE_STATE_SPACE_COMMAND)) // last 2 args are optional
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        if(inData->at(3).floatData.size() != inData->at(4).floatData.size())
        {
            simSetLastError(LUA_CREATE_STATE_SPACE_COMMAND, "Lower and upper bounds must have the same length.");
            break;
        }

        float w = 1.0;
        if(inData->size() >= 7)
        {
            w = inData->at(6).floatData[0];
            if(w <= 0)
            {
                simSetLastError(LUA_CREATE_STATE_SPACE_COMMAND, "State component weight must be positive.");
                break;
            }
        }
        int refFrame = -1;
        if(inData->size() >= 8)
        {
            refFrame = inData->at(7).intData[0];
            if ( (refFrame != -1)&&((simIsHandleValid(refFrame,sim_appobj_object_type)<=0)) )
            {
                simSetLastError(LUA_CREATE_STATE_SPACE_COMMAND, "Reference object handle is not valid.");
                break;
            }
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
        for(size_t i = 0; i < inData->at(3).floatData.size(); i++)
            statespace->boundsLow.push_back(inData->at(3).floatData[i]);
        for(size_t i = 0; i < inData->at(4).floatData.size(); i++)
            statespace->boundsHigh.push_back(inData->at(4).floatData[i]);
        statespace->defaultProjection = inData->at(5).intData[0] > 0;
        statespace->weight = w;
        statespace->refFrameHandle = refFrame;
        statespaces[statespace->header.handle] = statespace;
        returnResult = statespace->header.handle;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_DESTROY_STATE_SPACE_DESCR "Destroy the spacified state space component.<br /><br />" \
    "Note: state space components created during simulation are automatically destroyed when simulation ends."
#define LUA_DESTROY_STATE_SPACE_PARAMS \
    PARAM("stateSpaceHandle", "handle to state space component")
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

#define LUA_CREATE_TASK_DESCR "Create a task object, used to represent the motion planning task. A task object contains informations about: <ul>" \
    "<li>collision pairs (used by the default state validity checker)</li>" \
    "<li>state spaces</li>" \
    "<li>start state</li>" \
    "<li>goal state, or goal specification (e.g. pair of dummies, Lua callback, ...)</li>" \
    "<li>various Lua callbacks (projection evaluation, state validation, goal satisfaction)</li>" \
    "</ul>"
#define LUA_CREATE_TASK_PARAMS \
    PARAM("name", "a name for this task object")
#define LUA_CREATE_TASK_RET \
    PARAM("taskHandle", "a handle to the created task object")
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
        task->stateValidityCheckingResolution = 0.01f; // 1% of state space's extent
        task->validStateSampling.type = TaskDef::ValidStateSampling::DEFAULT;
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
        task->algorithm = sim_ompl_algorithm_KPIECE1;
        task->verboseLevel = 0;
        tasks[task->header.handle] = task;
        returnResult = task->header.handle;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

TaskDef * getTaskOrSetError(const char *CMD, simInt taskHandle)
{
    if(tasks.find(taskHandle) == tasks.end())
    {
        simSetLastError(CMD, "Invalid task handle.");
        return NULL;
    }

    return tasks[taskHandle];
}

#define LUA_PARAM_TASK_HANDLE "a handle to a task object created with " HYPERLINK(LUA_CREATE_TASK_COMMAND)
#define LUA_DESTROY_TASK_DESCR "Destroy the specified task object.<br /><br />" \
    "Note: task objects created during simulation are automatically destroyed when simulation ends."
#define LUA_DESTROY_TASK_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE)
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
        TaskDef *task = getTaskOrSetError(LUA_DESTROY_TASK_COMMAND, taskHandle);
        if(!task) break;

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
        case sim_ompl_statespacetype_position2d: return "sim_ompl_statespacetype_position2d";
        case sim_ompl_statespacetype_pose2d: return "sim_ompl_statespacetype_pose2d";
        case sim_ompl_statespacetype_position3d: return "sim_ompl_statespacetype_position3d";
        case sim_ompl_statespacetype_pose3d: return "sim_ompl_statespacetype_pose3d";
        case sim_ompl_statespacetype_joint_position: return "sim_ompl_statespacetype_joint_position";
        default: return "???";
    }
};

const char * algorithm_string(Algorithm alg)
{
    switch(alg)
    {
        case sim_ompl_algorithm_BiTRRT: return "sim_ompl_algorithm_BiTRRT";
        case sim_ompl_algorithm_BITstar: return "sim_ompl_algorithm_BITstar";
        case sim_ompl_algorithm_BKPIECE1: return "sim_ompl_algorithm_BKPIECE1";
        case sim_ompl_algorithm_CForest: return "sim_ompl_algorithm_CForest";
        case sim_ompl_algorithm_EST: return "sim_ompl_algorithm_EST";
        case sim_ompl_algorithm_FMT: return "sim_ompl_algorithm_FMT";
        case sim_ompl_algorithm_KPIECE1: return "sim_ompl_algorithm_KPIECE1";
        case sim_ompl_algorithm_LazyPRM: return "sim_ompl_algorithm_LazyPRM";
        case sim_ompl_algorithm_LazyPRMstar: return "sim_ompl_algorithm_LazyPRMstar";
        case sim_ompl_algorithm_LazyRRT: return "sim_ompl_algorithm_LazyRRT";
        case sim_ompl_algorithm_LBKPIECE1: return "sim_ompl_algorithm_LBKPIECE1";
        case sim_ompl_algorithm_LBTRRT: return "sim_ompl_algorithm_LBTRRT";
        //case sim_ompl_algorithm_LightningRetrieveRepair: return "sim_ompl_algorithm_LightningRetrieveRepair";
        case sim_ompl_algorithm_PDST: return "sim_ompl_algorithm_PDST";
        case sim_ompl_algorithm_PRM: return "sim_ompl_algorithm_PRM";
        case sim_ompl_algorithm_PRMstar: return "sim_ompl_algorithm_PRMstar";
        case sim_ompl_algorithm_pRRT: return "sim_ompl_algorithm_pRRT";
        case sim_ompl_algorithm_pSBL: return "sim_ompl_algorithm_pSBL";
        case sim_ompl_algorithm_RRT: return "sim_ompl_algorithm_RRT";
        case sim_ompl_algorithm_RRTConnect: return "sim_ompl_algorithm_RRTConnect";
        case sim_ompl_algorithm_RRTstar: return "sim_ompl_algorithm_RRTstar";
        case sim_ompl_algorithm_SBL: return "sim_ompl_algorithm_SBL";
        case sim_ompl_algorithm_SPARS: return "sim_ompl_algorithm_SPARS";
        case sim_ompl_algorithm_SPARStwo: return "sim_ompl_algorithm_SPARStwo";
        case sim_ompl_algorithm_STRIDE: return "sim_ompl_algorithm_STRIDE";
        case sim_ompl_algorithm_TRRT: return "sim_ompl_algorithm_TRRT";
        default: return "???";
    }
};

#define LUA_PRINT_TASK_INFO_DESCR "Print a summary of the specified task object. Useful for debugging and submitting bug reports."
#define LUA_PRINT_TASK_INFO_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE)
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
        TaskDef *task = getTaskOrSetError(LUA_PRINT_TASK_INFO_COMMAND, taskHandle);
        if(!task) break;

        std::stringstream s;
        std::string prefix = "OMPL: ";
        s << prefix << "task name: " << task->header.name << std::endl;
        s << prefix << "state spaces: (dimension: " << task->dim << ")" << std::endl;
        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];
            s << prefix << "    state space: " << stateSpace->header.handle << std::endl;
            s << prefix << "        name: " << stateSpace->header.name << std::endl;
            s << prefix << "        type: " << state_space_type_string(stateSpace->type) << std::endl;
            s << prefix << "        object handle: " << stateSpace->objectHandle << std::endl;
            s << prefix << "        bounds low: {";
            for(size_t j = 0; j < stateSpace->boundsLow.size(); j++)
                s << (j ? ", " : "") << stateSpace->boundsLow[j];
            s << "}" << std::endl;
            s << prefix << "        bounds high: {";
            for(size_t j = 0; j < stateSpace->boundsHigh.size(); j++)
                s << (j ? ", " : "") << stateSpace->boundsHigh[j];
            s << "}" << std::endl;
            s << prefix << "        default projection: " << (stateSpace->defaultProjection ? "true" : "false") << std::endl;
            s << prefix << "        weight: " << stateSpace->weight << std::endl;
        }
        s << prefix << "collision pairs: {";
        for(size_t i = 0; i < task->collisionPairHandles.size(); i++)
            s << (i ? ", " : "") << task->collisionPairHandles[i];
        s << "}" << std::endl;
        s << prefix << "start state: {";
        for(size_t i = 0; i < task->startState.size(); i++)
            s << (i ? ", " : "") << task->startState[i];
        s << "}" << std::endl;
        s << prefix << "goal:";
        switch(task->goal.type)
        {
        case TaskDef::Goal::STATE:
            s << std::endl;
            s << prefix << "    goal state: {";
            for(size_t i = 0; i < task->goal.state.size(); i++)
                s << (i ? ", " : "") << task->goal.state[i];
            s << "}" << std::endl;
            break;
        case TaskDef::Goal::DUMMY_PAIR:
            s << std::endl;
            s << prefix << "    robot dummy: " << task->goal.dummyPair.robotDummy << std::endl;
            s << prefix << "    goal dummy: " << task->goal.dummyPair.goalDummy << std::endl;
            s << prefix << "    ref dummy: " << task->goal.refDummy << std::endl;
            s << prefix << "    metric: {";
            for(size_t i = 0; i < 4; i++)
                s << (i ? ", " : "") << task->goal.metric[i];
            s << "}" << std::endl;
            s << prefix << "    tolerance: " << task->goal.tolerance << std::endl;
            break;
        case TaskDef::Goal::CLLBACK:
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
        case TaskDef::StateValidation::CLLBACK:
            s << std::endl;
            s << prefix << "    callback:" << std::endl;
            s << prefix << "        scriptId: " << task->stateValidation.callback.scriptId << std::endl;
            s << prefix << "        function: " << task->stateValidation.callback.function << std::endl;
            break;
        default:
            s << " ???" << std::endl;
            break;
        }
        s << prefix << "state validity checking resolution: " << task->stateValidityCheckingResolution << std::endl;
        s << prefix << "valid state sampling:";
        switch(task->validStateSampling.type)
        {
        case TaskDef::ValidStateSampling::DEFAULT:
            s << " default" << std::endl;
            break;
        case TaskDef::ValidStateSampling::CLLBACK:
            s << std::endl;
            s << prefix << "    callback:" << std::endl;
            s << prefix << "        scriptId: " << task->validStateSampling.callback.scriptId << std::endl;
            s << prefix << "        function: " << task->validStateSampling.callback.function << std::endl;
            s << prefix << "    callbackNear:" << std::endl;
            s << prefix << "        scriptId: " << task->validStateSampling.callbackNear.scriptId << std::endl;
            s << prefix << "        function: " << task->validStateSampling.callbackNear.function << std::endl;
            break;
        }
        s << prefix << "projection evaluation:";
        switch(task->projectionEvaluation.type)
        {
        case TaskDef::ProjectionEvaluation::DEFAULT:
            s << " default" << std::endl;
            break;
        case TaskDef::ProjectionEvaluation::CLLBACK:
            s << std::endl;
            s << prefix << "    callback:" << std::endl;
            s << prefix << "        scriptId:" << task->projectionEvaluation.callback.scriptId << std::endl;
            s << prefix << "        function:" << task->projectionEvaluation.callback.function << std::endl;
            break;
        default:
            s << " ???" << std::endl;
            break;
        }
        s << prefix << "algorithm: " << algorithm_string(task->algorithm) << std::endl;

        simAddStatusbarMessage(s.str().c_str());
        std::cout << s.str();

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_VERBOSE_LEVEL_DESCR "Set the verbosity level for messages printed to application console."
#define LUA_SET_VERBOSE_LEVEL_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("verboseLevel", "level of verbosity (positive integer), 0 to suppress any message")
#define LUA_SET_VERBOSE_LEVEL_RET ""
#define LUA_SET_VERBOSE_LEVEL_COMMAND "simExtOMPL_setVerboseLevel"
#define LUA_SET_VERBOSE_LEVEL_APIHELP "number result=" LUA_SET_VERBOSE_LEVEL_COMMAND "(number taskHandle, number verboseLevel)"
const int inArgs_SET_VERBOSE_LEVEL[]={2, sim_lua_arg_int, 0, sim_lua_arg_int, 0};

void LUA_SET_VERBOSE_LEVEL_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_VERBOSE_LEVEL, inArgs_SET_VERBOSE_LEVEL[0], LUA_SET_VERBOSE_LEVEL_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_SET_VERBOSE_LEVEL_COMMAND, taskHandle);
        if(!task) break;

        simInt verboseLevel = inData->at(1).intData[0];
        task->verboseLevel = verboseLevel;
        
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_DESCR "Set the resolution of state validity checking, expressed as fraction of state space's extent. Default resolution is 0.01 which is 1% of the state space's extent."
#define LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("resolution", "resolution of state validity checking, expressed as fraction of state space's extent")
#define LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_RET ""
#define LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_COMMAND "simExtOMPL_setStateValidityCheckingResolution"
#define LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_APIHELP "number result=" LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_COMMAND "(number taskHandle, number resolution)"
const int inArgs_SET_STATE_VALIDITY_CHECKING_RESOLUTION[]={2, sim_lua_arg_int, 0, sim_lua_arg_float, 0};

void LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_STATE_VALIDITY_CHECKING_RESOLUTION, inArgs_SET_STATE_VALIDITY_CHECKING_RESOLUTION[0], LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData = D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_SET_STATE_VALIDITY_CHECKING_RESOLUTION_COMMAND, taskHandle);
        if(!task) break;

        float stateValidityCheckingResolution = inData->at(1).floatData[0];
        task->stateValidityCheckingResolution = stateValidityCheckingResolution;
        
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_STATE_SPACE_DESCR "Set the state space of this task object."
#define LUA_SET_STATE_SPACE_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("stateSpaceHandles", "a table of handles to state space components, created with " HYPERLINK(LUA_CREATE_STATE_SPACE_COMMAND))
#define LUA_SET_STATE_SPACE_RET ""
#define LUA_SET_STATE_SPACE_COMMAND "simExtOMPL_setStateSpace"
#define LUA_SET_STATE_SPACE_APIHELP "number result=" LUA_SET_STATE_SPACE_COMMAND "(number taskHandle, table stateSpaceHandles)"
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

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_SET_STATE_SPACE_COMMAND, taskHandle);
        if(!task) break;

        bool valid_statespace_handles = true;

        for(size_t i = 0; i < inData->at(1).intData.size(); i++)
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

        task->stateSpaces.clear();
        task->dim = 0;
        for(size_t i = 0; i < inData->at(1).intData.size(); i++)
        {
            simInt stateSpaceHandle = inData->at(1).intData[i];
            task->stateSpaces.push_back(stateSpaceHandle);
            switch(statespaces.find(stateSpaceHandle)->second->type)
            {
                case sim_ompl_statespacetype_position2d:
                    task->dim += 2;
                    break;
                case sim_ompl_statespacetype_pose2d:
                    task->dim += 3;
                    break;
                case sim_ompl_statespacetype_position3d:
                    task->dim += 3;
                    break;
                case sim_ompl_statespacetype_pose3d:
                    task->dim += 7;
                    break;
                case sim_ompl_statespacetype_joint_position:
                    task->dim += 1;
                    break;
            }
        }
        
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_ALGORITHM_DESCR "Set the search algorithm for the specified task. Default algorithm used is KPIECE1."
#define LUA_SET_ALGORITHM_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("algorithm", "one of sim_ompl_algorithm_BiTRRT, sim_ompl_algorithm_BITstar, sim_ompl_algorithm_BKPIECE1, sim_ompl_algorithm_CForest, sim_ompl_algorithm_EST, sim_ompl_algorithm_FMT, sim_ompl_algorithm_KPIECE1, sim_ompl_algorithm_LazyPRM, sim_ompl_algorithm_LazyPRMstar, sim_ompl_algorithm_LazyRRT, sim_ompl_algorithm_LBKPIECE1, sim_ompl_algorithm_LBTRRT, sim_ompl_algorithm_PDST, sim_ompl_algorithm_PRM, sim_ompl_algorithm_PRMstar, sim_ompl_algorithm_pRRT, sim_ompl_algorithm_pSBL, sim_ompl_algorithm_RRT, sim_ompl_algorithm_RRTConnect, sim_ompl_algorithm_RRTstar, sim_ompl_algorithm_SBL, sim_ompl_algorithm_SPARS, sim_ompl_algorithm_SPARStwo, sim_ompl_algorithm_STRIDE, sim_ompl_algorithm_TRRT")
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
        TaskDef *task = getTaskOrSetError(LUA_SET_ALGORITHM_COMMAND, taskHandle);
        if(!task) break;

        Algorithm algorithm = static_cast<Algorithm>(inData->at(1).intData[0]);
        task->algorithm = algorithm;

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_COLLISION_PAIRS_DESCR "Set the collision pairs for the specified task object."
#define LUA_SET_COLLISION_PAIRS_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("collisionPairHandles", "a table containing 2 entity handles for each collision pair. A collision pair is represented by a collider and a collidee, that will be tested against each other. The first pair could be used for robot self-collision testing, and a second pair could be used for robot-environment collision testing. The collider can be an object or a collection handle. The collidee can be an object or collection handle, or sim_handle_all, in which case the collider will be checked agains all other collidable objects in the scene.")
#define LUA_SET_COLLISION_PAIRS_RET \
    PARAM("result", "0 if the operation failed.")
#define LUA_SET_COLLISION_PAIRS_COMMAND "simExtOMPL_setCollisionPairs"
#define LUA_SET_COLLISION_PAIRS_APIHELP "number result=" LUA_SET_COLLISION_PAIRS_COMMAND "(number taskHandle, table collisionPairHandles)"
const int inArgs_SET_COLLISION_PAIRS[]={2, sim_lua_arg_int, 0, sim_lua_arg_int|sim_lua_arg_table, 0};

void LUA_SET_COLLISION_PAIRS_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_COLLISION_PAIRS, inArgs_SET_COLLISION_PAIRS[0], LUA_SET_COLLISION_PAIRS_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_SET_COLLISION_PAIRS_COMMAND, taskHandle);
        if(!task) break;

        int numHandles = (inData->at(1).intData.size()/2)*2;
        std::vector<simInt>& pairHandles = inData->at(1).intData;
        task->collisionPairHandles.clear();
        for(int i = 0; i < numHandles; i++)
            task->collisionPairHandles.push_back(pairHandles[i]);
        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

bool checkStateSize(const char *CMD, const TaskDef *task, const std::vector<float>& s, std::string descr = "State")
{
    if(s.size() == 0)
    {
        simSetLastError(CMD, (descr + " is empty.").c_str());
        return false;
    }
    if(s.size() != task->dim)
    {
        simSetLastError(CMD, (descr + " is of incorrect size.").c_str());
        return false;
    }
    return true;
}

#define LUA_PARAM_ROBOT_STATE "a table of numbers, whose size must be consistent with the robot's state space specified in this task object"
#define LUA_SET_START_STATE_DESCR "Set the start state for the specified task object."
#define LUA_SET_START_STATE_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("state", LUA_PARAM_ROBOT_STATE)
#define LUA_SET_START_STATE_RET ""
#define LUA_SET_START_STATE_COMMAND "simExtOMPL_setStartState"
#define LUA_SET_START_STATE_APIHELP "number result=" LUA_SET_START_STATE_COMMAND "(number taskHandle, table state)"
const int inArgs_SET_START_STATE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 1};

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
        TaskDef *task = getTaskOrSetError(LUA_SET_START_STATE_COMMAND, taskHandle);
        if(!task) break;

        if(!checkStateSize(LUA_SET_START_STATE_COMMAND, task, inData->at(1).floatData))
            break;

        task->startState.clear();
        for(size_t i = 0; i < inData->at(1).floatData.size(); i++)
            task->startState.push_back(inData->at(1).floatData[i]);

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_GOAL_STATE_DESCR "Set the goal state for the specified task object."
#define LUA_SET_GOAL_STATE_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("state", LUA_PARAM_ROBOT_STATE)
#define LUA_SET_GOAL_STATE_RET ""
#define LUA_SET_GOAL_STATE_COMMAND "simExtOMPL_setGoalState"
#define LUA_SET_GOAL_STATE_APIHELP "number result=" LUA_SET_GOAL_STATE_COMMAND "(number taskHandle, table state)"
const int inArgs_SET_GOAL_STATE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 1};

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
        TaskDef *task = getTaskOrSetError(LUA_SET_GOAL_STATE_COMMAND, taskHandle);
        if(!task) break;

        if(!checkStateSize(LUA_SET_GOAL_STATE_COMMAND, task, inData->at(1).floatData))
            break;

        task->goal.type = TaskDef::Goal::STATE;
        task->goal.state.clear();
        for(size_t i = 0; i < inData->at(1).floatData.size(); i++)
            task->goal.state.push_back(inData->at(1).floatData[i]);

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_GOAL_DESCR "Set the goal for the specificed task object by a dummy pair. One of the two dummies is part of the robot. The other dummy is fixed in the environment. When the task is solved, the position or pose of the two dummies will (approximatively) be the same. Dummy-dummy distances are relative to an optional reference dummy, and are evaluated using an optional metric"
#define LUA_SET_GOAL_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("robotDummy", "a dummy attached to the robot") \
    PARAM("goalDummy", "a dummy fixed in the environment, representing the goal pose/position") \
    PARAM("tolerance", "the tolerated dummy-dummy distance") \
    PARAM("metric", "an optional metric (x,y,z,angle) used to evaluate the dummy-dummy distance") \
    PARAM("refDummy", "an optional reference dummy, relative to which the metric will be used")
#define LUA_SET_GOAL_RET ""
#define LUA_SET_GOAL_COMMAND "simExtOMPL_setGoal"
#define LUA_SET_GOAL_APIHELP "number result=" LUA_SET_GOAL_COMMAND "(number taskHandle, number robotDummy, number goalDummy, number tolerance=0.001, table_4 metric=nil, number refDummy=nil)"
const int inArgs_SET_GOAL[]={6, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_float, 0, sim_lua_arg_float|sim_lua_arg_table, 4, sim_lua_arg_int, 0};

void LUA_SET_GOAL_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_GOAL, inArgs_SET_GOAL[0]-3, LUA_SET_GOAL_COMMAND)) // 3 last args are optional
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_SET_GOAL_COMMAND, taskHandle);
        if(!task) break;

        task->goal.type = TaskDef::Goal::DUMMY_PAIR;
        task->goal.dummyPair.goalDummy = inData->at(1).intData[0];
        task->goal.dummyPair.robotDummy = inData->at(2).intData[0];

        if (inData->size()>=4)
    		task->goal.tolerance=inData->at(3).floatData[0];
        else
    		task->goal.tolerance=0.001f;

		if (inData->size()>=5)
		{
			task->goal.metric[0]=inData->at(4).floatData[0];
			task->goal.metric[1]=inData->at(4).floatData[1];
			task->goal.metric[2]=inData->at(4).floatData[2];
			task->goal.metric[3]=inData->at(4).floatData[3];
		}
		else
		{
			task->goal.metric[0]=1.0;
			task->goal.metric[1]=1.0;
			task->goal.metric[2]=1.0;
			task->goal.metric[3]=0.1f;
		}

		if (inData->size()>=6)
			task->goal.refDummy=inData->at(5).intData[0];
		else
			task->goal.refDummy=-1;

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

ob::PlannerPtr plannerFactory(Algorithm algorithm, ob::SpaceInformationPtr si)
{
    ob::PlannerPtr planner;
#define PLANNER(x) case sim_ompl_algorithm_##x: planner = ob::PlannerPtr(new og::x(si)); break
    switch(algorithm)
    {
        PLANNER(BiTRRT);
        PLANNER(BITstar);
        PLANNER(BKPIECE1);
        PLANNER(CForest);
        PLANNER(EST); // needs projection
        PLANNER(FMT);
        PLANNER(KPIECE1); // needs projection
        PLANNER(LazyPRM);
        PLANNER(LazyPRMstar);
        PLANNER(LazyRRT);
        PLANNER(LBKPIECE1);
        PLANNER(LBTRRT);
        //PLANNER(LightningRetrieveRepair);
        PLANNER(PDST); // needs projection
        PLANNER(PRM);
        PLANNER(PRMstar);
        PLANNER(pRRT);
        PLANNER(pSBL);
        PLANNER(RRT);
        PLANNER(RRTConnect);
        PLANNER(RRTstar);
        PLANNER(SBL); // needs projection
        PLANNER(SPARS);
        PLANNER(SPARStwo);
        PLANNER(STRIDE);
        PLANNER(TRRT);
    }
#undef PLANNER
    return planner;
}

#define LUA_COMPUTE_DESCR "Use OMPL to find a solution for this motion planning task."
#define LUA_COMPUTE_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("maxTime", "maximum time used for the path searching procedure, in seconds.") \
    PARAM("maxSimplificationTime", "(optional) maximum time used for the path simplification procedure, in seconds. 0 for a default simplification procedure.") \
    PARAM("stateCnt", "(optional) minimum number of states to be returned. 0 for a default behaviour.")
#define LUA_COMPUTE_RET \
    PARAM("states", "a table of states, representing the solution, from start to goal. States are specified linearly.")
#define LUA_COMPUTE_COMMAND "simExtOMPL_compute"
#define LUA_COMPUTE_APIHELP "number result, table states=" LUA_COMPUTE_COMMAND "(number taskHandle, number maxTime, number maxSimplificationTime, number stateCnt)"
const int inArgs_COMPUTE[]={4, sim_lua_arg_int, 0, sim_lua_arg_float, 0, sim_lua_arg_float, 0, sim_lua_arg_int, 0};

void LUA_COMPUTE_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;
    std::vector<simFloat> pathOut;

    do
    {
        if(!D.readDataFromLua(p, inArgs_COMPUTE, inArgs_COMPUTE[0]-2, LUA_COMPUTE_COMMAND)) // two last arguments are optional
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_COMPUTE_COMMAND, taskHandle);
        if(!task) break;

        simFloat maxTime = inData->at(1).floatData[0];
        simFloat simplificationMaxTime = 0.0;
        simInt minStates = 0;

        if(inData->size() >= 3)
    		simplificationMaxTime = inData->at(2).floatData[0];
        if(inData->size() >= 4)
    		minStates = inData->at(3).intData[0];

        try
        {
            ob::StateSpacePtr space(new StateSpace(task));
            task->stateSpacePtr = space;
            ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
            task->spaceInformationPtr = si;
            ob::ProjectionEvaluatorPtr projectionEval(new ProjectionEvaluator(space, task));
            space->registerDefaultProjection(projectionEval);
            ob::ProblemDefinitionPtr problemDef(new ob::ProblemDefinition(si));

            si->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(si, task)));

            si->setStateValidityCheckingResolution(task->stateValidityCheckingResolution);

            si->setValidStateSamplerAllocator(boost::bind(allocValidStateSampler, _1, task));

            ob::ScopedState<> startState(space);
            if(!checkStateSize(LUA_COMPUTE_COMMAND, task, task->startState, "Start state"))
                break;
            for(size_t i = 0; i < task->startState.size(); i++)
                startState[i] = task->startState[i];
            problemDef->addStartState(startState);

            ob::GoalPtr goal;
            if(task->goal.type == TaskDef::Goal::STATE)
            {
                if(!checkStateSize(LUA_COMPUTE_COMMAND, task, task->goal.state, "Goal state"))
                    break;
                ob::ScopedState<> goalState(space);
                for(size_t i = 0; i < task->goal.state.size(); i++)
                    goalState[i] = task->goal.state[i];
                goal = ob::GoalPtr(new ob::GoalState(si));
            }
            else if(task->goal.type == TaskDef::Goal::DUMMY_PAIR || task->goal.type == TaskDef::Goal::CLLBACK)
            {
                goal = ob::GoalPtr(new Goal(si, task, (double)task->goal.tolerance));
            }
            problemDef->setGoal(goal);

            ob::PlannerPtr planner = plannerFactory(task->algorithm, si);
            if(!planner)
            {
                simSetLastError(LUA_COMPUTE_COMMAND, "Invalid motion planning algorithm.");
                break;
            }
            planner->setProblemDefinition(problemDef);
            ob::PlannerStatus solved = planner->solve(maxTime);
            if(solved)
            {
                if(task->verboseLevel >= 2)
                    simAddStatusbarMessage("OMPL: simplifying solution...");

                const ob::PathPtr &path_ = problemDef->getSolutionPath();
                og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

                og::PathSimplifierPtr pathSimplifier(new og::PathSimplifier(si, goal));
                pathSimplifier->simplify(path, simplificationMaxTime); // always simplify the path, since the first version of the plugin had this

                if(task->verboseLevel >= 1)
                {
                    simAddStatusbarMessage("OMPL: found solution:");
                    std::stringstream s;
                    path.print(s);
                    simAddStatusbarMessage(s.str().c_str());
                }

                if(task->verboseLevel >= 2)
                    simAddStatusbarMessage("OMPL: interpolating solution...");

                if (minStates==0)
                    path.interpolate(); // this doesn't give the same result as path.interpolate(0) as I thought!!
                else
                    path.interpolate(minStates);
                if(task->verboseLevel >= 2)
                {
                    simAddStatusbarMessage("OMPL: interpolated:");
                    std::stringstream s;
                    path.print(s);
                    simAddStatusbarMessage(s.str().c_str());
                }

                for(size_t i = 0; i < path.getStateCount(); i++)
                {
                    const ob::StateSpace::StateType *s = path.getState(i);
                    std::vector<double> v;
                    space->copyToReals(v, s);
                    for(size_t j = 0; j < v.size(); j++)
                        pathOut.push_back((float)v[j]);
                }
                returnResult = 1;
            }
            else
            {
                if(task->verboseLevel >= 1)
                    simAddStatusbarMessage("OMPL: could not find solution.");
            }
        }
        catch(ompl::Exception& ex)
        {
            std::string s = "OMPL: exception: ";
            s += ex.what();
            std::cout << s << std::endl;
            simSetLastError(LUA_COMPUTE_COMMAND, s.c_str());
            if(task->verboseLevel >= 1)
                simAddStatusbarMessage(s.c_str());
        }
        task->stateSpacePtr.reset();
        task->spaceInformationPtr.reset();
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.pushOutData(CLuaFunctionDataItem(pathOut));
    D.writeDataToLua(p);
}

#define LUA_READ_STATE_DESCR "Read a state vector from current simulator state."
#define LUA_READ_STATE_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE)
#define LUA_READ_STATE_RET \
    PARAM("result", "") \
    PARAM("state", "state (vector)")
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
        TaskDef *task = getTaskOrSetError(LUA_READ_STATE_COMMAND, taskHandle);
        if(!task) break;

        if(!task->stateSpacePtr)
        {
            simSetLastError(LUA_READ_STATE_COMMAND, "This method can only be used inside callbacks.");
            break;
        }

        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        task->stateSpacePtr->as<StateSpace>()->readState(state);
        std::vector<double> stateVec = state.reals();
        for(size_t i = 0; i < stateVec.size(); i++)
            stateOut.push_back((float)stateVec[i]);

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.pushOutData(CLuaFunctionDataItem(stateOut));
    D.writeDataToLua(p);
}

#define LUA_WRITE_STATE_DESCR "Write the specified state to simulator"
#define LUA_WRITE_STATE_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("state", "state (vector)")
#define LUA_WRITE_STATE_RET \
    PARAM("result", "")
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
        TaskDef *task = getTaskOrSetError(LUA_WRITE_STATE_COMMAND, taskHandle);
        if(!task) break;

        if(!task->stateSpacePtr)
        {
            simSetLastError(LUA_WRITE_STATE_COMMAND, "This method can only be used inside callbacks.");
            break;
        }

        if(!checkStateSize(LUA_WRITE_STATE_COMMAND, task, inData->at(1).floatData))
            break;
        
        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        for(int i = 0; i < task->dim; i++)
            state[i] = (double)inData->at(1).floatData[i];
        task->stateSpacePtr->as<StateSpace>()->writeState(state);

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_IS_STATE_VALID_DESCR "Check if the specified state is valid. If a state validation callback has been specified, that will be used to determine the validity of the state, otherwise the default state validation method will be used."
#define LUA_IS_STATE_VALID_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("state", LUA_PARAM_ROBOT_STATE)
#define LUA_IS_STATE_VALID_RET ""
#define LUA_IS_STATE_VALID_COMMAND "simExtOMPL_isStateValid"
#define LUA_IS_STATE_VALID_APIHELP "number result=" LUA_IS_STATE_VALID_COMMAND "(number taskHandle, table state)"
const int inArgs_IS_STATE_VALID[]={2, sim_lua_arg_int, 0, sim_lua_arg_float|sim_lua_arg_table, 1};

void LUA_IS_STATE_VALID_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_IS_STATE_VALID, inArgs_IS_STATE_VALID[0], LUA_IS_STATE_VALID_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_IS_STATE_VALID_COMMAND, taskHandle);
        if(!task) break;

        if(!task->stateSpacePtr)
        {
            simSetLastError(LUA_IS_STATE_VALID_COMMAND, "This method can only be used inside callbacks.");
            break;
        }

        if(!checkStateSize(LUA_IS_STATE_VALID_COMMAND, task, inData->at(1).floatData))
            break;

        std::vector<double> stateVec;
        for(size_t i = 0; i < inData->at(1).floatData.size(); i++)
            stateVec.push_back((double)inData->at(1).floatData[i]);
        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        ob::State *s = &(*state);
        task->stateSpacePtr->copyFromReals(s, stateVec);

        returnResult = task->spaceInformationPtr->isValid(s) ? 1 : 0;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_PROJ_EVAL_CB_DESCR "Set a custom projection evaluation. The argument of the callback will be a state, and the return value must be a table of numbers, with a size equal to the projectionSize argument, i.e.<br /><br />table projection=evaluateProjection(table state)"
#define LUA_SET_PROJ_EVAL_CB_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("callback", "name of the Lua callback") \
    PARAM("projectionSize", "size of the projection (usually 2 or 3)")
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
        TaskDef *task = getTaskOrSetError(LUA_SET_PROJ_EVAL_CB_COMMAND, taskHandle);
        if(!task) break;

        std::string callback = inData->at(1).stringData[0];
        simInt projectionSize = inData->at(2).intData[0];

        if(projectionSize < 1)
        {
            simSetLastError(LUA_SET_PROJ_EVAL_CB_COMMAND, "Projection size must be positive.");
            break;
        }

        if(callback == "")
        {
            task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
            task->projectionEvaluation.dim = 0;
            task->projectionEvaluation.callback.scriptId = 0;
            task->projectionEvaluation.callback.function = "";
        }
        else
        {
            task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::CLLBACK;
            task->projectionEvaluation.dim = projectionSize;
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
#define LUA_SET_STATE_VAL_CB_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("callback", "name of the Lua calback")
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
        TaskDef *task = getTaskOrSetError(LUA_SET_STATE_VAL_CB_COMMAND, taskHandle);
        if(!task) break;

        std::string callback = inData->at(1).stringData[0];

        if(callback == "")
        {
            task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
            task->stateValidation.callback.scriptId = 0;
            task->stateValidation.callback.function = "";
        }
        else
        {
            task->stateValidation.type = TaskDef::StateValidation::CLLBACK;
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
#define LUA_SET_GOAL_CB_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("callback", "the name of the Lua callback")
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
        TaskDef *task = getTaskOrSetError(LUA_SET_GOAL_CB_COMMAND, taskHandle);
        if(!task) break;

        std::string callback = inData->at(1).stringData[0];

        if(callback == "")
        {
            simSetLastError(LUA_SET_GOAL_CB_COMMAND, "Invalid callback name.");
            break;
        }
        else
        {
            task->goal.type = TaskDef::Goal::CLLBACK;
            task->goal.callback.scriptId = p->scriptID;
            task->goal.callback.function = callback;
        }

        returnResult = 1;
    }
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.writeDataToLua(p);
}

#define LUA_SET_VALID_STATE_SAMPLER_CB_DESCR "The valid state sampler callbacks must generate valid states. There are two callbacks to implement:<ul><li>the valid state sampling callback:<br /><br />table sampledState=sample()<br /><br /></li><li>the near valid state sampling callback:<br /><br />table sampledState=sampleNear(table state, number distance)</li></ul>"
#define LUA_SET_VALID_STATE_SAMPLER_CB_PARAMS \
    PARAM("taskHandle", LUA_PARAM_TASK_HANDLE) \
    PARAM("callback", "the name of the Lua callback for sampling a state") \
    PARAM("callbackNear", "the name of the Lua callback for sampling near a given state within the given distance")
#define LUA_SET_VALID_STATE_SAMPLER_CB_RET ""
#define LUA_SET_VALID_STATE_SAMPLER_CB_COMMAND "simExtOMPL_setValidStateSamplerCallback"
#define LUA_SET_VALID_STATE_SAMPLER_CB_APIHELP "number result=" LUA_SET_VALID_STATE_SAMPLER_CB_COMMAND "(number taskHandle, string callback, string nearCallback)"
const int inArgs_SET_VALID_STATE_SAMPLER_CB[]={3, sim_lua_arg_int, 0, sim_lua_arg_string, 0, sim_lua_arg_string, 0};

void LUA_SET_VALID_STATE_SAMPLER_CB_CALLBACK(SLuaCallBack* p)
{
    p->outputArgCount = 0;
    CLuaFunctionData D;
    simInt returnResult = 0;

    do
    {
        if(!D.readDataFromLua(p, inArgs_SET_VALID_STATE_SAMPLER_CB, inArgs_SET_VALID_STATE_SAMPLER_CB[0], LUA_SET_VALID_STATE_SAMPLER_CB_COMMAND))
            break;

        std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

        simInt taskHandle = inData->at(0).intData[0];
        TaskDef *task = getTaskOrSetError(LUA_SET_VALID_STATE_SAMPLER_CB_COMMAND, taskHandle);
        if(!task) break;

        std::string callback = inData->at(1).stringData[0];
        std::string callbackNear = inData->at(2).stringData[0];

        if(callback == "" || callbackNear == "")
        {
            simSetLastError(LUA_SET_VALID_STATE_SAMPLER_CB_COMMAND, "Invalid callback name.");
            break;
        }
        else
        {
            task->validStateSampling.type = TaskDef::ValidStateSampling::CLLBACK;
            task->validStateSampling.callback.scriptId = p->scriptID;
            task->validStateSampling.callback.function = callback;
            task->validStateSampling.callbackNear.scriptId = p->scriptID;
            task->validStateSampling.callbackNear.function = callbackNear;
        }

        returnResult = 1;
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
void registerLuaCommands();
int main(int argc, char ** argv)
{
    std::cout << "<?xml version=\"1.0\"?>" << std::endl;
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
    REGISTER_LUA_COMMAND(CREATE_TASK);
    REGISTER_LUA_COMMAND(DESTROY_TASK);
    REGISTER_LUA_COMMAND(PRINT_TASK_INFO);
    REGISTER_LUA_COMMAND(SET_VERBOSE_LEVEL);
    REGISTER_LUA_COMMAND(SET_STATE_VALIDITY_CHECKING_RESOLUTION);
    REGISTER_LUA_COMMAND(SET_STATE_SPACE);
    REGISTER_LUA_COMMAND(SET_COLLISION_PAIRS);
    REGISTER_LUA_COMMAND(SET_START_STATE);
    REGISTER_LUA_COMMAND(SET_GOAL_STATE);
    REGISTER_LUA_COMMAND(SET_GOAL);
    REGISTER_LUA_COMMAND(COMPUTE);
    REGISTER_LUA_COMMAND(READ_STATE);
    REGISTER_LUA_COMMAND(WRITE_STATE);
    REGISTER_LUA_COMMAND(IS_STATE_VALID);
    REGISTER_LUA_COMMAND(SET_PROJ_EVAL_CB);
    REGISTER_LUA_COMMAND(SET_STATE_VAL_CB);
    REGISTER_LUA_COMMAND(SET_GOAL_CB);
    REGISTER_LUA_COMMAND(SET_VALID_STATE_SAMPLER_CB);
    REGISTER_LUA_COMMAND(SET_ALGORITHM);

    REGISTER_LUA_VARIABLE(sim_ompl_statespacetype_position2d);
    REGISTER_LUA_VARIABLE(sim_ompl_statespacetype_pose2d);
    REGISTER_LUA_VARIABLE(sim_ompl_statespacetype_position3d);
    REGISTER_LUA_VARIABLE(sim_ompl_statespacetype_pose3d);
    REGISTER_LUA_VARIABLE(sim_ompl_statespacetype_joint_position);

    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_BiTRRT);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_BITstar);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_BKPIECE1);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_CForest);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_EST);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_FMT);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_KPIECE1);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_LazyPRM);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_LazyPRMstar);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_LazyRRT);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_LBKPIECE1);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_LBTRRT);
    //REGISTER_LUA_VARIABLE(sim_ompl_algorithm_LightningRetrieveRepair);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_PDST);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_PRM);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_PRMstar);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_pRRT);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_pSBL);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_RRT);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_RRTConnect);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_RRTstar);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_SBL);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_SPARS);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_SPARStwo);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_STRIDE);
    REGISTER_LUA_VARIABLE(sim_ompl_algorithm_TRRT);
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
    if (vrepVer < 30203) // if V-REP version is smaller than 3.02.03
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'OMPL' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

    registerLuaCommands();

    return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{
    unloadVrepLibrary(vrepLib); // release the library
}

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData)
{
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal=NULL;

    if (message == sim_message_eventcallback_simulationended)
    { // Simulation just ended
        destroyTransientObjects();
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    return(retVal);
}

