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
#define strConCat(x, y, z)	CONCAT(x, y, z)

#define PLUGIN_VERSION 2 // 2 since version 3.2.1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#include <ompl/base/Goal.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/StateSpace.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <ompl/geometric/SimpleSetup.h>

//#include <ompl/geometric/planners/rrt/BiTRRT.h>
//#include <ompl/geometric/planners/bitstar/BITstar.h>
//#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
//#include <ompl/geometric/planners/cforest/CForest.h>
//#include <ompl/geometric/planners/est/EST.h>
//#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
//#include <ompl/geometric/planners/prm/LazyPRM.h>
//#include <ompl/geometric/planners/prm/LazyPRMstar.h>
//#include <ompl/geometric/planners/rrt/LazyRRT.h>
//#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
//#include <ompl/geometric/planners/rrt/LBTRRT.h>
//#include <ompl/geometric/planners/experience/LightningRetrieveRepair.h>
//#include <ompl/geometric/planners/pdst/PDST.h>
//#include <ompl/geometric/planners/prm/PRM.h>
//#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/planners/rrt/pRRT.h>
//#include <ompl/geometric/planners/sbl/pSBL.h>
//#include <ompl/geometric/planners/rrt/RRT.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/geometric/planners/sbl/SBL.h>
//#include <ompl/geometric/planners/prm/SPARS.h>
//#include <ompl/geometric/planners/prm/SPARStwo.h>
//#include <ompl/geometric/planners/stride/STRIDE.h>
//#include <ompl/geometric/planners/rrt/TRRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

enum StateSpaceType
{
    simx_ompl_statespacetype_position2d = 1,
    simx_ompl_statespacetype_pose2d,
    simx_ompl_statespacetype_position3d,
    simx_ompl_statespacetype_pose3d,
    simx_ompl_statespacetype_joint_position
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
                        dim = defaultProjectionSize();
                        break;
                    case TaskDef::Goal::DUMMY_PAIR:
                        dim = dummyPairProjectionSize();
                        break;
                    case TaskDef::Goal::CALLBACK:
                        dim = 0;
                        // this situation is not feasible
                        // a warning should be issued at begin of compute
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
            cellSizes_[i] = 0.25;
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
                        defaultProjection(state, projection);
                        break;
                    case TaskDef::Goal::DUMMY_PAIR:
                        dummyPairProjection(state, projection);
                        break;
                    default:
                        break;
                }
                break;
            case TaskDef::ProjectionEvaluation::CALLBACK:
                luaProjectCallback(state, projection);
                break;
            default:
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

#if 0
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

        std::cout << "ProjectionEvaluator::luaProjectCallback - calling Lua callback " << task->projectionEvaluator.callback.function << "..." << std::endl;

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->projectionEvaluator.callback.scriptId, task->projectionEvaluator.callback.function, &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->projectionEvaluator.callback.function))
            {
                std::cout << "ProjectionEvaluator::luaProjectCallback - Lua callback " << task->projectionEvaluator.callback.function << " returned ";
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
                std::cout << "ProjectionEvaluator::luaProjectCallback - Lua callback " << task->projectionEvaluator.callback.function << " return type(s) are wrong" << std::endl;
            }
        }
        else
        {
            std::cout << "ProjectionEvaluator::luaProjectCallback - call to Lua callback " << task->projectionEvaluator.callback.function << " failed" << std::endl;
        }

        // Release the data:
        D.releaseBuffers_luaFunctionCall(&c);
#endif
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

#if 0
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

        std::cout << "StateValidityChecker::checkCallback - calling Lua callback " << task->stateValidation.callback.function << "..." << std::endl;

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->stateValidation.callback.scriptId, task->stateValidation.callback.function, &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->stateValidation.callback.function))
            {
                std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                ret = outData->at(0).boolData[0];
                std::cout << "StateValidityChecker::checkCallback - Lua callback " << task->stateValidation.callback.function << " returned " << ret << std::endl;
            }
            else
            {
                std::cout << "StateValidityChecker::checkCallback - Lua callback " << task->stateValidation.callback.function << " return type(s) are wrong" << std::endl;
            }
        }
        else
        {
            std::cout << "StateValidityChecker::checkCallback - call to Lua callback " << task->stateValidation.callback.function << " failed" << std::endl;
        }

        // Release the data:
        D.releaseBuffers_luaFunctionCall(&c);
#endif

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
        std::vector<double> stateVec;
        statespace->copyToReals(stateVec, state);

        double dist = std::numeric_limits<double>::infinity();
        bool ret = false;

#if 0
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

        std::cout << "Goal::checkCallback - calling Lua callback " << task->goal.callback.function << "..." << std::endl;

        // Call the function "test" in the calling script:
        if(simCallScriptFunction(task->goal.callback.scriptId, task->goal.callback.function, &c, NULL) != -1)
        {
            // the call succeeded

            // Now check the return arguments:
            if(D.readDataFromLua_luaFunctionCall(&c, outArgs, outArgs[0], task->goal.callback.function))
            {
                std::vector<CLuaFunctionDataItem> *outData = D.getOutDataPtr_luaFunctionCall();
                ret = outData->at(0).boolData[0];
                dist = outData->at(1).floatData[0];
                std::cout << "Goal::checkCallback - Lua callback " << task->goal.callback.function << " returned " << ret << ", " << dist << std::endl;
            }
            else
            {
                std::cout << "Goal::checkCallback - Lua callback " << task->goal.callback.function << " return type(s) are wrong" << std::endl;
            }
        }
        else
        {
            std::cout << "Goal::checkCallback - call to Lua callback " << task->goal.callback.function << " failed" << std::endl;
        }

        // Release the data:
        D.releaseBuffers_luaFunctionCall(&c);
#endif

        *distance = dist;
        return ret;
    }

    ob::StateSpacePtr statespace;
    TaskDef *task;
    double tolerance;
};

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
        task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
        tasks[task->header.handle] = task;
        returnResult = task->header.handle;
	}
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
	D.writeDataToLua(p);
}

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

#define LUA_COMPUTE_COMMAND "simExtOMPL_compute"
#define LUA_COMPUTE_APIHELP "number result, table states=" LUA_COMPUTE_COMMAND "(number taskHandle, number maxTime)"
const int inArgs_COMPUTE[]={2, sim_lua_arg_int, 0, sim_lua_arg_float, 0};

void LUA_COMPUTE_CALLBACK(SLuaCallBack* p)
{
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
        //ob::PlannerPtr planner(new og::BiTRRT(si));
        //ob::PlannerPtr planner(new og::BITstar(si));
        //ob::PlannerPtr planner(new og::BKPIECE1(si));
        //ob::PlannerPtr planner(new og::CForest(si));
        //ob::PlannerPtr planner(new og::EST(si)); // needs projection
        //ob::PlannerPtr planner(new og::FMT(si));
        ob::PlannerPtr planner(new og::KPIECE1(si)); // needs projection
        //ob::PlannerPtr planner(new og::LazyPRM(si));
        //ob::PlannerPtr planner(new og::LazyPRMstar(si));
        //ob::PlannerPtr planner(new og::LazyRRT(si));
        //ob::PlannerPtr planner(new og::LBKPIECE1(si));
        //ob::PlannerPtr planner(new og::LBTRRT(si));
        //ob::PlannerPtr planner(new og::LightningRetrieveRepair(si));
        //ob::PlannerPtr planner(new og::PDST(si)); // needs projection
        //ob::PlannerPtr planner(new og::PRM(si));
        //ob::PlannerPtr planner(new og::PRMstar(si));
        //ob::PlannerPtr planner(new og::pRRT(si));
        //ob::PlannerPtr planner(new og::pSBL(si));
        //ob::PlannerPtr planner(new og::RRT(si));
        //ob::PlannerPtr planner(new og::RRTConnect(si));
        //ob::PlannerPtr planner(new og::RRTstar(si));
        //ob::PlannerPtr planner(new og::SBL(si)); // needs projection
        //ob::PlannerPtr planner(new og::SPARS(si));
        //ob::PlannerPtr planner(new og::SPARStwo(si));
        //ob::PlannerPtr planner(new og::STRIDE(si));
        //ob::PlannerPtr planner(new og::TRRT(si));
        setup.setPlanner(planner);
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
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
    D.pushOutData(CLuaFunctionDataItem(pathOut));
	D.writeDataToLua(p);
}

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

#define LUA_SET_PROJ_EVAL_CB_COMMAND "simExtOMPL_setProjectionEvaluationCallback"
#define LUA_SET_PROJ_EVAL_CB_APIHELP "number result=" LUA_SET_PROJ_EVAL_CB_COMMAND "(number taskHandle, number scriptId, string callback, number projectionSize)"
const int inArgs_SET_PROJ_EVAL_CB[]={4, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_string, 0, sim_lua_arg_int, 0};

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
        simInt scriptId = inData->at(1).intData[0];
        std::string callback = inData->at(2).stringData[0];
		simInt projectionSize = inData->at(3).intData[0];

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
            task->projectionEvaluation.callback.scriptId = scriptId;
            task->projectionEvaluation.callback.function = callback;
        }

        returnResult = 1;
	}
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
	D.writeDataToLua(p);
}

#define LUA_SET_STATE_VAL_CB_COMMAND "simExtOMPL_setStateValidationCallback"
#define LUA_SET_STATE_VAL_CB_APIHELP "number result=" LUA_SET_STATE_VAL_CB_COMMAND "(number taskHandle, number scriptId, string callback)"
const int inArgs_SET_STATE_VAL_CB[]={3, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_string, 0};

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
        simInt scriptId = inData->at(1).intData[0];
        std::string callback = inData->at(2).stringData[0];

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
            task->stateValidation.callback.scriptId = scriptId;
            task->stateValidation.callback.function = callback;
        }

        returnResult = 1;
	}
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
	D.writeDataToLua(p);
}

#define LUA_SET_GOAL_CB_COMMAND "simExtOMPL_setGoalCallback"
#define LUA_SET_GOAL_CB_APIHELP "number result=" LUA_SET_GOAL_CB_COMMAND "(number taskHandle, number scriptId, string callback)"
const int inArgs_SET_GOAL_CB[]={3, sim_lua_arg_int, 0, sim_lua_arg_int, 0, sim_lua_arg_string, 0};

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
        simInt scriptId = inData->at(1).intData[0];
        std::string callback = inData->at(2).stringData[0];

        if(tasks.find(taskHandle) == tasks.end())
        {
			simSetLastError(LUA_SET_GOAL_CB_COMMAND, "Invalid task handle.");
            break;
        }

        TaskDef *task = tasks[taskHandle];

        if(robots.find(task->robotHandle) == robots.end())
        {
			simSetLastError(LUA_SET_GOAL_CB_COMMAND, "Invalid robot handle.");
            break;
        }

        if(callback == "")
        {
			simSetLastError(LUA_SET_GOAL_CB_COMMAND, "Invalid robot handle.");
            break;
        }
        else
        {
            task->goal.type = TaskDef::Goal::CALLBACK;
            task->goal.callback.scriptId = scriptId;
            task->goal.callback.function = callback;
        }

        returnResult = 1;
	}
    while(0);

    D.pushOutData(CLuaFunctionDataItem(returnResult));
	D.writeDataToLua(p);
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

	std::vector<int> inArgs;

	// Register Lua commands:
#define REGISTER_LUA_COMMAND(NAME) { \
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_##NAME, inArgs); \
	simRegisterCustomLuaFunction(LUA_##NAME##_COMMAND, LUA_##NAME##_APIHELP, &inArgs[0], LUA_##NAME##_CALLBACK); \
}

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

#undef REGISTER_LUA_COMMAND
#define REGISTER_LUA_VARIABLE(NAME) simRegisterCustomLuaVariable(#NAME, (boost::lexical_cast<std::string>(NAME)).c_str())

    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_position2d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_pose2d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_position3d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_pose3d);
    REGISTER_LUA_VARIABLE(simx_ompl_statespacetype_joint_position);

#undef REGISTER_LUA_VARIABLE

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
	{	// This message is sent each time the scene was rendered (well, shortly after) (very often)
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

