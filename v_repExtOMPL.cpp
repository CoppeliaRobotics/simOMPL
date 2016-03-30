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
#include <ompl/base/goals/GoalStates.h>
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

#include "stubs.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
        std::vector<std::vector<simFloat> > states;
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
    // state space dimension:
    int dim;
    // how many things we should say in the V-REP console? (0 = stay silent)
    int verboseLevel;
    // OMPL classes (created with the setup() command):
    // state space
    ob::StateSpacePtr stateSpacePtr;
    // space information
    ob::SpaceInformationPtr spaceInformationPtr;
    // projection evaluator object
    ob::ProjectionEvaluatorPtr projectionEvaluatorPtr;
    // problem definition
    ob::ProblemDefinitionPtr problemDefinitionPtr;
    // planner
    ob::PlannerPtr planner;
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

        projectionEvaluationCallback_in in_args;
        projectionEvaluationCallback_out out_args;

        for(size_t i = 0; i < stateVec.size(); i++)
            in_args.state.push_back((float)stateVec[i]);

        if(projectionEvaluationCallback(task->projectionEvaluation.callback.scriptId, task->projectionEvaluation.callback.function.c_str(), &in_args, &out_args))
        {
            for(size_t i = 0; i < out_args.projection.size(); i++)
            {
                projection(i) = out_args.projection[i];
                std::cout << (i ? ", " : "") << out_args.projection[i];
            }
            std::cout << std::endl;
        }
        else
        {
            throw ompl::Exception("Projection evaluation callback " + task->projectionEvaluation.callback.function + " returned an error");
        }
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

        stateValidationCallback_in in_args;
        stateValidationCallback_out out_args;

        for(size_t i = 0; i < stateVec.size(); i++)
            in_args.state.push_back((float)stateVec[i]);

        if(stateValidationCallback(task->stateValidation.callback.scriptId, task->stateValidation.callback.function.c_str(), &in_args, &out_args))
        {
            ret = out_args.valid;
        }
        else
        {
            throw ompl::Exception("State validation callback " + task->stateValidation.callback.function + " returned an error");
        }

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

        bool ret = false;

        goalCallback_in in_args;
        goalCallback_out out_args;

        for(size_t i = 0; i < stateVec.size(); i++)
            in_args.state.push_back((float)stateVec[i]);

        if(goalCallback(task->goal.callback.scriptId, task->goal.callback.function.c_str(), &in_args, &out_args))
        {
            ret = out_args.satisfied;
            *distance = out_args.distance;
        }
        else
        {
            throw ompl::Exception("Goal callback " + task->goal.callback.function + " returned an error");
        }

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

            validStateSamplerCallback_in in_args;
            validStateSamplerCallback_out out_args;

            if(validStateSamplerCallback(task->validStateSampling.callback.scriptId, task->validStateSampling.callback.function.c_str(), &in_args, &out_args))
            {
                std::vector<double> stateVec;
                for(size_t i = 0; i < out_args.sampledState.size(); i++)
                    stateVec.push_back((double)out_args.sampledState[i]);
                task->stateSpacePtr->copyFromReals(state, stateVec);
                ret = true;
            }
            else
            {
                throw ompl::Exception("Valid state sampling callback " + task->validStateSampling.callback.function + " returned an error");
            }

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

            validStateSamplerCallbackNear_in in_args;
            validStateSamplerCallbackNear_out out_args;

            for(size_t i = 0; i < nearStateVec.size(); i++)
                in_args.state.push_back((float)nearStateVec[i]);
            in_args.distance = distance;

            if(validStateSamplerCallbackNear(task->validStateSampling.callbackNear.scriptId, task->validStateSampling.callbackNear.function.c_str(), &in_args, &out_args))
            {
                std::vector<double> stateVec;
                for(size_t i = 0; i < out_args.sampledState.size(); i++)
                    stateVec.push_back((double)out_args.sampledState[i]);
                task->stateSpacePtr->copyFromReals(state, stateVec);
                ret = true;
            }
            else
            {
                throw ompl::Exception("Near valid state sampling callback " + task->validStateSampling.callbackNear.function + " returned an error");
            }

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

void createStateSpace(SScriptCallBack *p, const char *cmd, createStateSpace_in *in, createStateSpace_out *out)
{
    if(in->boundsLow.size() != in->boundsHigh.size())
    {
        simSetLastError(cmd, "Lower and upper bounds must have the same length.");
        return;
    }

    if(in->weight <= 0)
    {
        simSetLastError(cmd, "State component weight must be positive.");
        return;
    }

    if(in->refObjectHandle != -1 && simIsHandleValid(in->refObjectHandle, sim_appobj_object_type) <= 0)
    {
        simSetLastError(cmd, "Reference object handle is not valid.");
        return;
    }

    StateSpaceDef *statespace = new StateSpaceDef();
    statespace->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
    statespace->header.handle = nextStateSpaceHandle++;
    statespace->header.name = in->name;
    statespace->type = static_cast<StateSpaceType>(in->type);
    statespace->objectHandle = in->objectHandle;
    for(size_t i = 0; i < in->boundsLow.size(); i++)
        statespace->boundsLow.push_back(in->boundsLow[i]);
    for(size_t i = 0; i < in->boundsHigh.size(); i++)
        statespace->boundsHigh.push_back(in->boundsHigh[i]);
    statespace->defaultProjection = in->useForProjection > 0;
    statespace->weight = in->weight;
    statespace->refFrameHandle = in->refObjectHandle;
    statespaces[statespace->header.handle] = statespace;
    out->stateSpaceHandle = statespace->header.handle;
}

void destroyStateSpace(SScriptCallBack *p, const char *cmd, destroyStateSpace_in *in, destroyStateSpace_out *out)
{
    if(statespaces.find(in->stateSpaceHandle) == statespaces.end())
    {
        simSetLastError(cmd, "Invalid state space handle handle.");
        out->result = 0;
        return;
    }

    StateSpaceDef *statespace = statespaces[in->stateSpaceHandle];
    statespaces.erase(in->stateSpaceHandle);
    delete statespace;
    
    out->result = 1;
}

void createTask(SScriptCallBack *p, const char *cmd, createTask_in *in, createTask_out *out)
{
    TaskDef *task = new TaskDef();
    task->header.destroyAfterSimulationStop = simGetSimulationState() != sim_simulation_stopped;
    task->header.handle = nextTaskHandle++;
    task->header.name = in->name;
    task->goal.type = TaskDef::Goal::STATE;
    task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
    task->stateValidityCheckingResolution = 0.01f; // 1% of state space's extent
    task->validStateSampling.type = TaskDef::ValidStateSampling::DEFAULT;
    task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
    task->algorithm = sim_ompl_algorithm_KPIECE1;
    task->verboseLevel = 0;
    tasks[task->header.handle] = task;
    out->taskHandle = task->header.handle;
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

void destroyTask(SScriptCallBack *p, const char *cmd, destroyTask_in *in, destroyTask_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    tasks.erase(in->taskHandle);
    delete task;

    out->result = 1;
}

void printTaskInfo(SScriptCallBack *p, const char *cmd, printTaskInfo_in *in, printTaskInfo_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    std::stringstream s;
    std::string prefix = "OMPL: ";
    s << prefix << "task name: " << task->header.name << std::endl;
    s << prefix << "state spaces: (dimension: " << task->dim << ")" << std::endl;
    for(size_t i = 0; i < task->stateSpaces.size(); i++)
    {
        StateSpaceDef *stateSpace = statespaces[task->stateSpaces[i]];
        s << prefix << "    state space: " << stateSpace->header.handle << std::endl;
        s << prefix << "        name: " << stateSpace->header.name << std::endl;
        s << prefix << "        type: " << statespacetype_string(stateSpace->type) << std::endl;
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
        s << prefix << "    goal state(s):" << std::endl;
        for(size_t j = 0; j < task->goal.states.size(); j++)
        {
            s << prefix << "        {";
            for(size_t i = 0; i < task->goal.states[j].size(); i++)
                s << (i ? ", " : "") << task->goal.states[j][i];
            s << "}" << std::endl;
        }
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

    out->result = 1;
}

void setVerboseLevel(SScriptCallBack *p, const char *cmd, setVerboseLevel_in *in, setVerboseLevel_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    task->verboseLevel = in->verboseLevel;
    
    out->result = 1;
}

void setStateValidityCheckingResolution(SScriptCallBack *p, const char *cmd, setStateValidityCheckingResolution_in *in, setStateValidityCheckingResolution_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    task->stateValidityCheckingResolution = in->resolution;
    
    out->result = 1;
}

void setStateSpace(SScriptCallBack *p, const char *cmd, setStateSpace_in *in, setStateSpace_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    bool valid_statespace_handles = true;

    for(size_t i = 0; i < in->stateSpaceHandles.size(); i++)
    {
        simInt stateSpaceHandle = in->stateSpaceHandles[i];

        if(statespaces.find(stateSpaceHandle) == statespaces.end())
        {
            valid_statespace_handles = false;
            break;
        }
    }

    if(!valid_statespace_handles)
    {
        simSetLastError(cmd, "Invalid state space handle.");
        return;
    }

    task->stateSpaces.clear();
    task->dim = 0;
    for(size_t i = 0; i < in->stateSpaceHandles.size(); i++)
    {
        simInt stateSpaceHandle = in->stateSpaceHandles[i];
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
    
    out->result = 1;
}

void setAlgorithm(SScriptCallBack *p, const char *cmd, setAlgorithm_in *in, setAlgorithm_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    task->algorithm = static_cast<Algorithm>(in->algorithm);

    out->result = 1;
}

void setCollisionPairs(SScriptCallBack *p, const char *cmd, setCollisionPairs_in *in, setCollisionPairs_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    int numHandles = (in->collisionPairHandles.size() / 2) * 2;
    task->collisionPairHandles.clear();
    for(int i = 0; i < numHandles; i++)
        task->collisionPairHandles.push_back(in->collisionPairHandles[i]);

    out->result = 1;
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
        std::stringstream ss;
        ss << descr << " is of incorrect size. Expected " << task->dim << ", got " << s.size() << ".";
        if(task->dim == 0)
            ss << " Did you forget to set the state space for this task?";
        simSetLastError(CMD, ss.str().c_str());
        return false;
    }
    return true;
}

void setStartState(SScriptCallBack *p, const char *cmd, setStartState_in *in, setStartState_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;
    if(!checkStateSize(cmd, task, in->state)) return;

    task->startState.clear();
    for(size_t i = 0; i < in->state.size(); i++)
        task->startState.push_back(in->state[i]);

    // for multi-query PRM, if the OMPL's ProblemDefinition has already been set,
    // we want only to clear the query and add the new start state:
    if(task->problemDefinitionPtr && task->planner && task->algorithm == sim_ompl_algorithm_PRM)
    {
        task->planner->as<og::PRM>()->clearQuery();
        ob::ScopedState<> startState(task->stateSpacePtr);
        for(size_t i = 0; i < task->startState.size(); i++)
            startState[i] = task->startState[i];
        task->problemDefinitionPtr->clearStartStates();
        task->problemDefinitionPtr->addStartState(startState);
    }

    out->result = 1;
}

void setGoalState(SScriptCallBack *p, const char *cmd, setGoalState_in *in, setGoalState_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;
    if(!checkStateSize(cmd, task, in->state)) return;

    task->goal.type = TaskDef::Goal::STATE;
    task->goal.states.clear();
    task->goal.states.push_back(std::vector<simFloat>());

    for(size_t i = 0; i < in->state.size(); i++)
        task->goal.states[0].push_back(in->state[i]);

    out->result = 1;
}

void addGoalState(SScriptCallBack *p, const char *cmd, addGoalState_in *in, addGoalState_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;
    if(!checkStateSize(cmd, task, in->state)) return;

    task->goal.type = TaskDef::Goal::STATE;

    size_t last = task->goal.states.size();
    task->goal.states.push_back(std::vector<simFloat>());

    for(size_t i = 0; i < in->state.size(); i++)
        task->goal.states[last].push_back(in->state[i]);

    out->result = 1;
}

void setGoal(SScriptCallBack *p, const char *cmd, setGoal_in *in, setGoal_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    task->goal.type = TaskDef::Goal::DUMMY_PAIR;
    task->goal.dummyPair.goalDummy = in->goalDummy;
    task->goal.dummyPair.robotDummy = in->robotDummy;

    task->goal.tolerance = in->tolerance;

    task->goal.metric[0] = in->metric[0];
    task->goal.metric[1] = in->metric[1];
    task->goal.metric[2] = in->metric[2];
    task->goal.metric[3] = in->metric[3];

    task->goal.refDummy = in->refDummy;

    out->result = 1;
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

void setup(SScriptCallBack *p, const char *cmd, setup_in *in, setup_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    try
    {
        task->stateSpacePtr = ob::StateSpacePtr(new StateSpace(task));
        task->spaceInformationPtr = ob::SpaceInformationPtr(new ob::SpaceInformation(task->stateSpacePtr));
        task->projectionEvaluatorPtr = ob::ProjectionEvaluatorPtr(new ProjectionEvaluator(task->stateSpacePtr, task));
        task->stateSpacePtr->registerDefaultProjection(task->projectionEvaluatorPtr);
        task->problemDefinitionPtr = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(task->spaceInformationPtr));
        task->spaceInformationPtr->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(task->spaceInformationPtr, task)));
        task->spaceInformationPtr->setStateValidityCheckingResolution(task->stateValidityCheckingResolution);
        task->spaceInformationPtr->setValidStateSamplerAllocator(boost::bind(allocValidStateSampler, _1, task));

        ob::ScopedState<> startState(task->stateSpacePtr);
        if(!checkStateSize(cmd, task, task->startState, "Start state"))
            return;
        for(size_t i = 0; i < task->startState.size(); i++)
            startState[i] = task->startState[i];
        task->problemDefinitionPtr->addStartState(startState);

        ob::GoalPtr goal;
        if(task->goal.type == TaskDef::Goal::STATE)
        {
            for(size_t i = 0; i < task->goal.states.size(); i++)
                if(!checkStateSize(cmd, task, task->goal.states[i], "Goal state"))
                    return;

            if(task->goal.states.size() > 1)
            {
                goal = ob::GoalPtr(new ob::GoalStates(task->spaceInformationPtr));
                for(size_t j = 0; j < task->goal.states.size(); j++)
                {
                    ob::ScopedState<> goalState(task->stateSpacePtr);
                    for(size_t i = 0; i < task->goal.states[j].size(); i++)
                        goalState[i] = task->goal.states[j][i];
                    goal->as<ob::GoalStates>()->addState(goalState);
                }
            }
            else if(task->goal.states.size() == 1)
            {
                goal = ob::GoalPtr(new ob::GoalState(task->spaceInformationPtr));
                ob::ScopedState<> goalState(task->stateSpacePtr);
                for(size_t i = 0; i < task->goal.states[0].size(); i++)
                    goalState[i] = task->goal.states[0][i];
                goal->as<ob::GoalState>()->setState(goalState);
            }
            else
            {
                simSetLastError(cmd, "No goal state specified.");
                return;
            }
        }
        else if(task->goal.type == TaskDef::Goal::DUMMY_PAIR || task->goal.type == TaskDef::Goal::CLLBACK)
        {
            goal = ob::GoalPtr(new Goal(task->spaceInformationPtr, task, (double)task->goal.tolerance));
        }
        task->problemDefinitionPtr->setGoal(goal);

        task->planner = plannerFactory(task->algorithm, task->spaceInformationPtr);
        if(!task->planner)
        {
            simSetLastError(cmd, "Invalid motion planning algorithm.");
            return;
        }
        task->planner->setProblemDefinition(task->problemDefinitionPtr);

        out->result = 1;
    }
    catch(ompl::Exception& ex)
    {
        std::string s = "OMPL: exception: ";
        s += ex.what();
        std::cout << s << std::endl;
        simSetLastError(cmd, s.c_str());
        if(task->verboseLevel >= 1)
            simAddStatusbarMessage(s.c_str());
    }
}

void solve(SScriptCallBack *p, const char *cmd, solve_in *in, solve_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    try
    {
        ob::PlannerStatus solved = task->planner->solve(in->maxTime);
        if(solved)
        {
            if(task->verboseLevel >= 1)
            {
                const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
                og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

                simAddStatusbarMessage("OMPL: found solution:");
                std::stringstream s;
                path.print(s);
                simAddStatusbarMessage(s.str().c_str());
            }

            out->result = 1;
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
        simSetLastError(cmd, s.c_str());
        if(task->verboseLevel >= 1)
            simAddStatusbarMessage(s.c_str());
    }
}

void simplifyPath(SScriptCallBack *p, const char *cmd, simplifyPath_in *in, simplifyPath_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    try
    {
        if(task->verboseLevel >= 2)
            simAddStatusbarMessage("OMPL: simplifying solution...");

        const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
        og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

        og::PathSimplifierPtr pathSimplifier(new og::PathSimplifier(task->spaceInformationPtr, task->problemDefinitionPtr->getGoal()));
        if(in->maxSimplificationTime < -std::numeric_limits<double>::epsilon())
            pathSimplifier->simplifyMax(path);
        else
            pathSimplifier->simplify(path, in->maxSimplificationTime);

        if(task->verboseLevel >= 1)
        {
            simAddStatusbarMessage("OMPL: simplified solution:");
            std::stringstream s;
            path.print(s);
            simAddStatusbarMessage(s.str().c_str());
        }

        out->result = 1;
    }
    catch(ompl::Exception& ex)
    {
        std::string s = "OMPL: exception: ";
        s += ex.what();
        std::cout << s << std::endl;
        simSetLastError(cmd, s.c_str());
        if(task->verboseLevel >= 1)
            simAddStatusbarMessage(s.c_str());
    }
}

void interpolatePath(SScriptCallBack *p, const char *cmd, interpolatePath_in *in, interpolatePath_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    try
    {
        if(task->verboseLevel >= 2)
            simAddStatusbarMessage("OMPL: interpolating solution...");

        const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
        og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

        if (in->stateCnt == 0)
            path.interpolate(); // this doesn't give the same result as path.interpolate(0) as I thought!!
        else
            path.interpolate(in->stateCnt);

        if(task->verboseLevel >= 2)
        {
            simAddStatusbarMessage("OMPL: interpolated:");
            std::stringstream s;
            path.print(s);
            simAddStatusbarMessage(s.str().c_str());
        }

        out->result = 1;
    }
    catch(ompl::Exception& ex)
    {
        std::string s = "OMPL: exception: ";
        s += ex.what();
        std::cout << s << std::endl;
        simSetLastError(cmd, s.c_str());
        if(task->verboseLevel >= 1)
            simAddStatusbarMessage(s.c_str());
    }
}

void getPath(SScriptCallBack *p, const char *cmd, getPath_in *in, getPath_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    try
    {
        const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
        og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

        for(size_t i = 0; i < path.getStateCount(); i++)
        {
            const ob::StateSpace::StateType *s = path.getState(i);
            std::vector<double> v;
            task->stateSpacePtr->copyToReals(v, s);
            for(size_t j = 0; j < v.size(); j++)
                out->states.push_back((float)v[j]);
        }

        out->result = 1;
    }
    catch(ompl::Exception& ex)
    {
        std::string s = "OMPL: exception: ";
        s += ex.what();
        std::cout << s << std::endl;
        simSetLastError(cmd, s.c_str());
        if(task->verboseLevel >= 1)
            simAddStatusbarMessage(s.c_str());
    }
}

void compute(SScriptCallBack *p, const char *cmd, compute_in *in, compute_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(!setup(p, in->taskHandle)) return;
    if(!solve(p, in->taskHandle, in->maxTime)) return;
    if(!simplifyPath(p, in->taskHandle, in->maxSimplificationTime)) return;
    if(!interpolatePath(p, in->taskHandle, in->stateCnt)) return;
    getPath_out path;
    getPath(p, &path, in->taskHandle);
    if(!path.result) return;
    out->states = path.states;
    out->result = 1;
}

void readState(SScriptCallBack *p, const char *cmd, readState_in *in, readState_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(!task->stateSpacePtr)
    {
        simSetLastError(cmd, "This method can only be used inside callbacks.");
        return;
    }

    ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
    task->stateSpacePtr->as<StateSpace>()->readState(state);
    std::vector<double> stateVec = state.reals();
    for(size_t i = 0; i < stateVec.size(); i++)
        out->state.push_back((float)stateVec[i]);

    out->result = 1;
}

void writeState(SScriptCallBack *p, const char *cmd, writeState_in *in, writeState_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(!task->stateSpacePtr)
    {
        simSetLastError(cmd, "This method can only be used inside callbacks.");
        return;
    }

    if(!checkStateSize(cmd, task, in->state))
        return;
    
    ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
    for(int i = 0; i < task->dim; i++)
        state[i] = (double)in->state[i];
    task->stateSpacePtr->as<StateSpace>()->writeState(state);

    out->result = 1;
}

void isStateValid(SScriptCallBack *p, const char *cmd, isStateValid_in *in, isStateValid_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(!task->stateSpacePtr)
    {
        simSetLastError(cmd, "This method can only be used inside callbacks.");
        return;
    }

    if(!checkStateSize(cmd, task, in->state))
        return;

    std::vector<double> stateVec;
    for(size_t i = 0; i < in->state.size(); i++)
        stateVec.push_back((double)in->state[i]);
    ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
    ob::State *s = &(*state);
    task->stateSpacePtr->copyFromReals(s, stateVec);

    out->valid = task->spaceInformationPtr->isValid(s) ? 1 : 0;
}

void setProjectionEvaluationCallback(SScriptCallBack *p, const char *cmd, setProjectionEvaluationCallback_in *in, setProjectionEvaluationCallback_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(in->projectionSize < 1)
    {
        simSetLastError(cmd, "Projection size must be positive.");
        return;
    }

    if(in->callback == "")
    {
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
        task->projectionEvaluation.dim = 0;
        task->projectionEvaluation.callback.scriptId = 0;
        task->projectionEvaluation.callback.function = "";
    }
    else
    {
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::CLLBACK;
        task->projectionEvaluation.dim = in->projectionSize;
        task->projectionEvaluation.callback.scriptId = p->scriptID;
        task->projectionEvaluation.callback.function = in->callback;
    }

    out->result = 1;
}

void setStateValidationCallback(SScriptCallBack *p, const char *cmd, setStateValidationCallback_in *in, setStateValidationCallback_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(in->callback == "")
    {
        task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
        task->stateValidation.callback.scriptId = 0;
        task->stateValidation.callback.function = "";
    }
    else
    {
        task->stateValidation.type = TaskDef::StateValidation::CLLBACK;
        task->stateValidation.callback.scriptId = p->scriptID;
        task->stateValidation.callback.function = in->callback;
    }

    out->result = 1;
}

void setGoalCallback(SScriptCallBack *p, const char *cmd, setGoalCallback_in *in, setGoalCallback_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(in->callback == "")
    {
        simSetLastError(cmd, "Invalid callback name.");
        return;
    }
    else
    {
        task->goal.type = TaskDef::Goal::CLLBACK;
        task->goal.callback.scriptId = p->scriptID;
        task->goal.callback.function = in->callback;
    }

    out->result = 1;
}

void setValidStateSamplerCallback(SScriptCallBack *p, const char *cmd, setValidStateSamplerCallback_in *in, setValidStateSamplerCallback_out *out)
{
    TaskDef *task = getTaskOrSetError(cmd, in->taskHandle);
    if(!task) return;

    if(in->callback == "" || in->callbackNear == "")
    {
        simSetLastError(cmd, "Invalid callback name.");
        return;
    }
    else
    {
        task->validStateSampling.type = TaskDef::ValidStateSampling::CLLBACK;
        task->validStateSampling.callback.scriptId = p->scriptID;
        task->validStateSampling.callback.function = in->callback;
        task->validStateSampling.callbackNear.scriptId = p->scriptID;
        task->validStateSampling.callbackNear.function = in->callbackNear;
    }

    out->result = 1;
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

    if(!registerScriptStuff())
    {
        std::cout << "Initialization failed.\n";
        unloadVrepLibrary(vrepLib);
        return(0);
    }

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

