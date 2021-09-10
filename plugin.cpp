#include <functional>
#include <iostream>
#include <sstream>
#include <vector>
#include <map>

#include <ompl/config.h>
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
#include <ompl/base/spaces/DubinsStateSpace.h>

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
#if OMPL_VERSION_VALUE >= 1001000 // 1.1.0
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#if OMPL_VERSION_VALUE >= 1005000 // 1.5.0
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#else
#include <ompl/geometric/planners/bitstar/BITstar.h>
#endif
#endif

#include "simPlusPlus/Plugin.h"
#include "simPlusPlus/Handle.h"
#include "config.h"
#include "plugin.h"
#include "stubs.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

#if OMPL_VERSION_VALUE >= 1004000  // 1.4.0
typedef Eigen::Ref<Eigen::VectorXd> OMPLProjection;
#else  // All other versions
typedef ompl::base::EuclideanProjection& OMPLProjection;
#endif

struct LuaCallbackFunction
{
    // name of the Lua function
    std::string function;
    // id of the script where the function is defined in
    simInt scriptId;
};

struct StateSpaceDef
{
    // name of this object:
    std::string name;
    // type of this state space:
    StateSpaceType type;
    // handle of the object (object, or joint if type = joint_position/cyclic_joint_position):
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
    // (specific to dubins state space) turning radius:
    double dubinsTurningRadius;
    // (specific to dubins state space) symmetric:
    bool dubinsIsSymmetric;
};

struct TaskDef
{
    // name of this object:
    std::string name;
    // state space is a composition of elementary state spaces (internal handles to StateSpaceDef objects):
    std::vector<std::string> stateSpaces;
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
    // how many things we should say in the console? (0 = stay silent)
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

template<> std::string sim::Handle<StateSpaceDef>::tag() { return "OMPL.StateSpace"; }
template<> std::string sim::Handle<TaskDef>::tag() { return "OMPL.Task"; }

class ProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    ProjectionEvaluator(sim::Handles<StateSpaceDef> &ssh, const ob::StateSpacePtr& space, TaskDef *task)
        : ob::ProjectionEvaluator(space), stateSpaceHandles(ssh), statespace(space)
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

    virtual void project(const ob::State *state, OMPLProjection projection) const
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
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);

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
            case sim_ompl_statespacetype_cyclic_joint_position:
                return 1;
            case sim_ompl_statespacetype_dubins:
                return 2;
            }
        }

        return 0;
    }

    virtual void defaultProjection(const ob::State *state, OMPLProjection projection) const
    {
        const ob::CompoundState *s = state->as<ob::CompoundStateSpace::StateType>();

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);

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
            case sim_ompl_statespacetype_cyclic_joint_position:
                projection(0) = s->as<ob::SO2StateSpace::StateType>(i)->value;
                break;
            case sim_ompl_statespacetype_dubins:
                projection(0) = s->as<ob::SE2StateSpace::StateType>(i)->getX();
                projection(1) = s->as<ob::SE2StateSpace::StateType>(i)->getY();
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
        int s = 0;
        for(int i = 0; i < 3; i++)
        {
            if(task->goal.metric[i] != 0.0)
                s++;
        }
        if(s == 0)
            s = 1; // if X/Y/Z are ignored
        return s;
    }

    virtual void dummyPairProjection(const ob::State *state, OMPLProjection projection) const
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
        int ind = 0;
        for(int i = 0; i < 3; i++)
        {
            if(task->goal.metric[i] != 0.0)
                projection(ind++) = pos[i];
        }
        if(ind == 0)
            projection(0) = 0.0; // if X/Y/Z are ignored

        // TODO: restore original state, no?
    }

    virtual int luaProjectCallbackSize() const
    {
        return task->projectionEvaluation.dim;
    }

    virtual void luaProjectCallback(const ob::State *state, OMPLProjection projection) const
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
            }
        }
        else
        {
            throw ompl::Exception("Projection evaluation callback " + task->projectionEvaluation.callback.function + " returned an error");
        }
    }

    sim::Handles<StateSpaceDef> stateSpaceHandles;
    TaskDef *task;
    const ob::StateSpacePtr& statespace;
    int dim;
};

class StateSpace : public ob::CompoundStateSpace
{
public:
    StateSpace(sim::Handles<StateSpaceDef> &ssh, TaskDef *task)
        : ob::CompoundStateSpace(), stateSpaceHandles(ssh), task(task)
    {
        setName("SimCompoundStateSpace");
        type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 1;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);

            ob::StateSpacePtr subSpace;

            switch(stateSpace->type)
            {
            case sim_ompl_statespacetype_pose2d:
                subSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->name + ".position");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->name + ".orientation");
                break;
            case sim_ompl_statespacetype_pose3d:
                subSpace = ob::StateSpacePtr(new ob::SE3StateSpace());
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->name + ".position");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->name + ".orientation");
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
            case sim_ompl_statespacetype_cyclic_joint_position:
                subSpace = ob::StateSpacePtr(new ob::SO2StateSpace());
                break;
            case sim_ompl_statespacetype_dubins:
                subSpace = ob::StateSpacePtr(new ob::DubinsStateSpace(stateSpace->dubinsTurningRadius, stateSpace->dubinsIsSymmetric));
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(0)->setName(stateSpace->name + ".position");
                subSpace->as<ob::CompoundStateSpace>()->getSubspace(1)->setName(stateSpace->name + ".orientation");
                break;
            }

            subSpace->setName(stateSpace->name);
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
            case sim_ompl_statespacetype_cyclic_joint_position:
                if(!stateSpace->boundsLow.empty() || !stateSpace->boundsHigh.empty())
                    sim::addLog(sim_verbosity_warnings, "cyclic_joint_position state space has no bounds");
                break;
            case sim_ompl_statespacetype_dubins:
                as<ob::SE2StateSpace>(i)->setBounds(bounds);
                break;
            }
        }
    }

    // writes state s to CoppeliaSim:
    void writeState(const ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        int j = 0;
        simFloat pos[3], orient[4], value;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);

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
            case sim_ompl_statespacetype_cyclic_joint_position:
                value = (float)s->as<ob::SO2StateSpace::StateType>(i)->value;
                simSetJointPosition(stateSpace->objectHandle, value);
                break;
            case sim_ompl_statespacetype_dubins:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                pos[0] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getX();
                pos[1] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getY();
                orient[2] = (float)s->as<ob::SE2StateSpace::StateType>(i)->getYaw();
                simSetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]);
                simSetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                break;
            }
        }
    }

    // reads state s from CoppeliaSim:
    void readState(ob::ScopedState<ob::CompoundStateSpace>& s)
    {
        simFloat pos[3], orient[4], value;

        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);

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
            case sim_ompl_statespacetype_cyclic_joint_position:
                simGetJointPosition(stateSpace->objectHandle, &value);
                s->as<ob::SO2StateSpace::StateType>(i)->value = value;
                break;
            case sim_ompl_statespacetype_dubins:
                simGetObjectPosition(stateSpace->objectHandle, stateSpace->refFrameHandle, &pos[0]);
                simGetObjectOrientation(stateSpace->objectHandle, stateSpace->refFrameHandle, &orient[0]); // Euler angles
                s->as<ob::SE2StateSpace::StateType>(i)->setXY(pos[0], pos[1]);
                s->as<ob::SE2StateSpace::StateType>(i)->setYaw(orient[2]);
                break;
            }
        }
    }

    // Store relative pose of objects:
    void saveRelPoseState(std::vector<float>& p)
    {
        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);
            if(stateSpace->type == sim_ompl_statespacetype_pose2d ||
                    stateSpace->type == sim_ompl_statespacetype_pose3d ||
                    stateSpace->type == sim_ompl_statespacetype_position2d ||
                    stateSpace->type == sim_ompl_statespacetype_position3d)
            {
                p.resize(p.size() + 7);
                simGetObjectPosition(stateSpace->objectHandle, sim_handle_parent, &p[p.size() - 7]);
                simGetObjectQuaternion(stateSpace->objectHandle, sim_handle_parent, &p[p.size() - 4]);
            }
        }
    }

    // Restore relative pose of objects:
    void restoreRelPoseState(const std::vector<float>& p)
    {
        int pt = 0;
        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);
            if(stateSpace->type == sim_ompl_statespacetype_pose2d ||
                    stateSpace->type == sim_ompl_statespacetype_pose3d ||
                    stateSpace->type == sim_ompl_statespacetype_position2d ||
                    stateSpace->type == sim_ompl_statespacetype_position3d)
            {
                simSetObjectPosition(stateSpace->objectHandle, sim_handle_parent, &p[pt+0]);
                simSetObjectQuaternion(stateSpace->objectHandle, sim_handle_parent, &p[pt+3]);
                pt += 7;
            }
        }
    }

protected:
    sim::Handles<StateSpaceDef> &stateSpaceHandles;
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
        std::vector<float> pose_old;
        statespace->as<StateSpace>()->saveRelPoseState(pose_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        // check collisions:
        bool inCollision = false;
        for(size_t i = 0; i < task->collisionPairHandles.size() / 2; i++)
        {
            if(task->collisionPairHandles[2 * i + 0] >= 0)
            {
                int r = simCheckCollision(task->collisionPairHandles[2 * i + 0], task->collisionPairHandles[2 * i + 1]);
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
        statespace->as<StateSpace>()->restoreRelPoseState(pose_old);

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
        std::vector<float> pose_old;
        statespace->as<StateSpace>()->saveRelPoseState(pose_old);

        // write query state:
        statespace->as<StateSpace>()->writeState(s);

        if(task->goal.metric[3] == 0.0)
        { // ignore orientation
            float goalPos[3];
            float robotPos[3];
            simGetObjectPosition(task->goal.dummyPair.goalDummy, task->goal.refDummy, &goalPos[0]);
            simGetObjectPosition(task->goal.dummyPair.robotDummy, task->goal.refDummy, &robotPos[0]);
            *distance = sqrt(pow((goalPos[0] - robotPos[0])*task->goal.metric[0], 2) + pow((goalPos[1] - robotPos[1])*task->goal.metric[1], 2) + pow((goalPos[2] - robotPos[2])*task->goal.metric[2], 2));
        }
        else
        { // do not ignore orientation
            float goalM[12];
            float robotM[12];
            simGetObjectMatrix(task->goal.dummyPair.goalDummy, task->goal.refDummy, goalM);
            simGetObjectMatrix(task->goal.dummyPair.robotDummy, task->goal.refDummy, robotM);
            float axis[3];
            float angle;
            simGetRotationAxis(robotM, goalM, axis, &angle);
            *distance = sqrt(pow((goalM[3] - robotM[3])*task->goal.metric[0], 2) + pow((goalM[7] - robotM[7])*task->goal.metric[1], 2) + pow((goalM[11] - robotM[11])*task->goal.metric[2], 2) + pow(angle*task->goal.metric[3], 2));
        }

        bool satisfied = *distance <= tolerance;

        // restore original state:
        statespace->as<StateSpace>()->writeState(s_old);
        statespace->as<StateSpace>()->restoreRelPoseState(pose_old);

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
        name_ = "SimValidStateSampler";
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

typedef std::shared_ptr<ValidStateSampler> ValidStateSamplerPtr;

ob::ValidStateSamplerPtr allocValidStateSampler(const ob::SpaceInformation *si, TaskDef *task)
{
    return ob::ValidStateSamplerPtr(new ValidStateSampler(si, task));
}

class OutputHandler : public ompl::msg::OutputHandler
{
public:
    void log(const std::string &text, ompl::msg::LogLevel level, const char *filename, int line)
    {
        int csim_level = sim_verbosity_none;
        switch(level)
        {
        case ompl::msg::LogLevel::LOG_DEV2:
        case ompl::msg::LogLevel::LOG_DEV1:
            csim_level = sim_verbosity_trace;
            break;
        case ompl::msg::LogLevel::LOG_DEBUG:
            csim_level = sim_verbosity_debug;
            break;
        case ompl::msg::LogLevel::LOG_INFO:
            csim_level = sim_verbosity_infos;
            break;
        case ompl::msg::LogLevel::LOG_WARN:
            csim_level = sim_verbosity_warnings;
            break;
        case ompl::msg::LogLevel::LOG_ERROR:
            csim_level = sim_verbosity_errors;
            break;
        case ompl::msg::LogLevel::LOG_NONE:
            csim_level = sim_verbosity_none;
            break;
        default:
            csim_level = sim_verbosity_none;
            break;
        }
        sim::addLog(csim_level, "OMPL: %s:%d: %s", filename, line, text);
    }
};

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        oh = new OutputHandler;
        ompl::msg::useOutputHandler(oh);

        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("OMPL (open motion planning library) Plugin");
        setBuildDate(BUILD_DATE);
    }

    void onEnd()
    {
        ompl::msg::restorePreviousOutputHandler();
        delete oh;
    }

    void onScriptStateDestroyed(int scriptID)
    {
        for(auto statespace : stateSpaceHandles.find(scriptID))
            delete stateSpaceHandles.remove(statespace);
        for(auto task : taskHandles.find(scriptID))
            delete taskHandles.remove(task);
    }

    void createStateSpace(createStateSpace_in *in, createStateSpace_out *out)
    {
        if(in->boundsLow.size() != in->boundsHigh.size())
            throw std::runtime_error("Lower and upper bounds must have the same length.");

        if(in->weight <= 0)
            throw std::runtime_error("State component weight must be positive.");

        if(in->refObjectHandle != -1 && simIsHandleValid(in->refObjectHandle, sim_appobj_object_type) <= 0)
            throw std::runtime_error("Reference object handle is not valid.");

        StateSpaceDef *statespace = new StateSpaceDef();
        statespace->name = in->name;
        statespace->type = static_cast<StateSpaceType>(in->type);
        statespace->objectHandle = in->objectHandle;
        for(size_t i = 0; i < in->boundsLow.size(); i++)
            statespace->boundsLow.push_back(in->boundsLow[i]);
        for(size_t i = 0; i < in->boundsHigh.size(); i++)
            statespace->boundsHigh.push_back(in->boundsHigh[i]);
        statespace->defaultProjection = in->useForProjection > 0;
        statespace->weight = in->weight;
        statespace->refFrameHandle = in->refObjectHandle;
        statespace->dubinsTurningRadius = 0.05;
        statespace->dubinsIsSymmetric = false;
        out->stateSpaceHandle = stateSpaceHandles.add(statespace, in->_.scriptID);
    }

    void destroyStateSpace(destroyStateSpace_in *in, destroyStateSpace_out *out)
    {
        StateSpaceDef *statespace = stateSpaceHandles.get(in->stateSpaceHandle);
        delete stateSpaceHandles.remove(statespace);
    }

    void setDubinsParams(setDubinsParams_in *in, setDubinsParams_out *out)
    {
        StateSpaceDef *statespace = stateSpaceHandles.get(in->stateSpaceHandle);
        statespace->dubinsTurningRadius = in->turningRadius;
        statespace->dubinsIsSymmetric = in->isSymmetric;
    }

    void createTask(createTask_in *in, createTask_out *out)
    {
        TaskDef *task = new TaskDef();
        task->name = in->name;
        task->goal.type = TaskDef::Goal::STATE;
        task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
        task->stateValidityCheckingResolution = 0.01f; // 1% of state space's extent
        task->validStateSampling.type = TaskDef::ValidStateSampling::DEFAULT;
        task->projectionEvaluation.type = TaskDef::ProjectionEvaluation::DEFAULT;
        task->algorithm = sim_ompl_algorithm_KPIECE1;
        task->verboseLevel = 0;
        out->taskHandle = taskHandles.add(task, in->_.scriptID);
    }

    TaskDef * getTask(const std::string &taskHandle, bool mustBeSetUp = false)
    {
        auto task = taskHandles.get(taskHandle);

        if(mustBeSetUp)
        {
            if(!task->stateSpacePtr || !task->spaceInformationPtr || !task->projectionEvaluatorPtr || !task->problemDefinitionPtr)
                throw std::runtime_error("simOMPL.setup(taskHandle) has not been called");
        }

        return task;
    }

    void destroyTask(destroyTask_in *in, destroyTask_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        delete taskHandles.remove(task);
    }

    void printTaskInfo(printTaskInfo_in *in, printTaskInfo_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        std::stringstream s;
        std::string prefix = "OMPL: ";
        s << prefix << "task name: " << task->name << std::endl;
        s << prefix << "state spaces: (dimension: " << task->dim << ")" << std::endl;
        for(size_t i = 0; i < task->stateSpaces.size(); i++)
        {
            StateSpaceDef *stateSpace = stateSpaceHandles.get(task->stateSpaces[i]);
            s << prefix << "    state space: " << task->stateSpaces[i] << std::endl;
            s << prefix << "        name: " << stateSpace->name << std::endl;
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
            if(stateSpace->type == sim_ompl_statespacetype_dubins)
            {
                s << prefix << "        (dubins) turning radius: " << stateSpace->dubinsTurningRadius << std::endl;
                s << prefix << "        (dubins) symmetric: " << (stateSpace->dubinsIsSymmetric ? "true" : "false") << std::endl;
            }
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
    }

    void setVerboseLevel(setVerboseLevel_in *in, setVerboseLevel_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        task->verboseLevel = in->verboseLevel;
    }

    void setStateValidityCheckingResolution(setStateValidityCheckingResolution_in *in, setStateValidityCheckingResolution_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        task->stateValidityCheckingResolution = in->resolution;
    }

    void setStateSpace(setStateSpace_in *in, setStateSpace_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        for(size_t i = 0; i < in->stateSpaceHandles.size(); i++)
        {
            try
            {
                stateSpaceHandles.get(in->stateSpaceHandles[i]);
            }
            catch(...)
            {
                throw std::runtime_error("Invalid state space handle.");
            }
        }

        task->stateSpaces.clear();
        task->dim = 0;
        for(size_t i = 0; i < in->stateSpaceHandles.size(); i++)
        {
            std::string stateSpaceHandle = in->stateSpaceHandles[i];
            task->stateSpaces.push_back(stateSpaceHandle);
            switch(stateSpaceHandles.get(stateSpaceHandle)->type)
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
            case sim_ompl_statespacetype_cyclic_joint_position:
                task->dim += 1;
                break;
            case sim_ompl_statespacetype_dubins:
                task->dim += 3;
                break;
            }
        }
    }

    void getStateSpaceDimension(getStateSpaceDimension_in *in, getStateSpaceDimension_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        out->dim = task->dim;
    }

    void setAlgorithm(setAlgorithm_in *in, setAlgorithm_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        task->algorithm = static_cast<Algorithm>(in->algorithm);
    }

    void setCollisionPairs(setCollisionPairs_in *in, setCollisionPairs_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        int numHandles = (in->collisionPairHandles.size() / 2) * 2;
        task->collisionPairHandles.clear();
        for(int i = 0; i < numHandles; i++)
            task->collisionPairHandles.push_back(in->collisionPairHandles[i]);
    }

    void validateStateSize(const TaskDef *task, const std::vector<float>& s, std::string descr = "State")
    {
        if(s.size() == 0)
            throw std::runtime_error(descr + " is empty.");
        if(s.size() != task->dim)
        {
            std::stringstream ss;
            ss << descr << " is of incorrect size. Expected " << task->dim << ", got " << s.size() << ".";
            if(task->dim == 0)
                ss << " Did you forget to set the state space for this task?";
            throw std::runtime_error(ss.str());
        }
    }

    bool isMultiQuery(Algorithm algorithm)
    {
        switch(algorithm)
        {
        case sim_ompl_algorithm_PRM:
        case sim_ompl_algorithm_PRMstar:
            return true;
        default:
            return false;
        }
    }

    void setStartState(setStartState_in *in, setStartState_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);
        validateStateSize(task, in->state);

        task->startState.clear();
        for(size_t i = 0; i < in->state.size(); i++)
            task->startState.push_back(in->state[i]);

        // for multi-query PRM, if the OMPL's ProblemDefinition has already been set,
        // we want only to clear the query and add the new start state:
        if(task->problemDefinitionPtr && task->planner && isMultiQuery(task->algorithm))
        {
            task->planner->as<og::PRM>()->clearQuery();
            ob::ScopedState<> startState(task->stateSpacePtr);
            for(size_t i = 0; i < task->startState.size(); i++)
                startState[i] = task->startState[i];
            task->problemDefinitionPtr->clearSolutionPaths();
            task->problemDefinitionPtr->clearStartStates();
            task->problemDefinitionPtr->addStartState(startState);
        }
    }

    void setGoalState(setGoalState_in *in, setGoalState_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);
        validateStateSize(task, in->state);

        task->goal.type = TaskDef::Goal::STATE;
        task->goal.states.clear();
        task->goal.states.push_back(std::vector<simFloat>());

        for(size_t i = 0; i < in->state.size(); i++)
            task->goal.states[0].push_back(in->state[i]);
    }

    void addGoalState(addGoalState_in *in, addGoalState_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);
        validateStateSize(task, in->state);

        task->goal.type = TaskDef::Goal::STATE;

        size_t last = task->goal.states.size();
        task->goal.states.push_back(std::vector<simFloat>());

        for(size_t i = 0; i < in->state.size(); i++)
            task->goal.states[last].push_back(in->state[i]);
    }

    void setGoal(setGoal_in *in, setGoal_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        task->goal.type = TaskDef::Goal::DUMMY_PAIR;
        task->goal.dummyPair.goalDummy = in->goalDummy;
        task->goal.dummyPair.robotDummy = in->robotDummy;

        task->goal.tolerance = in->tolerance;

        task->goal.metric[0] = in->metric[0];
        task->goal.metric[1] = in->metric[1];
        task->goal.metric[2] = in->metric[2];
        task->goal.metric[3] = in->metric[3];

        task->goal.refDummy = in->refDummy;
    }

    ob::PlannerPtr plannerFactory(Algorithm algorithm, ob::SpaceInformationPtr si)
    {
        ob::PlannerPtr planner;
#define PLANNER(x) case sim_ompl_algorithm_##x: planner = ob::PlannerPtr(new og::x(si)); break
        switch(algorithm)
        {
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
#if OMPL_VERSION_VALUE >= 1001000 // 1.1.0
            PLANNER(BiTRRT);
            PLANNER(BITstar);
#endif
        }
#undef PLANNER
        return planner;
    }

    void setup(setup_in *in, setup_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        task->stateSpacePtr = ob::StateSpacePtr(new StateSpace(stateSpaceHandles, task));
        task->spaceInformationPtr = ob::SpaceInformationPtr(new ob::SpaceInformation(task->stateSpacePtr));
        task->projectionEvaluatorPtr = ob::ProjectionEvaluatorPtr(new ProjectionEvaluator(stateSpaceHandles, task->stateSpacePtr, task));
        task->stateSpacePtr->registerDefaultProjection(task->projectionEvaluatorPtr);
        task->problemDefinitionPtr = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(task->spaceInformationPtr));
        task->spaceInformationPtr->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidityChecker(task->spaceInformationPtr, task)));
        task->spaceInformationPtr->setStateValidityCheckingResolution(task->stateValidityCheckingResolution);
        task->spaceInformationPtr->setValidStateSamplerAllocator(std::bind(allocValidStateSampler, std::placeholders::_1, task));

        ob::ScopedState<> startState(task->stateSpacePtr);
        validateStateSize(task, task->startState, "Start state");
        for(size_t i = 0; i < task->startState.size(); i++)
            startState[i] = task->startState[i];
        task->problemDefinitionPtr->addStartState(startState);

        ob::GoalPtr goal;
        if(task->goal.type == TaskDef::Goal::STATE)
        {
            for(size_t i = 0; i < task->goal.states.size(); i++)
                validateStateSize(task, task->goal.states[i], "Goal state");

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
                throw std::runtime_error("No goal state specified.");
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
            throw std::runtime_error("Invalid motion planning algorithm.");
        }
        task->planner->setProblemDefinition(task->problemDefinitionPtr);
    }

    void solve(solve_in *in, solve_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        ob::PlannerStatus solved = task->planner->solve(in->maxTime);
        if(solved)
        {
            out->solved = true;

            if(task->verboseLevel >= 1)
            {
                const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
                og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

                std::stringstream s;
                path.print(s);
                sim::addLog(sim_verbosity_infos, "found solution: " + s.str());
            }
        }
        else
        {
            out->solved = false;

            if(task->verboseLevel >= 1)
                sim::addLog(sim_verbosity_infos, "could not find solution.");
        }
    }

    void simplifyPath(simplifyPath_in *in, simplifyPath_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        if(task->verboseLevel >= 2)
            sim::addLog(sim_verbosity_debug, "simplifying solution...");

        const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
        og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

#if OMPL_VERSION_VALUE >= 1001000 // 1.1.0
        og::PathSimplifierPtr pathSimplifier(new og::PathSimplifier(task->spaceInformationPtr, task->problemDefinitionPtr->getGoal()));
#else
        og::PathSimplifierPtr pathSimplifier(new og::PathSimplifier(task->spaceInformationPtr));
#endif
        if(in->maxSimplificationTime < -std::numeric_limits<double>::epsilon())
            pathSimplifier->simplifyMax(path);
        else
            pathSimplifier->simplify(path, in->maxSimplificationTime);

        if(task->verboseLevel >= 1)
        {
            std::stringstream s;
            path.print(s);
            sim::addLog(sim_verbosity_infos, "simplified solution: " + s.str());
        }
    }

    void interpolatePath(interpolatePath_in *in, interpolatePath_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        if(task->verboseLevel >= 2)
            sim::addLog(sim_verbosity_debug, "interpolating solution...");

        const ob::PathPtr &path_ = task->problemDefinitionPtr->getSolutionPath();
        og::PathGeometric &path = static_cast<og::PathGeometric&>(*path_);

        if(in->stateCnt == 0)
            path.interpolate(); // this doesn't give the same result as path.interpolate(0) as I thought!!
        else
            path.interpolate(in->stateCnt);

        if(task->verboseLevel >= 2)
        {
            std::stringstream s;
            path.print(s);
            sim::addLog(sim_verbosity_infos, "interpolated solution: " + s.str());
        }
    }

    void hasSolution(hasSolution_in *in, hasSolution_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);
        out->result = task->problemDefinitionPtr->hasSolution();
    }

    void hasExactSolution(hasExactSolution_in *in, hasExactSolution_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);
        out->result = task->problemDefinitionPtr->hasExactSolution();
    }

    void hasApproximateSolution(hasApproximateSolution_in *in, hasApproximateSolution_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);
        out->result = task->problemDefinitionPtr->hasApproximateSolution();
    }

    void getGoalDistance(getGoalDistance_in *in, getGoalDistance_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);
        out->distance = task->problemDefinitionPtr->getSolutionDifference();
    }

    void getPath(getPath_in *in, getPath_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

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
    }

    void getPlannerData(getPlannerData_in *in, getPlannerData_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);
        ompl::base::PlannerData data(task->spaceInformationPtr);
        task->planner->getPlannerData(data);

        for(size_t i = 0; i < data.numVertices(); i++)
        {
            ompl::base::PlannerDataVertex v(data.getVertex(i));

            const ob::StateSpace::StateType *state = v.getState();
            std::vector<double> stateReals;
            task->stateSpacePtr->copyToReals(stateReals, state);
            for(unsigned int j = 0; j < stateReals.size(); j++)
                out->states.push_back((float)stateReals[j]);

            int tag = v.getTag();
            out->tags.push_back(tag);

            float tagReal;
            memcpy(&tagReal, &tag, sizeof(float));
            out->tagsReal.push_back(tagReal);

            for(size_t j = 0; j < data.numVertices(); j++)
            {
                if(!data.edgeExists(i, j)) continue;

                //ompl::base::PlannerDataEdge e(data.getEdge(i, j));
                out->edges.push_back(i);
                out->edges.push_back(j);

                ompl::base::Cost cost;
                data.getEdgeWeight(i, j, &cost);
                out->edgeWeights.push_back((float)cost.value());
            }
        }

        for(size_t i = 0; i < data.numStartVertices(); i++)
            out->startVertices.push_back(data.getStartIndex(i));

        for(size_t i = 0; i < data.numGoalVertices(); i++)
            out->goalVertices.push_back(data.getGoalIndex(i));
    }

    void compute(compute_in *in, compute_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        solve_in in2;
        in2._ = in->_;
        in2.taskHandle = in->taskHandle;
        in2.maxTime = in->maxTime;
        solve_out out2;
        solve(&in2, &out2);

        out->solved = out2.solved;
        if(!out->solved) return;

        simplifyPath_in in3;
        in3._ = in->_;
        in3.taskHandle = in->taskHandle;
        in3.maxSimplificationTime = in->maxSimplificationTime;
        simplifyPath_out out3;
        simplifyPath(&in3, &out3);

        interpolatePath_in in4;
        in4._ = in->_;
        in4.taskHandle = in->taskHandle;
        in4.stateCnt = in->stateCnt;
        interpolatePath_out out4;
        interpolatePath(&in4, &out4);

        getPath_in in5;
        in5._ = in->_;
        in5.taskHandle = in->taskHandle;
        getPath_out out5;
        getPath(&in5, &out5);

        out->states = out5.states;
    }

    void readState(readState_in *in, readState_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        task->stateSpacePtr->as<StateSpace>()->readState(state);
        std::vector<double> stateVec = state.reals();
        for(size_t i = 0; i < stateVec.size(); i++)
            out->state.push_back((float)stateVec[i]);
    }

    void writeState(writeState_in *in, writeState_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        validateStateSize(task, in->state);

        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        for(int i = 0; i < task->dim; i++)
            state[i] = (double)in->state[i];
        task->stateSpacePtr->as<StateSpace>()->writeState(state);
    }

    void isStateValid(isStateValid_in *in, isStateValid_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        validateStateSize(task, in->state);

        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        for(size_t i = 0; i < in->state.size(); i++)
            state[i] = (double)in->state[i];

        out->valid = task->spaceInformationPtr->isValid(state.get());
    }

    void isStateWithinBounds(isStateWithinBounds_in *in, isStateWithinBounds_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        validateStateSize(task, in->state);

        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        for(size_t i = 0; i < in->state.size(); i++)
            state[i] = (double)in->state[i];

        out->valid = state.satisfiesBounds();
    }

    void enforceBounds(enforceBounds_in *in, enforceBounds_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        validateStateSize(task, in->state);

        ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
        for(size_t i = 0; i < in->state.size(); i++)
            state[i] = (double)in->state[i];
        state.enforceBounds();
        for(size_t i = 0; i < in->state.size(); i++)
            out->state.push_back(state[i]);
    }

    void stateDistance(stateDistance_in *in, stateDistance_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        validateStateSize(task, in->a);
        validateStateSize(task, in->b);

        ob::ScopedState<ob::CompoundStateSpace> a(task->stateSpacePtr), b(task->stateSpacePtr);
        for(size_t i = 0; i < in->a.size(); i++)
        {
            a[i] = (double)in->a[i];
            b[i] = (double)in->b[i];
        }
        out->distance = a.distance(b);
    }

    void projectStates(projectStates_in *in, projectStates_out *out)
    {
        TaskDef *task = getTask(in->taskHandle, true);

        for(size_t h = 0; h < in->state.size(); h += task->dim)
        {
            ob::ScopedState<ob::CompoundStateSpace> state(task->stateSpacePtr);
            for(int i = 0; i < task->dim; i++)
                state[i] = (double)in->state[h + i];

            Eigen::VectorXd projection(task->projectionEvaluatorPtr->getDimension());
            task->projectionEvaluatorPtr->project(state(), projection);
            for(size_t j = 0; j < projection.size(); j++)
                out->projection.push_back((float)projection(j));
        }
    }

    void setProjectionEvaluationCallback(setProjectionEvaluationCallback_in *in, setProjectionEvaluationCallback_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        if(in->projectionSize < 1)
            throw std::runtime_error("Projection size must be positive.");

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
            task->projectionEvaluation.callback.scriptId = in->_.scriptID;
            task->projectionEvaluation.callback.function = in->callback;
        }
    }

    void setStateValidationCallback(setStateValidationCallback_in *in, setStateValidationCallback_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        if(in->callback == "")
        {
            task->stateValidation.type = TaskDef::StateValidation::DEFAULT;
            task->stateValidation.callback.scriptId = 0;
            task->stateValidation.callback.function = "";
        }
        else
        {
            task->stateValidation.type = TaskDef::StateValidation::CLLBACK;
            task->stateValidation.callback.scriptId = in->_.scriptID;
            task->stateValidation.callback.function = in->callback;
        }
    }

    void setGoalCallback(setGoalCallback_in *in, setGoalCallback_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        if(in->callback == "")
            throw std::runtime_error("Invalid callback name.");

        task->goal.type = TaskDef::Goal::CLLBACK;
        task->goal.callback.scriptId = in->_.scriptID;
        task->goal.callback.function = in->callback;
    }

    void setValidStateSamplerCallback(setValidStateSamplerCallback_in *in, setValidStateSamplerCallback_out *out)
    {
        TaskDef *task = getTask(in->taskHandle);

        if(in->callback == "" || in->callbackNear == "")
            throw std::runtime_error("Invalid callback name.");

        task->validStateSampling.type = TaskDef::ValidStateSampling::CLLBACK;
        task->validStateSampling.callback.scriptId = in->_.scriptID;
        task->validStateSampling.callback.function = in->callback;
        task->validStateSampling.callbackNear.scriptId = in->_.scriptID;
        task->validStateSampling.callbackNear.function = in->callbackNear;
    }

private:
    OutputHandler *oh;
    sim::Handles<TaskDef> taskHandles;
    sim::Handles<StateSpaceDef> stateSpaceHandles;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
