/*
 *  HRVOSimulator.cpp
 *  HRVO Library 0.5 beta (based on RVO Library)
 *
 *
 *  Copyright © 2009 University of North Carolina at Chapel Hill. All rights reserved.
 *  
 *  
 *  Permission to use, copy, modify, and distribute this software and its documentation for
 *  educational, research, and non-profit purposes, without fee, and without a written agreement is
 *  hereby granted, provided that the above copyright notice and following three paragraphs
 *  appear in all copies.
 *  
 *  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR ITS EMPLOYEES OR THE
 *  AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 *  DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 *  EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL HAS BEEN ADVISED OF THE POSSIBILITY OF
 *  SUCH DAMAGES.
 *  
 *  THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY DISCLAIM ANY
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE
 *  UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT,
 *  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *  
 *  
 *  Please send all BUG REPORTS to:
 *  
 *  geom@cs.unc.edu
 *  
 *   
 *  The authors may be contacted via:
 *  
 *  Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 *  Department of Computer Science
 *  3175 University of NC
 *  Chapel Hill, NC 27599-3175
 *  United States of America
 *
 *
 *  http://gamma.cs.unc.edu/HRVO/
 *
 */

#include "HRVOSimulator.h"

#include <stdexcept>

#include "Agent.h"
#include "Goal.h"
#include "KdTree.h"
#include "Obstacle.h"
#include "RoadmapVertex.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if HAVE_OPENMP
#include <omp.h>
#endif

#if _OPENMP
#include <omp.h>
#endif

namespace HRVO
{
	HRVOSimulator* HRVOSimulator::Instance_ = 0;
	
	HRVOSimulator::HRVOSimulator()
	: agents_(),
	  areAgentDefaultsSet_(false),
	  areAllAgentsAtGoals_(false),
	  automaticRadius_(-1.0f),
	  defaultAgent_(new Agent()),
	  globalTime_(0.0f),
	  goals_(),
	  isSimulationInitialized_(false),
	  kdTree_(0),	
	  obstacles_(),
	  roadmapVertices_(),
	  timeStep_(0.1f)
	{
	}
	
	HRVOSimulator::~HRVOSimulator()
	{
		delete defaultAgent_;
		
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i)
		{
			delete agents_[i];
		}
		
		for (int i = 0; i < static_cast<int>(obstacles_.size()); ++i)
		{
			delete obstacles_[i];
		}
		
		for (int i = 0; i < static_cast<int>(goals_.size()); ++i)
		{
			delete goals_[i];
		}
		
		for (int i = 0; i < static_cast<int>(roadmapVertices_.size()); ++i)
		{
			delete roadmapVertices_[i];
		}
	}
	
	HRVOSimulator* HRVOSimulator::Instance()
	{
		if (Instance_ == 0)
		{
            //std::srand(static_cast<unsigned int>(std::time(0)));
			Instance_ = new HRVOSimulator();
		}
		
		return Instance_;
	}
	
	int HRVOSimulator::addAgent(const Vector2& position, int goalNo)
	{
		if (isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation already initialized when adding agent");
		}
		
		if (!areAgentDefaultsSet_)
		{
			throw std::runtime_error("Agent defaults not set when adding agent.");
		}
		
		Agent* agent = new Agent(position, goalNo);
		agents_.push_back(agent);
		
		return static_cast<int>(agents_.size()) - 1;
	}
	
	int HRVOSimulator::addAgent(const Vector2& position, int goalNo, float neighborDist,
								int maxNeighbors, float radius, float goalRadius, float prefSpeed,
								float maxSpeed, 
#if DIFFERENTIAL_DRIVE
								float timeToOrientation, float wheelTrack,
#endif
								float uncertaintyOffset, float maxAccel, const Vector2& velocity,
								float orientation, int classNo)
	{
		if (isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation already initialized when adding agent.");
		}
		
		Agent* agent = new Agent(position, goalNo, neighborDist, maxNeighbors, classNo, radius,
								 velocity, maxAccel, goalRadius, prefSpeed, maxSpeed, orientation,
#if DIFFERENTIAL_DRIVE								 
								 timeToOrientation, wheelTrack,
#endif
								 uncertaintyOffset);
		agents_.push_back(agent);
		
		return static_cast<int>(agents_.size()) - 1;
	}
	
	int HRVOSimulator::addGoal(const Vector2& positionition)
	{
		if (isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation already initialized when adding goal.");
		}
		
		Goal* goal = new Goal(positionition);
		goals_.push_back(goal);
		
		return static_cast<int>(goals_.size()) - 1;
	}
	
	int HRVOSimulator::addObstacle(const Vector2& point1, const Vector2& point2)
	{
		if (isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation already initialized when adding static obstacle.");
		}
		
		Obstacle* obstacle = new Obstacle(point1, point2);
		obstacles_.push_back(obstacle);
		
		return static_cast<int>(obstacles_.size()) - 1;
	}
		
	void HRVOSimulator::addRoadmapEdge(int roadmapVertexNo1, int roadmapVertexNo2)
	{
		if (isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation already initialized when adding roadmap edge.");
		}
		
		float dist = abs(roadmapVertices_[roadmapVertexNo1]->position_ 
						 - roadmapVertices_[roadmapVertexNo2]->position_);
		roadmapVertices_[roadmapVertexNo1]->addNeighbor(dist, roadmapVertexNo2);
		roadmapVertices_[roadmapVertexNo2]->addNeighbor(dist, roadmapVertexNo1);
	}
	
	int HRVOSimulator::addRoadmapVertex(const Vector2& position)
	{
		if (isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation already initialized when adding roadmap vertex.");
		}
		
		RoadmapVertex * vertex = new RoadmapVertex(position);
		roadmapVertices_.push_back(vertex);
		
		return static_cast<int>(roadmapVertices_.size()) - 1;
	}
	
	void HRVOSimulator::doStep()
	{
		if (! isSimulationInitialized_)
		{
			throw std::runtime_error("Simulation not initialized when attempting to do step.");
		}
		
		if (timeStep_ == 0.0f)
		{
			throw std::runtime_error("Time step not set when attempting to do step.");
		}
		
		areAllAgentsAtGoals_ = true;
		
		kdTree_->buildAgentTree();
		
#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i)
		{
			agents_[i]->computePreferredVelocity();
			agents_[i]->computeNeighbors();
			agents_[i]->computeNewVelocity();
#if DIFFERENTIAL_DRIVE
			agents_[i]->computeWheelSpeeds();
#endif
		}
		
#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i)
		{
			agents_[i]->update();
		}
		
		globalTime_ += timeStep_;
	}
	
	void HRVOSimulator::initSimulation()
	{
		kdTree_ = new KdTree();
		kdTree_->buildObstacleTree();
		
		if (automaticRadius_ >= 0.0f)
		{
#pragma omp parallel for
			for (int i = 0; i < static_cast<int>(roadmapVertices_.size()); ++i)
			{
				roadmapVertices_[i]->computeNeighbors(automaticRadius_);
			}
		}
		
#pragma omp parallel for
		for (int i = 0; i < static_cast<int>(goals_.size()); ++i)
		{
			goals_[i]->computeShortestPathTree();
		}
		
		isSimulationInitialized_ = true;
	}
	
	int HRVOSimulator::getAgentClass(int agentNo) const
	{
		return agents_[agentNo]->classNo_;
	}
	
	int HRVOSimulator::getAgentGoal(int agentNo) const
	{
		return agents_[agentNo]->goalNo_;
	}
	
	float HRVOSimulator::getAgentGoalRadius(int agentNo) const
	{
		return agents_[agentNo]->goalRadius_;
	}
	
#if DIFFERENTIAL_DRIVE
	float HRVOSimulator::getAgentLeftWheelSpeed(int agentNo) const
	{
		return agents_[agentNo]->leftWheelSpeed_;
	}
#endif
	
	float HRVOSimulator::getAgentMaxAccel(int agentNo) const
	{
		return agents_[agentNo]->maxAccel_;
	}
	
	int HRVOSimulator::getAgentMaxNeighbors(int agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}
	
	float HRVOSimulator::getAgentMaxSpeed(int agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}
	
	float HRVOSimulator::getAgentNeighborDist(int agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}
	
	float HRVOSimulator::getAgentOrientation(int agentNo) const
	{
		return agents_[agentNo]->orientation_;
	}
	
	const Vector2& HRVOSimulator::getAgentPosition(int agentNo) const
	{
		return agents_[agentNo]->position_;
	}
	
	float HRVOSimulator::getAgentPrefSpeed(int agentNo) const
	{
		return agents_[agentNo]->prefSpeed_;
	}
	
	float HRVOSimulator::getAgentRadius(int agentNo) const
	{
		return agents_[agentNo]->radius_;
	}
	
	bool HRVOSimulator::getAgentReachedGoal(int agentNo) const
	{
		return agents_[agentNo]->isAtGoal_;
	}
	
#if DIFFERENTIAL_DRIVE
	float HRVOSimulator::getAgentRightWheelSpeed(int agentNo) const
	{
		return agents_[agentNo]->rightWheelSpeed_;
	}

	float HRVOSimulator::getAgentTimeToOrientation(int agentNo) const
	{
		return agents_[agentNo]->timeToOrientation_;
	}
#endif
	
	float HRVOSimulator::getAgentUncertaintyOffset(int agentNo) const
	{
		return agents_[agentNo]->uncertaintyOffset_;
	}
	
	const Vector2& HRVOSimulator::getAgentVelocity(int agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}
	
#if DIFFERENTIAL_DRIVE
	float HRVOSimulator::getAgentWheelTrack(int agentNo) const
	{
		return agents_[agentNo]->wheelTrack_;
	}
#endif
	
	float HRVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}
	
	int HRVOSimulator::getGoalNeighbor(int goalNo, int neighborNo) const
	{
		return goals_[goalNo]->roadmapVertex_->neighbors_[neighborNo].second;
	}
	
	int HRVOSimulator::getGoalNumNeighbors(int goalNo) const
	{
		return static_cast<int>(goals_[goalNo]->roadmapVertex_->neighbors_.size());
	}
	
	const Vector2& HRVOSimulator::getGoalPosition(int goalNo) const
	{
		return goals_[goalNo]->roadmapVertex_->position_;
	}
	
	int HRVOSimulator::getNumAgents() const
	{
		return static_cast<int>(agents_.size());
	}
	
	int HRVOSimulator::getNumGoals() const
	{
		return static_cast<int>(goals_.size());
	}
	
	int HRVOSimulator::getNumObstacles() const
	{
		return static_cast<int>(obstacles_.size());
	}
	
	int HRVOSimulator::getNumRoadmapVertices() const
	{
		return static_cast<int>(roadmapVertices_.size());
	}
	
	const Vector2& HRVOSimulator::getObstaclePoint1(int obstacleNo) const
	{
		return obstacles_[obstacleNo]->point1_;
	}
	
	const Vector2& HRVOSimulator::getObstaclePoint2(int obstacleNo) const
	{
		return obstacles_[obstacleNo]->point2_;
	}
	
	bool HRVOSimulator::getReachedGoal() const
	{
		return areAllAgentsAtGoals_;
	}
	
	int HRVOSimulator::getRoadmapVertexNeighbor(int roadmapVertexNo, int neighborNo) const
	{
		return roadmapVertices_[roadmapVertexNo]->neighbors_[neighborNo].second;
	}
	
	int HRVOSimulator::getRoadmapVertexNumNeighbors(int roadmapVertexNo) const
	{
		return static_cast<int>(roadmapVertices_[roadmapVertexNo]->neighbors_.size());
	}
	
	const Vector2& HRVOSimulator::getRoadmapVertexPosition(int roadmapVertexNo) const
	{
		return roadmapVertices_[roadmapVertexNo]->position_;
	}
	
	float HRVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}
	
	void HRVOSimulator::setAgentClass(int agentNo, int classNo)
	{
		agents_[agentNo]->classNo_ = classNo;
	}
	
	void HRVOSimulator::setAgentDefaults(float neighborDist, int maxNeighbors, float radius, 
										 float goalRadius, float prefSpeed, float maxSpeed,
#if DIFFERENTIAL_DRIVE
										 float timeToOrientation, float wheelTrack,
#endif
										 float uncertaintyOffset, float maxAccel, 
										 const Vector2& velocity, float orientation, 
										 int classNo)
	{
		defaultAgent_->classNo_ = classNo;
		defaultAgent_->goalRadius_ = goalRadius;
		defaultAgent_->maxAccel_ = maxAccel;
		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->newVelocity_ = velocity;
		defaultAgent_->uncertaintyOffset_ = uncertaintyOffset;
		defaultAgent_->orientation_ = orientation;
		defaultAgent_->prefSpeed_ = prefSpeed;
		defaultAgent_->radius_ = radius;
#if DIFFERENTIAL_DRIVE
		defaultAgent_->timeToOrientation_ = timeToOrientation;
#endif
		defaultAgent_->velocity_ = velocity;
		
#if DIFFERENTIAL_DRIVE
		defaultAgent_->wheelTrack_ = wheelTrack;
		
		defaultAgent_->computeWheelSpeeds();
#endif

		areAgentDefaultsSet_ = true;
	}
	
	void HRVOSimulator::setAgentGoal(int agentNo, int goalNo)
	{
		agents_[agentNo]->goalNo_ = goalNo;
	}
	
	void HRVOSimulator::setAgentGoalRadius(int agentNo, float goalRadius)
	{
		agents_[agentNo]->goalRadius_ = goalRadius;
	}
	
	void HRVOSimulator::setAgentMaxAccel(int agentNo, float maxAccel)
	{
		agents_[agentNo]->maxAccel_ = maxAccel;
	}
	
	void HRVOSimulator::setAgentMaxNeighbors(int i, int maxNeighbors)
	{
		agents_[i]->maxNeighbors_ = maxNeighbors;
	}
	
	void HRVOSimulator::setAgentMaxSpeed(int i, float maxSpeed)
	{
		agents_[i]->maxSpeed_ = maxSpeed;
	}
	
	void HRVOSimulator::setAgentNeighborDist(int i, float neighborDist)
	{
		agents_[i]->neighborDist_ = neighborDist;
	}
	
	void HRVOSimulator::setAgentOrientation(int i, float orientation)
	{
		agents_[i]->orientation_ = orientation;
	}
	
	void HRVOSimulator::setAgentPosition(int i, const Vector2& position)
	{
		agents_[i]->position_ = position;
	}
	
	void HRVOSimulator::setAgentPrefSpeed(int i, float prefSpeed)
	{
		agents_[i]->prefSpeed_ = prefSpeed;
	}
	
	void HRVOSimulator::setAgentRadius(int i, float radius)
	{
		agents_[i]->radius_ = radius;
	}
	
#if DIFFERENTIAL_DRIVE
	void HRVOSimulator::setAgentTimeToOrientation(int i, float timeToOrientation)
	{
		agents_[i]->timeToOrientation_ = timeToOrientation;
	}
#endif
	
	void HRVOSimulator::setAgentUncertaintyOffset(int i, float uncertaintyOffset)
	{
		agents_[i]->uncertaintyOffset_ = uncertaintyOffset;
	}
	
	void HRVOSimulator::setAgentVelocity(int i, const Vector2& velocity)
	{
		agents_[i]->velocity_ = velocity;
	}
	
#if DIFFERENTIAL_DRIVE
	void HRVOSimulator::setAgentWheelTrack(int i, float wheelTrack)
	{
		agents_[i]->wheelTrack_ = wheelTrack;
	}
#endif
	
	void HRVOSimulator::setRoadmapAutomatic(float automaticRadius)
	{
		automaticRadius_ = automaticRadius;
	}	
	
	void HRVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
}
