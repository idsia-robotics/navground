/*
 *  HRVOSimulator.h
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

#ifndef HRVO_SIMULATOR_H
#define HRVO_SIMULATOR_H

#include <limits>
#include <vector>

#include "Vector2.h"


/*!
 *  @defined 
 *  @abstract    Set to 0 for a holonomic agent; set to 1 for a differential-drive agent.
 */
#define DIFFERENTIAL_DRIVE 1

namespace HRVO
{
	class Agent;
	class Goal;
	class KdTree;
	class Obstacle;
	class RoadmapVertex;
	
	/*!
	 *  @class
	 *  @abstract   Defines the simulation.
	 */
	class HRVOSimulator
	{
	public:
		/*!
		 *  @function
		 *  @abstract   Destroys this simulator instance.
		 */
		~HRVOSimulator();
		
		/*!
		 *  @function
		 *  @abstract   Returns a pointer to the singleton simulator instance.
		 *  @result     A pointer to the singleton simulator instance.
		 */
		static HRVOSimulator* Instance();
		
		/*!
		 *  @function
		 *  @abstract   Adds a new agent with default properties to the simulation.
		 *  @param      position  The two-dimensional starting position of this agent.
		 *  @param      goalNo  The goal number of this agent.
		 *  @result     The (non-negative) number of the agent.
		 */
		int addAgent(const Vector2& position, int goalNo);
		
		/*!
		 *  @function
		 *  @abstract   Adds a new agent to the simulation.
		 *  @param      position  The two-dimensional starting position of this agent.
		 *  @param      goalNo  The goal number of this agent.
		 *  @param      neighborDist  The maximum neighbor distance of this agent.
		 *  @param      maxNeighbors  The maximum neighbor count of this agent.
		 *  @param      radius  The radius of this agent.
		 *  @param      goalRadius  The goal radius of this agent.
		 *  @param      prefSpeed  The preferred speed of this agent.
		 *  @param      maxSpeed  The maximum speed of this agent.
		 *  @param      uncertaintyOffset  The uncertainty offset of this agent.
		 *  @param      maxAccel  The maximum linear acceleration of this agent (optional).
		 *  @param      velocity  The initial two-dimensional linear velocity of this agent
		 *              (optional).
		 *  @param      orientation  The initial orientation (in radians) of this agent (optional).
		 *  @param      classNo  The (non-negative) class number of this agent (optional).
		 *  @result     The (non-negative) number of the agent.
		 */
		int addAgent(const Vector2& position, int goalNo, float neighborDist, int maxNeighbors,
					 float radius, float goalRadius, float prefSpeed, float maxSpeed, 
#if DIFFERENTIAL_DRIVE
					 float timeToOrientation, float wheelTrack,
#endif					 
					 float uncertaintyOffset = 0.0f, 
					 float maxAccel = std::numeric_limits<float>::infinity(),
					 const Vector2& velocity = Vector2(0.0f, 0.0f), float orientation = 0.0f,
					 int classNo = 0);
		
		/*!
		 *  @function
		 *  @abstract   Adds a new goal to the simulation.
		 *  @param      position  The two-dimensional position of this goal.
		 *  @result     The (non-negative) number of the goal.
		 */
		int addGoal(const Vector2& position);

		/*!
		 *  @function
		 *  @abstract   Adds a new static obstacle to the simulation.
		 *  @param      point1  The two-dimensional position of the first endpoint of this static
		 *                      obstacle.
		 *  @param      point2  The two-dimensional position of the second endpoint of this static
		 *						obstacle.
		 *  @result     The (non-negative) number of the static obstacle.
		 */
		int addObstacle(const Vector2& point1, const Vector2& point2);
		
		/*!
		 *  @function
		 *  @abstract   Adds a new roadmap edge to the simulation.
		 *  @param      roadmapVertexNo1  The number of the roadmap vertex which is to be the first
		 *                                endpoint of this roadmap edge.
		 *  @param      roadmapVertexNo2  The number of the roadmap vertex which is to be the second
		 *                                endpoint of this roadmap edge.
		 */
		void addRoadmapEdge(int roadmapVertexNo1, int roadmapVertexNo2);
		
		/*!
		 *  @function
		 *  @abstract   Adds a new roadmap vertex to the simulation.
		 *  @param      position  The two-dimensional position of this roadmap vertex.
		 *  @result     The (non-negative) number of the roadmap vertex.
		 */
		int addRoadmapVertex(const Vector2& position);
		
		/*!
		 *  @function
		 *  @abstract   Performs a simulation step; updates the orientation, two-dimensional
		 *              position, and two-dimensional velocity of each agent, and the progress of
		 *              each towards its goal.  
		 */
		void doStep();
		
		/*!
		 *  @function
		 *  @abstract   Returns the class number of a specified agent.
		 *  @param      agentNo  The number of the agent whose class number is to be retrieved.
		 *  @result     The present (non-negative) class number of the agent.
		 */
		int getAgentClass(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the goal number of a specified agent.
		 *  @param      agentNo  The number of the agent whose goal number is to be retrieved.
		 *  @result     The present goal number of the agent.
		 */
		int getAgentGoal(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the goal radius of a specified agent.
		 *  @param      agentNo  The number of the agent whose goal radius is to be retrieved.
		 *  @result     The present goal radius of the agent
		 */
		float getAgentGoalRadius(int agentNo) const;

#if DIFFERENTIAL_DRIVE
		/*!
		 *  @function
		 *  @abstract   Returns the left wheel speed of a specified agent.
		 *  @param      agentNo  The number of the agent whose left wheel speed is to be retrieved.
		 *  @result     The present (signed) left wheel speed of the agent.
		 */
		float getAgentLeftWheelSpeed(int agentNo) const;
#endif
		
		/*!
		 *  @function
		 *  @abstract   Returns the maximum linear acceleration of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum linear acceleration is to be
		 *                       retrieved.
		 *  @result     The present maximum linear acceleration of the agent.
		 */
		float getAgentMaxAccel(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the maximum neighbor count of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum neighbor count is to be
		 *                       retrieved.
		 *  @result     The present maximum neighbor count of the agent.
		 */
		int getAgentMaxNeighbors(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the maximum speed of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum speed is to be retrieved.
		 *  @result     The present maximum speed of the agent.
		 */
		float getAgentMaxSpeed(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the maximum neighbor distance of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum neighbor distance is to be
		 *                       retrieved.
		 *  @result     The present maximum neighbor distance of the agent.
		 */
		float getAgentNeighborDist(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the orientation of a specified agent.
		 *  @param      agentNo  The number of the agent whose orientation is to be retrieved.
		 *  @result     The present orientation (in radians) of the agent.
		 */
		float getAgentOrientation(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the two-dimensional position of a specified agent.
		 *  @param      agentNo  The number of the agent whose two-dimensional position is to be
		 *                       retrieved.
		 *  @result     The present two-dimensional position of the (center of) the agent.
		 */
		const Vector2& getAgentPosition(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the preferred speed of a specified agent.
		 *  @discussion The preferred speed of an agent is the speed it would choose to take if it
		 *              were not influenced by other agents or static obstacles.
		 *  @param      agentNo  The number of the agent whose preferred speed is to be retrieved.
		 *  @result     The present preferred speed of the agent.
		 */
		float getAgentPrefSpeed(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the radius of a specified agent.
		 *  @param      agentNo  The number of the agent whose radius is to be retrieved.
		 *  @result     The present radius of the agent.
		 */
		float getAgentRadius(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the progress towards its goal of a specified agent.
		 *  @param      agentNo  The number of the agent whose progress towards its goal is to be
		 *                       retrieved.
		 *  @result     True if the agent has reached its goal; false otherwise.
		 */
		bool getAgentReachedGoal(int agentNo) const;

#if DIFFERENTIAL_DRIVE
		/*!
		 *  @function
		 *  @abstract   Returns the right wheel speed of a specified agent.
		 *  @param      agentNo  The number of the agent whose right wheel speed is to be retrieved.
		 *  @result     The present (signed) right wheel speed of the agent.
		 */
		float getAgentRightWheelSpeed(int agentNo) const;

		/*!
		 *  @function
		 *  @abstract   Returns the "time to orientation" of a specified agent.
		 *  @discussion The time to orientation is the time period that a differential-drive agent 
		 *              is given to assume the orientation defined by its new linear velocity.
		 *  @param      agentNo  The number of the agent whose time to orientation is to be
		 *                       retrieved.
		 *  @result     The present time to orientation of the agent.
		 */
		float getAgentTimeToOrientation(int agentNo) const;
#endif
		
		/*!
		 *  @function
		 *  @abstract   Returns the "uncertainty offset" of a specified agent.
		 *  @discussion The uncertainty offset is the amount velocity obstacles are widened to 
		 *              allow for uncertainty in the two-dimensional position and velocity of a 
		 *              differential drive agent.
		 *  @param      agentNo  The number of the agent whose uncertainty offset is to be
		 *                       retrieved.
		 *  @result     The present uncertainty offset of the agent.
		 */
		float getAgentUncertaintyOffset(int agentNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the two-dimensional linear velocity of a specified agent.
		 *  @param      agentNo  The number of the agent whose two-dimensional linear velocity is to
		 *                       be retrieved.
		 *  @result     The present two-dimensional linear velocity of the agent.
		 */
		const Vector2& getAgentVelocity(int agentNo) const;
	
#if DIFFERENTIAL_DRIVE
		/*!
		 *  @function
		 *  @abstract   Returns the wheel track of a specified agent.
		 *  @param      agentNo  The number of the agent whose wheel track is to be retrieved.
		 *  @result     The present wheel track of the agent.
		 */
		float getAgentWheelTrack(int agentNo) const;
#endif
		
		/*!
		 *  @function
		 *  @abstract   Returns the global time of the simulation.
		 *  @result     The present global time of the simulation (zero initially).
		 */
		float getGlobalTime() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the nth-nearest neighbor of a specified goal.
		 *  @param      goalNo  The number of the goal whose nth-nearest neighbor is to be
		 *                      retrieved.
		 *  @param      neighborNo  The number n for the nth-nearest neighbor calculation.
		 *  @result     The roadmap vertex number of the nth-nearest neighbor of the goal.
		 */
		int getGoalNeighbor(int goalNo, int neighborNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the neighbor count of a specified goal.
		 *  @param      goalNo  The number of the goal whose neighbor count is to be retrieved.
		 *  @result     The count of roadmap vertices that neighbor the goal.
		 */
		int getGoalNumNeighbors(int goalNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the two-dimensional position of a specified goal.
		 *  @param      goalNo  The number of the goal whose two-dimensional position is to be
		 *                      retrieved.
		 *  @result     The two-dimensional position of the goal.
		 */
		const Vector2& getGoalPosition(int goalNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the count of agents in the simulation.
		 *  @result     The count of agents in the simulation.
		 */
		int getNumAgents() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the count of goals in the simulation.
		 *  @result     The count of goals in the simulation.
		 */
		int getNumGoals() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the count of static obstacles in the simulation.
		 *  @result     The count of static obstacles in the simulation.
		 */
		int getNumObstacles() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the count of roadmap vertices in the simulation.
		 *  @result     The count of roadmap vertices in the simulation.
		 */
		int getNumRoadmapVertices() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the two-dimensional position of the first endpoint of a specified
		 *              static obstacle.
		 *  @param      obstacleNo  The number of the static obstacle whose first endpoint is to be
		 *                          retrieved.
		 *  @result     The two-dimensional position of the first endpoint of the specified static
		 *              obstacle.
		 */
		const Vector2& getObstaclePoint1(int obstacleNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the two-dimensional position of the second endpoint of a specified
		 *              static obstacle.
		 *  @param      obstacleNo  The number of the static obstacle whose second endpoint is to be
		 *                          retrieved.
		 *  @result     The two-dimensional position of the second endpoint of the specified static
		 *              obstacle.
		 */
		const Vector2& getObstaclePoint2(int obstacleNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the progress towards their goals of all agents.
		 *  @result     True if all agents have reached their goals; false otherwise.
		 */
		bool getReachedGoal() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the nth-nearest neighbor of a specified roadmap vertex.
		 *  @param      roadmapVertexNo  The number of the roadmap vertex whose nth-nearest neighbor
		 *                               is to be retrieved.
		 *  @param      neighborNo  The number n for the nth-nearest neighbor calculation.
		 *  @result     The roadmap vertex number of the nth-nearest neighbor of the roadmap vertex.
		 */
		int getRoadmapVertexNeighbor(int roadmapVertexNo, int neighborNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the neighbor count of a specified roadmap vertex.
		 *  @param      roadmapVertexNo  The number of the roadmap vertex whose neighbor count is to
		 *                               be retrieved.
		 *  @result     The count of roadmap vertices that neighbor the roadmap vertex.
		 */
		int getRoadmapVertexNumNeighbors(int roadmapVertexNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the two-dimensional position of a specified roadmap vertex.
		 *  @param      roadmapVertexNo  The number of the roadmap vertex whose two-dimensional
		 *                               position is to be retrieved.
		 *  @result     The two-dimensional position of the roadmap vertex.
		 */
		const Vector2& getRoadmapVertexPosition(int roadmapVertexNo) const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the time step of the simulation.
		 *  @result     The present time step of the simulation.
		 */
		float getTimeStep() const;
		
		/*!
		 *  @function
		 *  @abstract   Processes the agents, goals, roadmap vertices and edges, and static
		 *              obstacles that have been added, and begins the simulation.  
		 *  @discussion No further agents, goals, roadmap vertices and edges, or static obstacles
		 *              may be added to the simulation after this function has been called.
		 */
		void initSimulation();
		
		/*!
		 *  @function
		 *  @abstract   Sets the class number of a specified agent.
		 *  @param      agentNo  The number of the agent whose class number is to be modified.
		 *  @param      classNo  The replacement (non-negative) class number.
		 */
		void setAgentClass(int agentNo, int classNo);
		
		/*!
		 *  @function
		 *  @abstract   Sets the default properties for any new agent that is added.
		 *  @param      neighborDist  The default maximum neighbor distance of a new agent.
		 *  @param      maxNeighbors  The default maximum neighbor count of a new agent.
		 *  @param      radius  The default radius of a new agent.
		 *  @param      goalRadius  The default goal radius of a new agent.
		 *  @param      prefSpeed  The default preferred speed of a new agent.
		 *  @param      maxSpeed  The default maximum speed of a new agent.
		 *  @param      uncertaintyOffset  The default uncertainty offset of a new agent.
		 *  @param      maxAccel  The default maximum linear acceleration of a new agent (optional).
		 *  @param      velocity  The default initial two-dimensional linear velocity of a new agent
		 *                        (optional).
		 *  @param      orientation  The default initial orientation (in radians) of a new agent
		 *                           (optional).
		 *  @param      classNo  The default (non-negative) class number of a new agent (optional).
		 */
		void setAgentDefaults(float neighborDist, int maxNeighbors, float radius, float goalRadius,
							  float prefSpeed, float maxSpeed, 
#if DIFFERENTIAL_DRIVE
							  float timeToOrientation, float wheelTrack,
#endif							  
							  float uncertaintyOffset,
							  float maxAccel = std::numeric_limits<float>::infinity(),
							  const Vector2& velocity = Vector2(), float orientation = 0.0f,
							  int classNo = 0);
		
		/*!
		 *  @function
		 *  @abstract   Sets the goal number of a specified agent.
		 *  @param      agentNo  The number of the agent whose goal number is to be modified.
		 *  @param      goalNo  The replacement goal number.
		 */
		void setAgentGoal(int agentNo, int goalNo);
		
		/*!
		 *  @function
		 *  @abstract   Sets the goal radius of a specified agent.
		 *  @param      agentNo  The number of the agent whose goal radius is to be modified.
		 *  @param      goalRadius  The replacement goal radius.
		 */
		void setAgentGoalRadius(int agentNo, float goalRadius);
		
		/*!
		 *  @function
		 *  @abstract   Sets the maximum linear acceleraton of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum linear acceleration is to be
		 *                       modified.
		 *  @param      maxAccel  The replacement maximum linear acceleration.
		 */
		void setAgentMaxAccel(int agentNo, float maxAccel);
		
		/*!
		 *  @function
		 *  @abstract   Sets the maximum neighbor count of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum neighbor count is to be
		 *                       modified.
		 *  @param      maxNeighbors  The replacement maximum neighbor count.
		 */
		void setAgentMaxNeighbors(int agentNo, int maxNeighbors);
		
		/*!
		 *  @function
		 *  @abstract   Sets the maximum speed of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum speed is to be modified.
		 *  @param      maxSpeed  The replacement maximum speed.
		 */
		void setAgentMaxSpeed(int agentNo, float maxSpeed);
		
		/*!
		 *  @function
		 *  @abstract   Sets the maximum neighbor distance of a specified agent.
		 *  @param      agentNo  The number of the agent whose maximum neighbor distance is to be
		 *                       modified.
		 *  @param      neighborDist  The replacement maximum neighbor distance.
		 */
		void setAgentNeighborDist(int agentNo, float neighborDist);
		
		/*!
		 *  @function
		 *  @abstract   Sets the orientation of a specified agent.
		 *  @param      agentNo  The number of the agent whose orientation is to be modified.
		 *  @param      orientation  The replacement orientation (in radians).
		 */
		void setAgentOrientation(int agentNo, float orientation);
		
		/*!
		 *  @function
		 *  @abstract   Sets the two-dimensional position of a specified agent.
		 *  @param      agentNo  The number of the agent whose two-dimensional position is to be
		 *                       modified.
		 *  @param      position  The replacement two-dimensional position.
		 */
		void setAgentPosition(int agentNo, const Vector2& position);
		
		/*!
		 *  @function
		 *  @abstract   Sets the preferred speed of a specified agent.
		 *  @discussion The preferred speed of an agent is the speed it would choose to take if it
		 *              were not influenced by other agents or static obstacles.
		 *  @param      agentNo  The number of the agent whose preferred speed is to be modified.
		 *  @param      prefSpeed  The replacement preferred speed.
		 */
		void setAgentPrefSpeed(int agentNo, float prefSpeed);
		
		/*!
		 *  @function
		 *  @abstract   Sets the radius of a specified agent.
		 *  @param      agentNo  The number of the agent whose radius is to be modified.
		 *  @param      radius  The replacement radius.
		 */
		void setAgentRadius(int agentNo, float radius);
		
#if DIFFERENTIAL_DRIVE
		/*!
		 *  @function
		 *  @abstract   Sets the "time to orientation" of a specified agent.
		 *  @discussion The time to orientation is the time period that a differential-drive agent 
		 *              is given to assume the orientation defined by its new linear velocity.
		 *  @param      agentNo  The number of the agent whose time to orientation is to be
		 *                       modified.
		 *  @param      timeToOrientation  The replacement time to orientation.
		 */
		void setAgentTimeToOrientation(int agentNo, float timeToOrientation);
		
		/*!
		 *  @function
		 *  @abstract   Sets the wheel track of a specified agent.
		 *  @param      agentNo  The number of the agent whose wheel track is to be modified.
		 *  @param      wheelTrack  The replacement wheel track.
		 */
		void setAgentWheelTrack(int agentNo, float wheelTrack);
#endif
		
		/*!
		 *  @function
		 *  @abstract   Sets the "uncertainty offset" of a specified agent.
		 *  @discussion The uncertainty offset is the amount velocity obstacles are widened to 
		 *              allow for uncertainty in the two-dimensional position and velocity of a 
		 *              differential drive agent.
		 *  @param      agentNo  The number of the agent whose uncertainty offset is to be modified.
		 *  @param      uncertaintyOffset  The replacement uncertainty offset.
		 */
		void setAgentUncertaintyOffset(int agentNo, float uncertaintyOffset);
		
		/*!
		 *  @function
		 *  @abstract   Sets the two-dimensional linear velocity of a specified agent.
		 *  @param      agentNo  The number of the agent whose two-dimensional linear velocity is to
		 *                       be modified.
		 *  @param      velocity  The replacement two-dimensional linear velocity.
		 */
		void setAgentVelocity(int agentNo, const Vector2& velocity);
		
		/*!
		 *  @function
		 *  @abstract   Sets the radius within which neighboring roadmap vertices should
		 *              automatically be connected by roadmap edges.
		 *  @param      automaticRadius  If positive, the radius within which neighboring roadmap
		 *                               vertices should automatically be connected; otherwise no
		 *                               edges will be automatically be connected.
		 */
		void setRoadmapAutomatic(float automaticRadius);
		
		/*!
		 *  @function
		 *  @abstract   Sets the time step of the simulation.
		 *  @param      timeStep  The time step of the simulation.
		 */
		void setTimeStep(float timeStep);
		
	private:
		/*!
		 *  @function
		 *  @abstract   Constructs a simulator instance.
		 */
		HRVOSimulator();
		
		static HRVOSimulator* Instance_;
		
		std::vector<Agent*> agents_;
		bool areAgentDefaultsSet_;
		bool areAllAgentsAtGoals_;
		float automaticRadius_;
		Agent* defaultAgent_;
		float globalTime_;
		std::vector<Goal*> goals_;
		bool isSimulationInitialized_;
		KdTree* kdTree_;
		std::vector<Obstacle*>  obstacles_;
		std::vector<RoadmapVertex*> roadmapVertices_;
		float timeStep_;
		
		friend class Agent;
		friend class Goal;
		friend class KdTree;
		friend class Obstacle;
		friend class RoadmapVertex;
	};	
}

#endif
