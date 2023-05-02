/*
 *  Agent.h
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

#ifndef AGENT_HRVO_H
#define AGENT_HRVO_H

#include "HRVODef.h"
#include "HRVOSimulator.h"

namespace HRVO
{
	/*!
	 *  @class
	 *  @abstract   Defines an agent in the simulation.
	 */
	class Agent
	{
	public:
		/*!
		 *  @struct
		 *  @abstract   Defines a sample point.
		 *  @field      position  The two-dimensional position of the sample point.
		 *  @field      velocityObstacleNo1  The number of the first velocity obstacle.
		 *  @field      velocityObstacleNo2  The number of the second velocity obstacle.
		 */
		struct SamplePoint
		{
			Vector2 position;
			int velocityObstacleNo1;
			int velocityObstacleNo2;
		};

		/*!
		 *  @struct
		 *  @abstract   Defines a velocity obstacle.
		 *  @field      apex  The two-dimensional position of the apex of the velocity obstacle.
		 *  @field      side1  The direction of the first side of the velocity obstacle.
		 *  @field      side2  The direction of the second side of the velocity obstacle.
		 */
		struct VelocityObstacle
		{
			Vector2 apex;
			Vector2 side1;
			Vector2 side2;
		};

		/*!
		 *  @function
		 *  @abstract   Constructs an agent instance.
		 */
		Agent();

		/*!
		 *  @function
		 *  @abstract   Constructs an agent instance with default properties.
		 *  @param      position  The two-dimensional starting position of this agent.
		 *  @param      goalNo  The goal number of this agent.
		 */
		Agent(const Vector2& position, int goalNo);

		/*!
		 *  @function
		 *  @abstract   Constructs an agent instance with the specified properties.
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
		 */
		Agent(const Vector2& position, int goalNo, float neighborDist, int maxNeighbors,
			  int classNo, float radius, const Vector2& velocity, float maxAccel, float goalRadius,
			  float prefSpeed, float maxSpeed, float orientation,
#if DIFFERENTIAL_DRIVE
			  float timeToOrientation, float wheelTrack,
#endif
			  float uncertaintyOffset);
		/*!
		 *  @function
		 *  @abstract   Destroys this agent instance.
		 */
		~Agent();

		/*!
		 *  @function
		 *  @abstract   Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/*!
		 *  @function
		 *  @abstract   Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/*!
		 *  @function
		 *  @abstract   Computes the preferred velocity of this agent.
		 */
		void computePreferredVelocity();

#if DIFFERENTIAL_DRIVE
		/*!
		 *  @function
		 *  @abstract   Computes the wheel speeds of this agent.
		 */
		void computeWheelSpeeds();
#endif

		/*!
		 *  @function
		 *  @abstract   Inserts an agent neighbor into the set of neighbors of this agent.
		 *  @param      agentNo  The number of the agent to be inserted.
		 *  @param      rangeSq  The squared range around this agent.
		 */
		void insertAgentNeighbor(int agentNo, float& rangeSq);

		/*!
		 *  @function
		 *  @abstract   Inserts a static obstacle neighbor into the set of neighbors of this agent.
		 *  @param      agentNo  The number of the static obstacle to be inserted.
		 *  @param      rangeSq  The squared range around this agent.
		 */
		void insertObstacleNeighbor(int obstacleNo, float& rangeSq);

		/*!
		 *  @function
		 *  @abstract   Updates the orientation, two-dimensional position, and two-dimensional
		 *              velocity of this agent.
		 */
		void update();


        /*! by me
         */


		int classNo_;
		int goalNo_;
		float goalRadius_;
		bool isAtGoal_;
		bool isColliding_;

#if DIFFERENTIAL_DRIVE
		float leftWheelSpeed_;
#endif

		float maxAccel_;
		int maxNeighbors_;
		float maxSpeed_;
		float neighborDist_;
		std::set<std::pair<float, std::pair<int, int> > > neighbors_;
		Vector2 newVelocity_;
		float orientation_;
		Vector2 position_;
		float prefSpeed_;
		Vector2 prefVelocity_;
		float radius_;

#if DIFFERENTIAL_DRIVE
		float rightWheelSpeed_;
#endif

		int subGoalNo_;

#if DIFFERENTIAL_DRIVE
		float timeToOrientation_;
#endif

		float uncertaintyOffset_;
		Vector2 velocity_;

#if DIFFERENTIAL_DRIVE
		float wheelTrack_;
#endif

		static HRVOSimulator* Sim_;


		static const int AGENT_TYPE = 0;
		static const int OBSTACLE_TYPE = 1;

        std::vector<Agent*> agents_;
        std::vector<Obstacle*>  obstacles_;

		friend class HRVOSimulator;
		friend class KdTree;
	};
}



#endif
