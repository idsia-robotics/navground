/*
 *  KdTree.h
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

#ifndef KD_TREE_H
#define KD_TREE_H

#include "HRVODef.h"

namespace HRVO
{
	/*!
	 *  @class
	 *  @abstract   Defines kd-trees for agents and static obstacles in the simulation.
	 */
	class KdTree
	{
	private:
		/*!
		 *  @struct 
		 *  @abstract   Defines an agent kd-tree node.
		 *  @field      begin  The beginning node number.
		 *  @field      end  The ending node number.
		 *  @field      left  The left node number.
		 *  @field      maxX  The maximum x-coordinate.
		 *  @field      maxY  The maximum y-coordinate.
		 *  @field      minX  The minimum x-coordinate.
		 *  @field      minY  The minimum y-coordinate.
		 *  @field      right  The right node number.
		 */
		struct AgentTreeNode
		{
			int begin;
			int end;
			int left;
			float maxX;
			float maxY;			
			float minX;
			float minY;
			int right;
		};
		
		/*!
		 *  @struct 
		 *  @abstract   Defines an obstacle kd-tree node.
		 *  @field      left  The left obstacle tree node.
		 *  @field      obstacleNo  The obstacle number.
		 *  @field      right  The right obstacle tree node.
		 */
		struct ObstacleTreeNode
		{
			ObstacleTreeNode* left;
			int obstacleNo;
			ObstacleTreeNode* right;
		};
		
		/*!
		 *  @function
		 *  @abstract   Constructs a kd-tree instance.
		 */
		KdTree();
		
		/*!
		 *  @function
		 *  @abstract   Destroys this kd-tree instance.
		 */
		~KdTree();
		
		/*!
		 *  @function
		 *  @abstract   Builds an agent kd-tree.
		 */
		void buildAgentTree();
		
		void buildAgentTreeRecursive(int begin, int end, int node);
		
		/*!
		 *  @function
		 *  @abstract   Builds an obstacle kd-tree.
		 */
		void buildObstacleTree();
		
		ObstacleTreeNode* buildObstacleTreeRecursive(const std::vector<int>& obstacleNos);
		
		/*!
		 *  @function
		 *  @abstract   Computes the agent neighbors of the specified agent.
		 *  @param      agent  A pointer to the agent for which agent neighbors are to be computed.
		 *  @param      rangeSq  The squared range around the agent.
		 */
		void computeAgentNeighbors(Agent* agent, float& rangeSq) const;
		
		/*!
		 *  @function
		 *  @abstract   Computes the obstacle neighbors of the specified agent.
		 *  @param      agent  A pointer to the agent for which obstacle neighbors are to be
		 *                     computed.
		 *  @param      rangeSq  The squared range around the agent.
		 */
		void computeObstacleNeighbors(Agent* agent, float& rangeSq) const;
		
		/*!
		 *  @function
		 *  @abstract   Deletes the specified obstacle tree node.
		 *  @param      node  A pointer to the obstacle tree node to be deleted.
		 */
		void deleteObstacleTree(ObstacleTreeNode* node);
		
		void queryAgentTreeRecursive(Agent* agent, float& rangeSq, int node) const;
		
		void queryObstacleTreeRecursive(Agent* agent, float& rangeSq, ObstacleTreeNode* node) const;
		
		/*!
		 *  @function
		 *  @abstract   Queries the visibility between two points within a specified radius.
		 *  @param      q1  The first point between which visibility is to be tested.
		 *  @param      q2  The second point between which visibility is to be tested.
		 *  @param      radius  The radius within which visibility is to be tested.
		 *  @result     True if q1 and q2 are mutually visible within the radius; false otherwise.
		 */
		bool queryVisibility(const Vector2& q1, const Vector2& q2, float radius) const;
		
		bool queryVisibilityRecursive(const Vector2& q1, const Vector2& q2, float radius,
									  ObstacleTreeNode* node) const;
		
		std::vector<int> agentNos_;
		std::vector<AgentTreeNode> agentTree_;
		ObstacleTreeNode* obstacleTree_;
		
		static HRVOSimulator* Sim_;
		
		static const int MAX_LEAF_SIZE = 10;
		
		friend class Agent;
		friend class HRVOSimulator;
		friend class RoadmapVertex;
	};
}

#endif
