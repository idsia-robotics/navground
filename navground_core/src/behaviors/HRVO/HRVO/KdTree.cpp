/*
 *  KdTree.cpp
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

#include "KdTree.h"

#include <limits>

#include "Agent.h"
#include "HRVOSimulator.h"
#include "Obstacle.h"

namespace HRVO
{
	HRVOSimulator*  KdTree::Sim_ = HRVOSimulator::Instance();
	
	KdTree::KdTree()
	: agentNos_(),
	  agentTree_(),
	  obstacleTree_(0)
	{
		for (int i = 0; i < static_cast<int>(Sim_->agents_.size()); ++i)
		{
			agentNos_.push_back(i);
		}
		
		agentTree_.resize(2 * Sim_->agents_.size()-1);
	}
	
	KdTree::~KdTree()
	{
		if (obstacleTree_ != 0)
		{
			deleteObstacleTree(obstacleTree_);
		}
	}
	
	void KdTree::buildAgentTree()
	{
		if (!agentNos_.empty())
		{
			buildAgentTreeRecursive(0, static_cast<int>(agentNos_.size()), 0);
		}
	}
	
	void KdTree::buildAgentTreeRecursive(int begin, int end, int node)
	{
		agentTree_[node].begin = begin;
		agentTree_[node].end = end;
		
		agentTree_[node].minX = agentTree_[node].maxX = Sim_->agents_[agentNos_[begin]]->position_.x();
		agentTree_[node].minY = agentTree_[node].maxY = Sim_->agents_[agentNos_[begin]]->position_.y();
		
		for (int i = begin + 1; i < end; ++i)
		{
			if (Sim_->agents_[agentNos_[i]]->position_.x() > agentTree_[node].maxX)
			{
				agentTree_[node].maxX = Sim_->agents_[agentNos_[i]]->position_.x();
			}
			else if (Sim_->agents_[agentNos_[i]]->position_.x() < agentTree_[node].minX)
			{
				agentTree_[node].minX = Sim_->agents_[agentNos_[i]]->position_.x();
			}
			
			if (Sim_->agents_[agentNos_[i]]->position_.y() > agentTree_[node].maxY)
			{
				agentTree_[node].maxY = Sim_->agents_[agentNos_[i]]->position_.y();
			}
			else if (Sim_->agents_[agentNos_[i]]->position_.y() < agentTree_[node].minY)
			{
				agentTree_[node].minY = Sim_->agents_[agentNos_[i]]->position_.y();
			}
		}
		
		if (end - begin > MAX_LEAF_SIZE)
		{
			bool isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY 
							   - agentTree_[node].minY);
			float splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) 
								: 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));
			
			int left = begin;
			int right = end - 1;
			
			while (true)
			{
				while (left <= right && (isVertical ? Sim_->agents_[agentNos_[left]]->position_.x() 
										 : Sim_->agents_[agentNos_[left]]->position_.y()) < splitValue)
				{
					++left;
				}
				
				while (right >= left && (isVertical ? Sim_->agents_[agentNos_[right]]->position_.x() 
										 : Sim_->agents_[agentNos_[right]]->position_.y()) >= splitValue)
				{
					--right;
				}
				
				if (left > right)
				{
					break;
				}
				else
				{
					std::swap(agentNos_[left], agentNos_[right]);
					++left;
					--right;
				}
			}
			
			int leftSize = left - begin;
			
			if (leftSize == 0)
			{
				++leftSize;
				++left;
				++right;
			}
			
			agentTree_[node].left = node + 1;
			agentTree_[node].right = node + 1 + (2 * leftSize - 1);
			
			buildAgentTreeRecursive(begin, left, agentTree_[node].left);
			buildAgentTreeRecursive(left, end, agentTree_[node].right);
		}
	}
	
	void KdTree::buildObstacleTree()
	{
		if (obstacleTree_ != 0)
		{
			deleteObstacleTree(obstacleTree_);
		}
		
		std::vector<int> obstacles(Sim_->obstacles_.size());
		
		for (int i = 0; i < static_cast<int>(Sim_->obstacles_.size()); ++i)
		{
			obstacles[i] = i;
		}
		
		obstacleTree_ = buildObstacleTreeRecursive(obstacles);
	}
	
	
	KdTree::ObstacleTreeNode* KdTree::buildObstacleTreeRecursive(const std::vector<int>& obstacleNos)
	{
		ObstacleTreeNode* node = new ObstacleTreeNode;
		
		if (obstacleNos.empty())
		{
			node->obstacleNo = -1;
			return node;
		}
		else
		{
			int optimalSplit = 0;
			int minLeft = static_cast<int>(obstacleNos.size());
			int minRight = static_cast<int>(obstacleNos.size());
			
			for (int i = 0; i < static_cast<int>(obstacleNos.size()); ++i)
			{
				int leftSize = 0;
				int rightSize = 0;
				
				for (int j = 0; j < static_cast<int>(obstacleNos.size()); ++j)
				{
					if (i != j)
					{
						float j1LeftOfI = leftOf(Sim_->obstacles_[obstacleNos[i]]->point1_, 
												 Sim_->obstacles_[obstacleNos[i]]->point2_,
												 Sim_->obstacles_[obstacleNos[j]]->point1_);
						float j2LeftOfI = leftOf(Sim_->obstacles_[obstacleNos[i]]->point1_,
												 Sim_->obstacles_[obstacleNos[i]]->point2_,
												 Sim_->obstacles_[obstacleNos[j]]->point2_);
						
						if (j1LeftOfI >= 0.0f && j2LeftOfI >= 0.0f)
						{
							++leftSize;
						}
						else if (j1LeftOfI <= 0.0f && j2LeftOfI <= 0.0f)
						{
							++rightSize;
						}
						else
						{
							++leftSize;
							++rightSize;
						}
						
						if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) 
							>= std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight)))
						{
							break;
						}
					}
				}
				
				if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) 
					< std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight)))
				{
					minLeft = leftSize;
					minRight = rightSize;
					optimalSplit = i;
				}
			}
			
			std::vector<int> leftObstacles(minLeft);
			std::vector<int> rightObstacles(minRight);
			
			int leftCounter = 0;
			int rightCounter = 0;
			int i = optimalSplit;
			
			for (int j = 0; j < static_cast<int>(obstacleNos.size()); ++j)
			{
				if (i != j)
				{
					float j1LeftOfI = leftOf(Sim_->obstacles_[obstacleNos[i]]->point1_,
											 Sim_->obstacles_[obstacleNos[i]]->point2_,
											 Sim_->obstacles_[obstacleNos[j]]->point1_);
					float j2LeftOfI = leftOf(Sim_->obstacles_[obstacleNos[i]]->point1_,
											 Sim_->obstacles_[obstacleNos[i]]->point2_,
											 Sim_->obstacles_[obstacleNos[j]]->point2_);
					
					if (j1LeftOfI >= -EPSILON && j2LeftOfI >= -EPSILON)
					{
						leftObstacles[leftCounter++] = obstacleNos[j];
					}
					else if (j1LeftOfI <= EPSILON && j2LeftOfI <= EPSILON)
					{
						rightObstacles[rightCounter++] = obstacleNos[j];
					}
					else
					{
						float t = det(Sim_->obstacles_[obstacleNos[i]]->point2_ 
									  - Sim_->obstacles_[obstacleNos[i]]->point1_,
									  Sim_->obstacles_[obstacleNos[j]]->point1_ 
									  - Sim_->obstacles_[obstacleNos[i]]->point1_)
						/ det(Sim_->obstacles_[obstacleNos[i]]->point2_ 
							  - Sim_->obstacles_[obstacleNos[i]]->point1_, 
							  Sim_->obstacles_[obstacleNos[j]]->point1_ 
							  - Sim_->obstacles_[obstacleNos[j]]->point2_);
						
						Vector2 splitpoint = Sim_->obstacles_[obstacleNos[j]]->point1_
						+ t * (Sim_->obstacles_[obstacleNos[j]]->point2_ 
							   - Sim_->obstacles_[obstacleNos[j]]->point1_);
						
						Obstacle* newObstacle = new Obstacle(splitpoint, 
															 Sim_->obstacles_[obstacleNos[j]]->point2_);
						Sim_->obstacles_.push_back(newObstacle);
						int newObstacleId = static_cast<int>(Sim_->obstacles_.size()) - 1;
						Sim_->obstacles_[obstacleNos[j]]->point2_ = splitpoint;
						
						if (j1LeftOfI > 0.0f)
						{
							leftObstacles[leftCounter++] = obstacleNos[j];
							rightObstacles[rightCounter++] = newObstacleId;
						}
						else
						{
							rightObstacles[rightCounter++] = obstacleNos[j];
							leftObstacles[leftCounter++] = newObstacleId;
						}
					}
				}
			}
			
			node->obstacleNo = obstacleNos[optimalSplit];
			node->left = buildObstacleTreeRecursive(leftObstacles);
			node->right = buildObstacleTreeRecursive(rightObstacles);
			return node;
		}
	}
	
	void KdTree::computeAgentNeighbors(Agent* agent, float& rangeSq) const
	{
		queryAgentTreeRecursive(agent, rangeSq, 0);
	}
	
	void KdTree::computeObstacleNeighbors(Agent* agent, float& rangeSq) const
	{
		queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
	}
	
	void KdTree::deleteObstacleTree(ObstacleTreeNode* node)
	{
		if (node->obstacleNo == -1)
		{
			delete node;
		}
		else
		{
			deleteObstacleTree(node->left);
			deleteObstacleTree(node->right);
			delete node;
		}
	}
	
	void KdTree::queryAgentTreeRecursive(Agent* agent, float& rangeSq, int node) const
	{
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE)
		{
			for (int i = agentTree_[node].begin; i < agentTree_[node].end; ++i)
			{
				agent->insertAgentNeighbor(agentNos_[i], rangeSq);
			}
		}
		else
		{
			float distSqLeft = 0.0f;
			float distSqRight = 0.0f;
			
			if (agent->position_.x() < agentTree_[agentTree_[node].left].minX)
			{
				distSqLeft += sqr(agentTree_[agentTree_[node].left].minX - agent->position_.x());
			}
			else if (agent->position_.x() > agentTree_[agentTree_[node].left].maxX)
			{
				distSqLeft += sqr(agent->position_.x() - agentTree_[agentTree_[node].left].maxX);
			}
			
			if (agent->position_.y() < agentTree_[agentTree_[node].left].minY)
			{
				distSqLeft += sqr(agentTree_[agentTree_[node].left].minY - agent->position_.y());
			}
			else if (agent->position_.y() > agentTree_[agentTree_[node].left].maxY)
			{
				distSqLeft += sqr(agent->position_.y() - agentTree_[agentTree_[node].left].maxY);
			}
			
			if (agent->position_.x() < agentTree_[agentTree_[node].right].minX)
			{
				distSqRight += sqr(agentTree_[agentTree_[node].right].minX - agent->position_.x());
			}
			else if (agent->position_.x() > agentTree_[agentTree_[node].right].maxX)
			{
				distSqRight += sqr(agent->position_.x() - agentTree_[agentTree_[node].right].maxX);
			}
			
			if (agent->position_.y() < agentTree_[agentTree_[node].right].minY)
			{
				distSqRight += sqr(agentTree_[agentTree_[node].right].minY - agent->position_.y());
			}
			else if (agent->position_.y() > agentTree_[agentTree_[node].right].maxY)
			{
				distSqRight += sqr(agent->position_.y() - agentTree_[agentTree_[node].right].maxY);
			}
			
			if (distSqLeft < distSqRight)
			{
				if (distSqLeft < rangeSq)
				{
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
					
					if (distSqRight < rangeSq)
					{
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					}
				}
			}
			else
			{
				if (distSqRight < rangeSq)
				{
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					
					if (distSqLeft < rangeSq)
					{
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
					}
				}
			}
			
		}
	}
	
	void KdTree::queryObstacleTreeRecursive(Agent* agent, float& rangeSq,
											ObstacleTreeNode* node) const
	{
		if (node->obstacleNo == -1)
		{
			return;
		}
		else
		{
			Obstacle* obstacle = Sim_->obstacles_[node->obstacleNo];
			float agentLeftOfLine = leftOf(obstacle->point1_, obstacle->point2_, agent->position_);
			
			queryObstacleTreeRecursive(agent, rangeSq,
									   (agentLeftOfLine >= 0.0f ? node->left : node->right));
			
			float distSqLine = sqr(agentLeftOfLine) / absSq(obstacle->point2_ - obstacle->point1_);
			
			if (distSqLine < rangeSq)
			{
				agent->insertObstacleNeighbor(node->obstacleNo, rangeSq);
				
				if (distSqLine < rangeSq)
				{
					queryObstacleTreeRecursive(agent, rangeSq,
											   (agentLeftOfLine >= 0.0f ? node->right : node->left));
				}
			}
		}
	}
	
	bool KdTree::queryVisibility(const Vector2& q1, const Vector2& q2, float radius) const
	{
		return queryVisibilityRecursive(q1, q2, radius, obstacleTree_);
	}
	
	bool KdTree::queryVisibilityRecursive(const Vector2& q1, const Vector2& q2, float radius,
										  ObstacleTreeNode* node) const
	{
		if (node->obstacleNo == -1)
		{
			return true;
		}
		else
		{
			Obstacle* obstacle = Sim_->obstacles_[node->obstacleNo];
			
			float q1LeftOfI = leftOf(obstacle->point1_, obstacle->point2_, q1);
			float q2LeftOfI = leftOf(obstacle->point1_, obstacle->point2_, q2);
			
			if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f)
			{
				return queryVisibilityRecursive(q1, q2, radius, node->left);
			}
			else if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f)
			{
				return queryVisibilityRecursive(q1, q2, radius, node->right);
			}
			else
			{
				float point1LeftOfQ = leftOf(q1, q2, obstacle->point1_);
				float point2LeftOfQ = leftOf(q1, q2, obstacle->point2_);
				float invLengthQ = 1.0f / absSq(q2 - q1);
				
				return (point1LeftOfQ * point2LeftOfQ >= 0.0f
						&& sqr(point1LeftOfQ) * invLengthQ >= sqr(radius)
						&& sqr(point2LeftOfQ) * invLengthQ >= sqr(radius)
						&& queryVisibilityRecursive(q1, q2, radius, node->left)
						&& queryVisibilityRecursive(q1, q2, radius, node->right));
			}
		}
	}
}
