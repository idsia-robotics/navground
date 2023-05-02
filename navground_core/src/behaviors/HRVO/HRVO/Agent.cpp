/*
 *  Agent.cpp
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

#include "Agent.h"

#include "Goal.h"
#include "KdTree.h"
#include "Obstacle.h"
#include "RoadmapVertex.h"



namespace HRVO
{
	HRVOSimulator* Agent::Sim_ = HRVOSimulator::Instance();

	const int Agent::AGENT_TYPE;
	const int Agent::OBSTACLE_TYPE;

	Agent::Agent()
	: isAtGoal_(false),
	  classNo_(0),
	  goalNo_(-1),
	  goalRadius_(0.0f),
	  isColliding_(false),
#if DIFFERENTIAL_DRIVE
	  leftWheelSpeed_(0.0f),
#endif
	  maxAccel_(0.0f),
	  maxNeighbors_(0),
	  maxSpeed_(0.0f),
	  neighborDist_(0.0f),
	  neighbors_(),
	  newVelocity_(),
	  orientation_(0.0f),
	  position_(),
	  prefSpeed_(0.0f),
	  prefVelocity_(),
	  radius_(0.0f),
#if DIFFERENTIAL_DRIVE
	  rightWheelSpeed_(0.0f),
#endif
	  subGoalNo_(-2),
#if DIFFERENTIAL_DRIVE
	  timeToOrientation_(0.0f),
#endif
	  uncertaintyOffset_(0.0f),
	  velocity_()
#if DIFFERENTIAL_DRIVE
	  , wheelTrack_(0.0f)
#endif
	{
	}

	Agent::Agent(const Vector2& position, int goalNo)
	: classNo_(Sim_->defaultAgent_->classNo_),
	  goalNo_(goalNo),
	  goalRadius_(Sim_->defaultAgent_->goalRadius_),
	  isAtGoal_(false),
	  isColliding_(false),
#if DIFFERENTIAL_DRIVE
	  leftWheelSpeed_(0.0f),
#endif
	  maxAccel_(Sim_->defaultAgent_->maxAccel_),
	  maxNeighbors_(Sim_->defaultAgent_->maxNeighbors_),
	  maxSpeed_(Sim_->defaultAgent_->maxSpeed_),
	  neighborDist_(Sim_->defaultAgent_->neighborDist_),
	  neighbors_(),
	  newVelocity_(Sim_->defaultAgent_->velocity_),
	  orientation_(Sim_->defaultAgent_->orientation_),
	  position_(position),
	  prefSpeed_(Sim_->defaultAgent_->prefSpeed_),
	  prefVelocity_(),
	  radius_(Sim_->defaultAgent_->radius_),
#if DIFFERENTIAL_DRIVE
	  rightWheelSpeed_(0.0f),
#endif
	  subGoalNo_(-2),
#if DIFFERENTIAL_DRIVE
	  timeToOrientation_(Sim_->defaultAgent_->timeToOrientation_),
#endif
	  uncertaintyOffset_(Sim_->defaultAgent_->uncertaintyOffset_),
	  velocity_(Sim_->defaultAgent_->velocity_)
#if DIFFERENTIAL_DRIVE
	  , wheelTrack_(Sim_->defaultAgent_->wheelTrack_)
#endif
	{
#if DIFFERENTIAL_DRIVE
		computeWheelSpeeds();
#endif
	}

	Agent::Agent(const Vector2& position, int goalNo, float neighborDist, int maxNeighbors, int classNo, float radius, const Vector2& velocity, float maxAccel, float goalRadius, float prefSpeed, float maxSpeed, float orientation,
#if DIFFERENTIAL_DRIVE
				 float timeToOrientation, float wheelTrack,
#endif
				 float uncertaintyOffset)
	: classNo_(classNo),
	  goalNo_(goalNo),
	  goalRadius_(goalRadius),
	  isAtGoal_(false),
	  isColliding_(false),
#if DIFFERENTIAL_DRIVE
	  leftWheelSpeed_(0.0f),
#endif
	  maxAccel_(maxAccel),
	  maxNeighbors_(maxNeighbors),
	  maxSpeed_(maxSpeed),
	  neighborDist_(neighborDist),
	  neighbors_(),
	  newVelocity_(velocity),
	  orientation_(orientation),
	  position_(position),
	  prefSpeed_(prefSpeed),
	  prefVelocity_(),
	  radius_(radius),
#if DIFFERENTIAL_DRIVE
	  rightWheelSpeed_(0.0f),
#endif
	  subGoalNo_(-2),
#if DIFFERENTIAL_DRIVE
	  timeToOrientation_(timeToOrientation),
#endif
	  uncertaintyOffset_(uncertaintyOffset),
	  velocity_(velocity)
#if DIFFERENTIAL_DRIVE
	  , wheelTrack_(wheelTrack)
#endif
	{
#if DIFFERENTIAL_DRIVE
		computeWheelSpeeds();
#endif
	}

	Agent::~Agent()
	{
	}

	void Agent::computeNeighbors()
	{
		isColliding_ = false;
		neighbors_.clear();

		float rangeSq = std::min(sqr(neighborDist_), sqr(std::max(Sim_->timeStep_, maxSpeed_ / maxAccel_) * maxSpeed_ + radius_));
		Sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);

		if (isColliding_)
		{
			return;
		}

		if (static_cast<int>(neighbors_.size()) != maxNeighbors_)
		{
			rangeSq = sqr(neighborDist_);
		}

		Sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
	}

	void Agent::computeNewVelocity()
	{
		std::vector<VelocityObstacle> velocityObstacles;

		VelocityObstacle velocityObstacle;

		for (std::set<std::pair<float, std::pair<int, int> > >::iterator j = neighbors_.begin(); j != neighbors_.end(); ++j)
		{
			int entityType = j->second.first;
			int entityNo = j->second.second;

			if (entityType == AGENT_TYPE)
			{
				Agent* other = this->agents_[entityNo];

				if (absSq(other->position_ - position_) > sqr(other->radius_ + radius_))
				{
					float angle = atan(other->position_ - position_);
					float openingAngle = std::asin((other->radius_ + radius_) / abs(other->position_ - position_));

					velocityObstacle.side1 = Vector2(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
					velocityObstacle.side2 = Vector2(std::cos(angle + openingAngle), std::sin(angle + openingAngle));

					if (leftOf(Vector2(0.0f, 0.0f), other->position_ - position_, prefVelocity_ - other->prefVelocity_) > 0.0f)
					{
						float d, s;

						d = 2.0f * std::sin(openingAngle) * std::cos(openingAngle);
						s = det((velocity_ - other->velocity_) / 2.0f, velocityObstacle.side2) / d;
						velocityObstacle.apex = other->velocity_ + s * velocityObstacle.side1 - (uncertaintyOffset_ * abs(other->position_ - position_) / (other->radius_ + radius_)) * norm(other->position_ - position_);
					}
					else
					{
						float d, s;

						d = 2.0f * std::sin(openingAngle) * std::cos(openingAngle);
						s = det((velocity_ - other->velocity_) / 2.0f, velocityObstacle.side1) / d;
						velocityObstacle.apex = other->velocity_ + s * velocityObstacle.side2 - (uncertaintyOffset_ * abs(other->position_ - position_) / (other->radius_ + radius_)) * norm(other->position_ - position_);
					}

					velocityObstacles.push_back(velocityObstacle);
				}
				else
				{

					velocityObstacle.apex = (other->velocity_ + velocity_) / 2.0f - (uncertaintyOffset_ + (other->radius_ + radius_ - abs(other->position_ - position_)) / (2.0f * Sim_->timeStep_)) * norm(other->position_ - position_);
					velocityObstacle.side1 = normalN(position_, other->position_);
					velocityObstacle.side2 = -velocityObstacle.side1;
					velocityObstacles.push_back(velocityObstacle);
				}
			}
			else if (entityType == OBSTACLE_TYPE)
			{
				// TODO
			}
		}
		std::multimap<float, SamplePoint> potentialSamplePoints;

		SamplePoint samplePoint;

		samplePoint.velocityObstacleNo1 = -1;
		samplePoint.velocityObstacleNo2 = -1;

		if (absSq(prefVelocity_) < sqr(maxSpeed_))
		{
			samplePoint.position = prefVelocity_;
		}
		else
		{
			samplePoint.position = maxSpeed_ * norm(prefVelocity_);
		}

		potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));

		for (int i = 0; i < static_cast<int>(velocityObstacles.size()); ++i)
		{
			samplePoint.velocityObstacleNo1 = i;
			samplePoint.velocityObstacleNo2 = i;

			float dotProduct1 = (prefVelocity_ - velocityObstacles[i].apex) * velocityObstacles[i].side1;
			float dotProduct2 = (prefVelocity_ - velocityObstacles[i].apex) * velocityObstacles[i].side2;

			if (dotProduct1 > 0.0f && leftOf(Vector2(0.0f, 0.0f), velocityObstacles[i].side1, prefVelocity_ - velocityObstacles[i].apex) > 0.0f)
			{
				samplePoint.position = velocityObstacles[i].apex + dotProduct1 * velocityObstacles[i].side1;

				if (absSq(samplePoint.position) < sqr(maxSpeed_))
				{
					potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
				}
			}

			if (dotProduct2 > 0.0f && leftOf(Vector2(0.0f, 0.0f), velocityObstacles[i].side2, prefVelocity_ - velocityObstacles[i].apex) < 0.0f)
			{
				samplePoint.position = velocityObstacles[i].apex + dotProduct2 * velocityObstacles[i].side2;

				if (absSq(samplePoint.position) < sqr(maxSpeed_))
				{
					potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
				}
			}
		}

		for (int i = -1; i < static_cast<int>(velocityObstacles.size()) - 1; ++i)
		{
			for (int j = i + 1; j < static_cast<int>(velocityObstacles.size()); ++j)
			{
				samplePoint.velocityObstacleNo1 = i;
				samplePoint.velocityObstacleNo2 = j;

				if (i == -1)
				{
					float discriminant = sqr(maxSpeed_) - sqr(det(velocityObstacles[j].apex, velocityObstacles[j].side1));

					if (discriminant > 0.0f)
					{

						float t1 = -(velocityObstacles[j].apex * velocityObstacles[j].side1) + std::sqrt(discriminant);
						float t2 = -(velocityObstacles[j].apex * velocityObstacles[j].side1) - std::sqrt(discriminant);

						if (t1 >= 0.0f)
						{
							samplePoint.position = velocityObstacles[j].apex + t1 * velocityObstacles[j].side1;
							potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
						}

						if (t2 >= 0.0f)
						{
							samplePoint.position = velocityObstacles[j].apex + t2 * velocityObstacles[j].side1;
							potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
						}
					}

					discriminant = sqr(maxSpeed_) - sqr(det(velocityObstacles[j].apex, velocityObstacles[j].side2));

					if (discriminant > 0.0f)
					{
						float t1 = -(velocityObstacles[j].apex * velocityObstacles[j].side2) + std::sqrt(discriminant);
						float t2 = -(velocityObstacles[j].apex * velocityObstacles[j].side2) - std::sqrt(discriminant);

						if (t1 >= 0.0f)
						{
							samplePoint.position = velocityObstacles[j].apex + t1 * velocityObstacles[j].side2;
							potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
						}

						if (t2 >= 0.0f)
						{
							samplePoint.position = velocityObstacles[j].apex + t2 * velocityObstacles[j].side2;
							potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
						}
					}

				}
				else
				{
					float d, s, t;

					d = det(velocityObstacles[i].side1, velocityObstacles[j].side1);

					if (d != 0.0f)
					{
						s = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[j].side1) / d;
						t = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[i].side1) / d;

						if (s >= 0.0f && t >= 0.0f)
						{
							samplePoint.position = velocityObstacles[i].apex + s * velocityObstacles[i].side1;

							if (absSq(samplePoint.position) < sqr(maxSpeed_))
							{
								potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
							}
						}
					}

					d = det(velocityObstacles[i].side2, velocityObstacles[j].side1);

					if (d != 0.0f)
					{
						s = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[j].side1) / d;
						t = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[i].side2) / d;

						if (s >= 0.0f && t >= 0.0f)
						{
							samplePoint.position = velocityObstacles[i].apex + s * velocityObstacles[i].side2;

							if (absSq(samplePoint.position) < sqr(maxSpeed_))
							{
								potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
							}
						}
					}

					d = det(velocityObstacles[i].side1, velocityObstacles[j].side2);

					if (d != 0.0f)
					{
						s = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[j].side2) / d;
						t = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[i].side1) / d;

						if (s >= 0.0f && t >= 0.0f)
						{
							samplePoint.position = velocityObstacles[i].apex + s * velocityObstacles[i].side1;

							if (absSq(samplePoint.position) < sqr(maxSpeed_))
							{
								potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
							}
						}
					}

					d = det(velocityObstacles[i].side2, velocityObstacles[j].side2);

					if (d != 0.0f)
					{
						s = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[j].side2) / d;
						t = det(velocityObstacles[j].apex - velocityObstacles[i].apex, velocityObstacles[i].side2) / d;

						if (s >= 0.0f && t >= 0.0f)
						{
							samplePoint.position = velocityObstacles[i].apex + s * velocityObstacles[i].side2;

							if (absSq(samplePoint.position) < sqr(maxSpeed_))
							{
								potentialSamplePoints.insert(std::make_pair(absSq(prefVelocity_ - samplePoint.position), samplePoint));
							}
						}
					}
				}
			}
		}

		bool isValid = false;
		int optimal = -1;

		for (std::multimap<float, SamplePoint>::iterator i = potentialSamplePoints.begin(); i != potentialSamplePoints.end(); ++i)
		{
			samplePoint = i->second;
			isValid = true;

			for (int j = 0; j < static_cast<int>(velocityObstacles.size()); ++j)
			{
				if (j != samplePoint.velocityObstacleNo1 && j != samplePoint.velocityObstacleNo2 && leftOf(Vector2(0.0f, 0.0f), velocityObstacles[j].side2, samplePoint.position - velocityObstacles[j].apex) < 0.0f && leftOf(Vector2(0.0f, 0.0f), velocityObstacles[j].side1, samplePoint.position - velocityObstacles[j].apex) > 0.0f)
				{
					isValid = false;

					if (j > optimal)
					{
						optimal = j;
						newVelocity_ = samplePoint.position;
					}

					break;
				}
			}

			if (isValid)
			{
				newVelocity_ = samplePoint.position;

				break;
			}
		}
	}

	void Agent::computePreferredVelocity()
	{
		Goal* goal = Sim_->goals_[goalNo_];

		if (subGoalNo_ == -1)
		{
			if (!Sim_->kdTree_->queryVisibility(goal->roadmapVertex_->position_, position_, 0.0f))
			{
				subGoalNo_ = -2;
			}
		}
		else if (subGoalNo_ >= 0)
		{
			if (Sim_->kdTree_->queryVisibility(Sim_->roadmapVertices_[subGoalNo_]->position_, position_, 0.0f))
			{
				int advancedSubGoal = goal->dist_[subGoalNo_].second;

				if (advancedSubGoal == -1)
				{
					if (Sim_->kdTree_->queryVisibility(goal->roadmapVertex_->position_, position_, radius_))
					{
						subGoalNo_ = -1;
					}
				}
				else if (Sim_->kdTree_->queryVisibility(Sim_->roadmapVertices_[advancedSubGoal]->position_, position_, radius_))
				{
					subGoalNo_ = advancedSubGoal;
				}
			}
			else
			{
				subGoalNo_ = -2;
			}
		}

		if (subGoalNo_ == -2)
		{
			if (Sim_->kdTree_->queryVisibility(goal->roadmapVertex_->position_, position_, radius_))
			{
				subGoalNo_ = -1;
			}
			else
			{
				float minDist = std::numeric_limits<float>::infinity();

				for (int i = 0; i < static_cast<int>(goal->dist_.size()); ++i)
				{
					float dist = goal->dist_[i].first + abs(position_ - Sim_->roadmapVertices_[i]->position_);

					if (dist < minDist && Sim_->kdTree_->queryVisibility(Sim_->roadmapVertices_[i]->position_, position_, radius_))
					{
						minDist = dist;
						subGoalNo_ = i;
					}
				}
			}
		}

		if (subGoalNo_ == -2)
		{
			subGoalNo_ = -1;
		}

		Vector2 subGoalPos;

		if (subGoalNo_ == -1)
		{
			subGoalPos = goal->roadmapVertex_->position_;
		}
		else
		{
			subGoalPos = Sim_->roadmapVertices_[subGoalNo_]->position_;
		}

		float distSqToSubGoal = absSq(subGoalPos - position_);

		if (subGoalNo_ == -1 && sqr(prefSpeed_ * Sim_->timeStep_) > distSqToSubGoal)
		{
			prefVelocity_ = (subGoalPos - position_) / Sim_->timeStep_;
		}
		else
		{
			prefVelocity_ = prefSpeed_ * (subGoalPos - position_) / std::sqrt(distSqToSubGoal);
		}
	}

#if DIFFERENTIAL_DRIVE
	void Agent::computeWheelSpeeds()
	{
		float orientationDiff, speedDiff, targetOrientation, targetSpeed;

		if (isAtGoal_)
		{
			targetOrientation = orientation_;
		}
		else
		{
			targetOrientation = atan(newVelocity_);
		}

		orientationDiff = std::fmod(targetOrientation - orientation_, 2.0f * static_cast<float>(M_PI));

		if (orientationDiff < -static_cast<float>(M_PI))
		{
			orientationDiff += 2.0f * static_cast<float>(M_PI);
		}

		if (orientationDiff > static_cast<float>(M_PI))
		{
			orientationDiff -= 2.0f * static_cast<float>(M_PI);
		}

		speedDiff = (orientationDiff * wheelTrack_) / timeToOrientation_;

		if (speedDiff > 2.0f * maxSpeed_)
		{
			speedDiff = 2.0f * maxSpeed_;
		}
		else if (speedDiff < -2.0f * maxSpeed_)
		{
			speedDiff = -2.0f * maxSpeed_;
		}

		targetSpeed = abs(newVelocity_);

		if (targetSpeed + std::fabs(speedDiff) / 2.0f > maxSpeed_)
		{
			if (speedDiff >= 0.0f)
			{
				rightWheelSpeed_ = maxSpeed_;
				leftWheelSpeed_ = maxSpeed_ - speedDiff;
			}
			else
			{
				leftWheelSpeed_ = maxSpeed_;
				rightWheelSpeed_ = maxSpeed_ + speedDiff;
			}
		}
		else if (targetSpeed - std::fabs(speedDiff) / 2.0f < -maxSpeed_)
		{
			if (speedDiff >= 0.0f)
			{
				leftWheelSpeed_ = -maxSpeed_;
				rightWheelSpeed_ = -maxSpeed_ + speedDiff;
			}
			else
			{
				rightWheelSpeed_ = -maxSpeed_;
				leftWheelSpeed_ = -maxSpeed_ - speedDiff;
			}
		}
		else
		{
			rightWheelSpeed_ = targetSpeed + speedDiff / 2.0f;
			leftWheelSpeed_ = targetSpeed - speedDiff / 2.0f;
		}
	}
#endif

	void Agent::insertAgentNeighbor(int agentNo, float& rangeSq)
	{
		Agent* other = this->agents_[agentNo];

		if (this != other)
		{
			float distSq = absSq(position_ - other->position_);

			if (distSq < sqr(radius_ + other->radius_) && distSq < rangeSq)
			{
				if (!isColliding_)
				{
					isColliding_ = true;
					neighbors_.clear();
				}

				if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
				{
					neighbors_.erase(--neighbors_.end());
				}

				neighbors_.insert(std::make_pair(distSq, std::make_pair(AGENT_TYPE, agentNo)));

				if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
				{
					rangeSq = (--neighbors_.end())->first;
				}
			}
			else if (!isColliding_ && distSq < rangeSq)
			{
				if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
				{
					neighbors_.erase(--neighbors_.end());
				}

				neighbors_.insert(std::make_pair(distSq, std::make_pair(AGENT_TYPE, agentNo)));

				if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
				{
					rangeSq = (--neighbors_.end())->first;
				}
			}
		}
	}

	void Agent::insertObstacleNeighbor(int obstacleNo, float& rangeSq)
	{
		Obstacle* obstacle = this->obstacles_[obstacleNo];
		float distSq = distSqPointLineSegment(obstacle->point1_, obstacle->point2_, position_);

		if (distSq < sqr(radius_) && distSq < rangeSq)
		{
			if (!isColliding_)
			{
				isColliding_ = true;
				neighbors_.clear();
				rangeSq = sqr(radius_);
			}

			if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
			{
				neighbors_.erase(--neighbors_.end());
			}

			neighbors_.insert(std::make_pair(distSq, std::make_pair(OBSTACLE_TYPE, obstacleNo)));

			if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
			{
				rangeSq = (--neighbors_.end())->first;
			}
		}
		else if (!isColliding_ && distSq < rangeSq)
		{
			if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
			{
				neighbors_.erase(--neighbors_.end());
			}

			neighbors_.insert(std::make_pair(distSq, std::make_pair(OBSTACLE_TYPE, obstacleNo)));

			if (static_cast<int>(neighbors_.size()) == maxNeighbors_)
			{
				rangeSq = (--neighbors_.end())->first;
			}
		}
	}


	void Agent::update()
	{
#if DIFFERENTIAL_DRIVE
		float averageWheelSpeed = (rightWheelSpeed_ + leftWheelSpeed_) / 2.0f;
		float wheelSpeedDifference = rightWheelSpeed_ - leftWheelSpeed_;

	    position_ += Sim_->timeStep_ * averageWheelSpeed * Vector2(std::cos(orientation_), std::sin(orientation_));
		orientation_ += wheelSpeedDifference * Sim_->timeStep_ / wheelTrack_;
		velocity_ = averageWheelSpeed * Vector2(std::cos(orientation_), std::sin(orientation_));
#else

		float dv = abs(newVelocity_ - velocity_);

		if (dv < maxAccel_ * Sim_->timeStep_)
		{
			velocity_ = newVelocity_;
		}
		else
		{
			velocity_ = (1.0f - (maxAccel_ * Sim_->timeStep_ / dv)) * velocity_ + (maxAccel_ * Sim_->timeStep_ / dv) * newVelocity_;
		}

		position_ += velocity_ * Sim_->timeStep_;
#endif

		if (absSq(Sim_->goals_[goalNo_]->roadmapVertex_->position_ - position_) < sqr(goalRadius_))
		{
			isAtGoal_ = true;
		}
		else
		{
			isAtGoal_ = false;
			Sim_->areAllAgentsAtGoals_ = false;
		}

#if !DIFFERENTIAL_DRIVE
		if (!isAtGoal_)
		{
			orientation_ = atan(prefVelocity_);
		}
#endif
	}
}
