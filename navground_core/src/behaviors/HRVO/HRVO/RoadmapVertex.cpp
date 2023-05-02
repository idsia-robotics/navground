/*
 *  RoadmapVertex.cpp
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

#include "RoadmapVertex.h"

#include <limits>

#include "HRVOSimulator.h"
#include "KdTree.h"
#include "Obstacle.h"

namespace HRVO
{
	HRVOSimulator*  RoadmapVertex::Sim_ = HRVOSimulator::Instance();
	
	RoadmapVertex::RoadmapVertex(const Vector2& position)
	: neighbors_(),
	  position_(position)
	{
	}
	
	RoadmapVertex::~RoadmapVertex()
	{
	}
	
	void RoadmapVertex::addNeighbor(float dist, int neighborNo)
	{
		neighbors_.push_back(std::make_pair(dist, neighborNo));
	}
	
	void RoadmapVertex::computeNeighbors(float radius)
	{
		neighbors_.clear();
		
		for (int i = 0; i < static_cast<int>(Sim_->roadmapVertices_.size()); ++i)
		{
			if (Sim_->roadmapVertices_[i] != this
				&& Sim_->kdTree_->queryVisibility(position_, 
												  Sim_->roadmapVertices_[i]->position_, radius))
			{
				addNeighbor(abs(Sim_->roadmapVertices_[i]->position_ - position_), i);
			}
		}
	}
}
