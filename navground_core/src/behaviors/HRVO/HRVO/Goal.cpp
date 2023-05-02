/*
 *  Goal.cpp
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

#include "Goal.h"

#include <limits>

#include "HRVOSimulator.h"
#include "RoadmapVertex.h"

namespace HRVO
{
	HRVOSimulator* Goal::Sim_ = HRVOSimulator::Instance();
	
	Goal::Goal(const Vector2& position)
	: dist_(),
	  roadmapVertex_(new RoadmapVertex(position))
	{
	}
	
	Goal::~Goal()
	{
		delete roadmapVertex_;
	}
	
	void Goal::computeShortestPathTree()
	{
		roadmapVertex_->computeNeighbors(Sim_->automaticRadius_);
		
		std::multimap<float, int> Q;
		dist_.assign(Sim_->roadmapVertices_.size(),
					 std::make_pair(std::numeric_limits<float>::infinity(), -1));
		std::vector<std::multimap<float, int>::iterator> position_in_Q(Sim_->roadmapVertices_.size(),
																	   Q.end());
		
		for (int j = 0; j < static_cast<int>(roadmapVertex_->neighbors_.size()); ++j)
		{
			int u = roadmapVertex_->neighbors_[j].second;
			float distance = roadmapVertex_->neighbors_[j].first;
			dist_[u] = std::make_pair(distance, -1);
			position_in_Q[u] = Q.insert(std::make_pair(distance, u));
		}
		
		int u, v;
		
		while (!Q.empty())
		{
			u = Q.begin()->second;
			Q.erase(Q.begin());
			position_in_Q[u] = Q.end();
			
			for (int j = 0; j < static_cast<int>(Sim_->roadmapVertices_[u]->neighbors_.size()); ++j)
			{
				v = Sim_->roadmapVertices_[u]->neighbors_[j].second;
				float dist_uv = Sim_->roadmapVertices_[u]->neighbors_[j].first;
				
				if (dist_[v].first > dist_[u].first + dist_uv)
				{
					dist_[v] = std::make_pair(dist_[u].first + dist_uv, u);
					
					if (position_in_Q[v] == Q.end())
					{
						position_in_Q[v] = Q.insert(std::make_pair(dist_[v].first, v));
					}
					else
					{
						Q.erase(position_in_Q[v]);
						position_in_Q[v] = Q.insert(std::make_pair(dist_[v].first, v));
					}
				}
			}
		}
	}
}
