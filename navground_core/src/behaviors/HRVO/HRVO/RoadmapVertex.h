/*
 *  RoadmapVertex.h
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

#ifndef ROADMAP_VERTEX_H
#define ROADMAP_VERTEX_H

#include "HRVODef.h"

namespace HRVO
{
	/*!
	 *  @class
	 *  @abstract   Defines a roadmap vertex in the simulation.
	 */
	class RoadmapVertex
	{
	private:
		/*!
		 *  @function
		 *  @abstract   Constructs a roadmap vertex instance at the specified two-dimensional
		 *              position.
		 *  @param      position  The two-dimensional position of this roadmap vertex.
		 */
		explicit RoadmapVertex(const Vector2& position);
		
		/*!
		 *  @function
		 *  @abstract   Destroys this roadmap vertex instance.
		 */
		~RoadmapVertex();
		
		/*!
		 *  @function
		 *  @abstract   Adds a neighbor to the list of neighbors of this roadmap vertex.
		 *  @param      dist  The distance to the neighbor.
		 *  @param      neighbor  The number of the neighbor.
		 */
		void addNeighbor(float dist, int neighborNo);
		
		/*!
		 *  @function
		 *  @abstract   Computes the neighbors of this roadmap vertex.
		 *  @param      radius  The radius within which to search for neighbors.
		 */
		void computeNeighbors(float radius);
		
		std::vector<std::pair<float, int> > neighbors_;
		Vector2 position_;
		
		static HRVOSimulator* Sim_;
		
		friend class Agent;
		friend class Goal;
		friend class HRVOSimulator;
	};
}

#endif
