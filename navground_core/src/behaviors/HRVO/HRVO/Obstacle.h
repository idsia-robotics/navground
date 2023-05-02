/*
 *  Obstacle.h
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

#ifndef OBSTACLE_HRVO_H
#define OBSTACLE_HRVO_H

#include "HRVODef.h"

namespace HRVO
{
	/*!
	 *  @class
	 *  @abstract   Defines statics obstacles in the simulation.
	 */
	class Obstacle
	{
	public:
		/*!
		 *  @function
		 *  @abstract   Constructs a static obstacle instance with endpoints at the specified 
		 *              two-dimensional positions.
		 *  @param      point1  The two-dimensional position of the first endpoint of this static
		 *                      obstacle.
		 *  @param      point2  The two-dimensional position of the second endpoint of this static
		 *                      obstacle.
		 */
		Obstacle(const Vector2& point1, const Vector2& point2);
		
		/*!
		 *  @function
		 *  @abstract   Destroys this static obstacle instance.
		 */
		~Obstacle();

		Vector2 normal_;
		Vector2 point1_;
		Vector2 point2_;
		
		friend class Agent;
		friend class HRVOSimulator;
		friend class KdTree;
		friend class RoadmapVertex;
	};
}

#endif

