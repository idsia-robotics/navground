/*
 *  HRVODef.h
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

#ifndef HRVO_DEF_H
#define HRVO_DEF_H

#include <algorithm>
#include <cmath>
#include <ctime>
#include <limits>
#include <map>
#include <set>
#include <vector>

#include "Vector2.h"

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif

static const float EPSILON = 0.00001f;

namespace HRVO
{
	class Agent;
	class HRVOSimulator;
	class Obstacle;
	class RoadmapVertex;
	
	/*!
	 *  @function
	 *  @abstract   Computes the squared distance from a line segment with the specified endpoints
	 *              to a specified point.
	 *  @param      a  The first endpoint of the line segment.
	 *  @param      b  The second endpoint of the line segment.
	 *  @param      c  The point to which the squared distance is to be calculated.
	 *  @result     The squared distance from the line segment to the point.
	 */
	inline float distSqPointLineSegment(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		float r = ((c - a) * (b - a)) / absSq(b - a);
		
		if (r < 0.0f)
		{
			return absSq(c - a);
		}
		else if (r > 1.0f)
		{
			return absSq(c - b);
		}
		else
		{
			return absSq(c - (a + r * (b - a)));
		}
	}
	
	/*!
	 *  @function
	 *  @abstract   Computes the signed distance from a line connecting the specified points to a 
	 *              specified point.
	 *  @param      a  The first point on the line.
	 *  @param      b  The second point on the line.
	 *  @param      c  The point to which the signed distance is to be calculated.
	 *  @result     Positive when the point c lies to the left of the point ab.
	 */
	inline float leftOf(const Vector2& a, const Vector2& b, const Vector2& c)
	{
		return det(a - c, b - a);
	}
	
	/*!
	 *  @function
	 *  @abstract   Computes the square of a float.
	 *  @param      a  The float to be squared.
	 *  @result     The square of the float.
	 */
	inline float sqr(float a)
	{
		return a * a;
	}
	
	/*!
	 *  @function
	 *  @abstract   Computes the time to collision of a ray with a line segment.
	 *  @param      p  The starting position of the ray.
	 *  @param      v  The direction of the ray.
	 *  @param      a  The first endpoint of the line segment.
	 *  @param      b  The second endpoint of the line segment.
	 *  @param      isColliding  True if the ray and line segment are colliding; false otherwise.
	 *  @result     The time to/from collision of the ray p+tv to the line segment ab; infinity if 
	 *              the line segment is not hit; negative if the ray and line segment are colliding.
	 */
	inline float timeToCollision(const Vector2& p, const Vector2& v, const Vector2& a,
								 const Vector2& b, bool isColliding)
	{
		float D = det(v, b - a);
		
		if (D == 0.0f)
		{
			if (isColliding)
			{
				return -std::numeric_limits<float>::infinity();
			}
			else
			{
				return std::numeric_limits<float>::infinity();
			}
		}
		
		float invD = 1.0f / D;
		float t = det(a - p, b - a) * invD;
		float s = det(p - a, v) * -invD;
		
		if (t < 0.0f || s < 0.0f || s > 1.0f)
		{
			if (isColliding)
			{
				return -std::numeric_limits<float>::infinity();
			}
			else
			{
				return std::numeric_limits<float>::infinity();
			}
		}
		else
		{
			return t;
		}
	}
	
	/*!
	 *  @function
	 *  @abstract   Computes the time to collision of a ray with a disc.
	 *  @param      p  The starting position of the ray.
	 *  @param      v  The direction of the ray.
	 *  @param      p2  The center of the disc.
	 *  @param      radius  The radius of the disc.
	 *  @param      isColliding  True if the ray and disc are colliding; false otherwise.
	 *  @result     The time to/from collision of the ray p+tv to the disc; infinity if the disc is
	 *              not hit; negative if the ray and disc are colliding.
	 */
	inline float timeToCollision(const Vector2& p, const Vector2& v, const Vector2& p2,
								 float radius, bool isColliding)
	{
		Vector2 ba = p2 - p;
		float diameterSq = radius * radius;
		float time;
		
		float discriminant = -sqr(det(v, ba)) + diameterSq * absSq(v);
		
		if (discriminant > 0.0f)
		{
			if (isColliding)
			{
				time = (v * ba + std::sqrt(discriminant)) / absSq(v);
				
				if (time < 0.0f)
				{
					time = -std::numeric_limits<float>::infinity();
				}
			}
			else
			{
				time = (v * ba - std::sqrt(discriminant)) / absSq(v);
				
				if (time < 0.0f)
				{
					time = std::numeric_limits<float>::infinity();
				}
			}
		}
		else
		{
			if (isColliding)
			{
				time = -std::numeric_limits<float>::infinity();
			}
			else
			{
				time = std::numeric_limits<float>::infinity();
			}
		}
		
		return time;
	}
}

#endif
