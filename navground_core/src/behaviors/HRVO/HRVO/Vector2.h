/*
 *  Vector2.h
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

#ifndef VECTOR_2_HRVO_H
#define VECTOR_2_HRVO_H



#include <iosfwd>

namespace HRVO
{
	/*!
	 *  @class
	 *  @abstract   Defines a two-dimensional vector.
	 */
	class Vector2
	{
	public:		
		/*!
		 *  @function
		 *  @abstract   Constructs and initializes a two-dimensional vector instance to (0.0, 0.0).
		 */
		Vector2();
		
		/*!
		 *  @function
		 *  @abstract   Constructs and initializes a two-dimensional vector from the specified
		 *              two-dimensional vector.
		 *  @param      vector  The two-dimensional vector containing the xy initialization data.
		 */	
		Vector2(const Vector2& vector);
		
		/*!
		 *  @function
		 *  @abstract   Constructs and initializes a two-dimensional vector from the specified
		 *              xy-coordinates.
		 *  @param      x  The x-coordinate of the two-dimensional vector.
		 *  @param      y  The y-coordinate of the two-dimensional vector.
		 */
		Vector2(float x, float y);
		
		/*!
		 *  @function
		 *  @abstract   Destroys this two-dimensional vector instance.
		 */
		~Vector2();
		
		/*!
		 *  @function
		 *  @abstract   Returns the x-coordinate of this two-dimensional vector.
		 *  @result     The x-coordinate of the two-dimensional vector.
		 */
		float x() const;
		
		/*!
		 *  @function
		 *  @abstract   Returns the y-coordinate of this two-dimensional vector.
		 *  @result     The y-coordinate of the two-dimensional vector.
		 */
		float y() const;
		
		Vector2 operator-() const;
		
		const Vector2& operator+() const;
		
		float operator*(const Vector2& vector) const;
		
		Vector2 operator*(float s) const;
		
		Vector2 operator/(float s) const;
		
		Vector2 operator+(const Vector2& vector) const;
		
		Vector2 operator-(const Vector2& vector) const;
		
		bool operator==(const Vector2& vector) const;
		
		bool operator!=(const Vector2& vector) const;
		
		Vector2& operator*=(float s);
		
		Vector2& operator/=(float s);
		
		Vector2& operator+=(const Vector2& vector);
		
		Vector2& operator-=(const Vector2& vector);
		
	private:
		float x_;
		
		float y_;
	};	
}

HRVO::Vector2 operator*(float s, const HRVO::Vector2& vector);

std::ostream& operator<<(std::ostream& os, const HRVO::Vector2& vector);

/*!
 *  @function
 *  @abstract   Computes the length of a specified two-dimensional vector.
 *  @param      vector  The two-dimensional vector whose length is to be computed.
 *  @result     The length of the two-dimensional vector.
 */
float abs(const HRVO::Vector2& vector);

/*!
 *  @function
 *  @abstract   Computes the squared length of a specified two-dimensional vector.
 *  @param      vector  The two-dimensional vector whose squared length is to be calculated.
 *  @result     The squared length of the two-dimensional vector.
 */
float absSq(const HRVO::Vector2& vector);

/*!
 *  @function
 *  @abstract   Computes the angle between a specified vector and the positive x-axis.
 *  @param      vector  The two-dimensional vector whose angle with the positive x-axis is to be
 *                      calculated.
 *  @result     The angle (in radians) between the two-dimensional vector and the positive x-axis in
 *              the range [-PI, PI].
 */
float atan(const HRVO::Vector2& vector);

/*!
 *  @function
 *  @abstract   Computes the determinant of a two-dimensional square matrix with rows consisting of
 *              the specified two-dimensional vectors.
 *  @param      vector1  The top row of the two-dimensional square matrix.
 *  @param      vector2  The bottom row of the two-dimensional square matrix.
 *  @result     The determinant of the two-dimensional square matrix.
 */
float det(const HRVO::Vector2& vector1, const HRVO::Vector2& vector2);

/*!
 *  @function
 *  @abstract   Computes the normalization of a specified two-dimensional vector.
 *  @param      vector  The two-dimensional vector whose normalization is to be calculated.
 *  @result     The normalization of the two-dimensional vector.
 */
HRVO::Vector2 norm(const HRVO::Vector2& vector);

/*!
 *  @function
 *  @abstract   Computes the normal to a line segment with the specified endpoints.
 *  @param      vector1  The first endpoint of the line segment.
 *  @param      vector2  The second endpoint of the line segment.
 *  @result     The two-dimensional normal vector of the line segment.
 */
HRVO::Vector2 normalN(const HRVO::Vector2& vector1, const HRVO::Vector2& vector2);

#endif
