/*
 *  Vector2.cpp
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

#include "Vector2.h"

#include <cmath>
#include <ostream>

namespace HRVO
{
	Vector2::Vector2() 
	: x_(0.0f),
	  y_(0.0f)
	{
	}
	
	Vector2::Vector2(const Vector2& q)
	: x_(q.x()),
	  y_(q.y())
	{
	}
	
	Vector2::Vector2(float x, float y)
	: x_(x),
	  y_(y)
	{
	}
	
	Vector2::~Vector2() 
	{
	}
	
	float Vector2::x() const
	{
		return x_;
	}
	
	float Vector2::y() const
	{
		return y_;
	}
	
	Vector2 Vector2::operator-() const
	{
		return Vector2(-x_, -y_);
	}
	
	const Vector2& Vector2::operator+() const
	{
		return *this;
	}
	
	float Vector2::operator*(const Vector2& q) const
	{
		return x_ * q.x() + y_ * q.y();
	}
	
	Vector2 Vector2::operator*(float a) const
	{
		return Vector2(x_ * a, y_ * a);
	}
	
	Vector2 Vector2::operator/(float a) const
	{
		float inv_a = 1.0f / a;
		
		return Vector2(x_ * inv_a, y_ * inv_a);
	}
	
	Vector2 Vector2::operator+(const Vector2& q) const
	{
		return Vector2(x_ + q.x(), y_ + q.y());
	}
	
	Vector2 Vector2::operator-(const Vector2& q) const
	{
		return Vector2(x_ - q.x(), y_ - q.y());
	}
	
	bool Vector2::operator==(const Vector2& q) const
	{
		return x_ == q.x() && y_ == q.y();
	}
	
	bool Vector2::operator!=(const Vector2& q) const
	{
		return x_ != q.x() || y_ != q.y();
	}
	
	Vector2& Vector2::operator*=(float a)
	{
		x_ *= a;
		y_ *= a;
		
		return *this;
	}
	
	Vector2& Vector2::operator/=(float a)
	{
		float inv_a = 1.0f / a;
		x_ *= inv_a;
		y_ *= inv_a;
		
		return *this;
	}
	
	Vector2& Vector2::operator+=(const Vector2& q)
	{
		x_ += q.x();
		y_ += q.y();
		
		return *this;
	}
	
	Vector2& Vector2::operator-=(const Vector2& q)
	{
		x_ -= q.x();
		y_ -= q.y();
		
		return *this;
	}
}

HRVO::Vector2 operator*(float a, const HRVO::Vector2& q)
{
	return HRVO::Vector2(a * q.x(), a * q.y());
}

std::ostream& operator<<(std::ostream& os, const HRVO::Vector2& q)
{
	os << q.x() << " " << q.y();
	
	return os;
}

float abs(const HRVO::Vector2& q)
{
	return std::sqrt(q * q);
}

float absSq(const HRVO::Vector2& q)
{
	return q * q;
}

float atan(const HRVO::Vector2& q)
{
	return std::atan2(q.y(), q.x());
}

float det(const HRVO::Vector2& p, const HRVO::Vector2& q)
{
	return p.x() * q.y() - p.y() * q.x();
}

HRVO::Vector2 norm(const HRVO::Vector2& q)
{
	return q / abs(q);
}

HRVO::Vector2 normalN(const HRVO::Vector2& p, const HRVO::Vector2& q)
{
	return norm(HRVO::Vector2(q.y() - p.y(), -(q.x() - p.x())));
}
