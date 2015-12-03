#pragma once
#include "Vec2.h"
#include "Utils.h"   // for PointD
#include <cmath>

struct LineEquation  // m_a * x + m_b * y + m_c = 0;
{
	double m_a;
	double m_b;
	double m_c;

	LineEquation(double a, double b, double c)
		: m_a(a), m_b(b), m_c(c)                {}

	// line from direction vector and point
	static LineEquation fromDirectionVector(const PointD& a, const Vec2<double>& v)
	{
		/* 
		 * Line from vector: 
		 *   have: A{x,y}, v{x, y}
		 *         (x - A.x) / v.x = (y - A.y) / v.y
		 *
		 *   A = vy, B = -vx, C = (vx * Ay - vy * Ax)
		 */
		return LineEquation(v.m_y, -v.m_x, v.m_x * a.y - v.m_y * a.x);
	}

private:
	static double matrixDeterminant(double a, double b, double c, double d) { return a * d - b * c; }

public:

	bool isIntersects(const LineEquation& another, PointD& result) const
	{
		static const double EPSILON = 1e-5; // TODO - move
											
		// using Cramer's rule
		double zn = matrixDeterminant(m_a, m_b, another.m_a, another.m_b);
		if (std::abs(zn) < EPSILON)
			return false;

		result.x = -matrixDeterminant(m_c, m_b, another.m_c, another.m_b) / zn;
		result.y = -matrixDeterminant(m_a, m_c, another.m_a, another.m_c) / zn;
		return true;
	}

	bool isParallel(const LineEquation& another) const 
	{
		return std::abs(matrixDeterminant(m_a, m_b, another.m_a, another.m_b)) < PointD::k_epsilon;
	}

	bool isEquivalent(const LineEquation& another)
	{
		return std::abs(matrixDeterminant(m_a, m_b, another.m_a, another.m_b)) < PointD::k_epsilon
			&& std::abs(matrixDeterminant(m_a, m_c, another.m_a, another.m_c)) < PointD::k_epsilon
			&& std::abs(matrixDeterminant(m_b, m_c, another.m_b, another.m_c)) < PointD::k_epsilon;
	}

};
