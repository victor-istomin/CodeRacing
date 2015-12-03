#pragma once

#include <cmath>


template <class T>
class Vec2 {
public:
	T m_x, m_y;

	Vec2()				: m_x(0), m_y(0)         {}
	Vec2(T x, T y)		: m_x(x), m_y(y)         {}
	Vec2(const Vec2& v) : m_x(v.m_x), m_y(v.m_y) {}

	Vec2& operator=(const Vec2& v)       { m_x = v.m_x; m_y = v.m_y; return *this; }
	Vec2 operator+(Vec2& v)	const        { return Vec2(m_x + v.m_x, m_y + v.m_y); }
	Vec2 operator-(Vec2& v) const        { return Vec2(m_x - v.m_x, m_y - v.m_y); }

	Vec2& operator+=(Vec2& v)            { m_x += v.m_x; m_y += v.m_y; return *this; }
	Vec2& operator-=(Vec2& v)            { m_x -= v.m_x; m_y -= v.m_y; return *this; }

	Vec2 operator+(double s) const       { return Vec2(m_x + s, m_y + s); }
	Vec2 operator-(double s) const       { return Vec2(m_x - s, m_y - s); }
	Vec2 operator*(double s) const       { return Vec2(m_x * s, m_y * s); }
	Vec2 operator/(double s) const       { return Vec2(m_x / s, m_y / s); }

	Vec2& operator+=(double s) { m_x += s; m_y += s; return *this; }
	Vec2& operator-=(double s) { m_x -= s; m_y -= s; return *this; }
	Vec2& operator*=(double s) { m_x *= s; m_y *= s; return *this; }
	Vec2& operator/=(double s) { m_x /= s; m_y /= s; return *this; }

	Vec2& set(T x, T y)        { m_x = x; m_y = y; return *this;  }

	Vec2& rotate(double deg) 
	{
		double theta = deg / 180.0 * M_PI;
		double c = cos(theta);
		double s = sin(theta);
		double tx = m_x * c - m_y * s;
		double ty = m_x * s + m_y * c;
		m_x = static_cast<T>(tx);
		m_y = static_cast<T>(ty);

		return *this;
	}

	Vec2& normalize() 
	{
		if (length() == 0) 
			return *this;

		*this /= length();
		return *this;
	}

	double dist(const Vec2& v) const { Vec2 d(v.m_x - m_x, v.m_y - m_y); return d.length(); }
	double length()            const { return std::hypot(m_x, m_y); }

	Vec2& truncate(double length)    
	{
		double a = angle();

		m_x = length * cos(a);
		m_y = length * sin(a);
		return *this;
	}

	double angle() const
	{
		return std::atan2(m_y, m_x);
	}

	Vec2 ortho() const { return Vec2(m_y, -m_x); }

	static double dot(const Vec2& v1, const Vec2& v2)        { return v1.m_x * v2.m_x + v1.m_y * v2.m_y; }
	static double cross(const Vec2& v1, const Vec2& v2)      { return (v1.m_x * v2.m_y) - (v1.m_y * v2.m_x); }

	static double angleBetween(const Vec2& a, const Vec2& b) 
	{ 
		double cosine = dot(a, b) / a.length() / b.length();
		return std::acos(cosine); // todo - sign?
	}
};

typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;
