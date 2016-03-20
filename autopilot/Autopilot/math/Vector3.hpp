// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Copyright 2010 Michael Smith, all rights reserved.

//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

// Derived closely from:
/****************************************
* 3D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*   11-12-2003
*   18-12-2003
*   06-06-2004
*
* © 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
*
* Notes:
* if a*b = 0 then a & b are orthogonal
* a%b = -b%a
* a*(b%c) = (a%b)*c
* a%b = a(cast to matrix)*b
* (a%b).norm() = area of parallelogram formed by a & b
* (a%b).norm() = a.norm()*b.norm() * sin(angle between a & b)
* (a%b).norm() = 0 if angle between a & b = 0 or a.norm() = 0 or b.norm() = 0
* a * (b%c) = volume of parallelpiped formed by a, b, c
* vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
* scalar triple product: a*(b%c) = c*(a%b) = b*(c%a)
* vector quadruple product: (a%b)*(c%d) = (a*c)*(b*d) - (a*d)*(b*c)
* if a is unit vector along b then a%b = -b%a = -b(cast to matrix)*a = 0
* vectors a1...an are linearly dependant if there exists a vector of scalars (b) where a1*b1 + ... + an*bn = 0
*    or if the matrix (A) * b = 0
*
****************************************/

#ifndef VECTOR3_H
#define VECTOR3_H

#include <inttypes.h>
#include <math.h>
#include <math/MathUtils.hpp>

namespace math {

template <typename T>
class Vector3
{
public:
    T x, y, z;

    // trivial ctor
    Vector3<T>() {
    	x = y = z = 0;
    }

    // setting ctor
    Vector3<T>(const T x0, const T y0, const T z0) : x(x0), y(y0), z(z0) {
    }

    // setting vector
    Vector3<T>(const Vector3<T>& v) :
    		x(v.x),
    		y(v.y),
    		z(v.z)
    		{
    }

    // function call operator
    void operator ()(const T x0, const T y0, const T z0)
    {
    	x= x0;
    	y= y0;
    	z= z0;
    }

    // function call operator
    void operator ()(const Vector3<T> v)
    {
    	x= v.x;
    	y= v.y;
    	z= v.z;
    }

    // function call operator
    T operator [](const uint8_t idx) const
    {
    	T res;
    	switch (idx)
    	{
    	case 0:
    		res = x;
    		break;
    	case 1:
    		res = y;
    		break;
    	case 2:
    		res = z;
    		break;
    	default:
    		res=(T)0;
    	}
    	return res;
    }

    // test for equality
    bool operator==(const Vector3<T> &v)
    {
    	return (x==v.x && y==v.y && z==v.z);
    }

    // test for inequality
    bool operator !=(const Vector3<T> &v)
    {
    	return (x!=v.x || y!=v.y || z!=v.z);
    }

    // multiplication term by term
    Vector3<T>& operator *= (const Vector3<T>& v);
    // division term by term
	Vector3<T>& operator /= (const Vector3<T>& v);


    // negation
    Vector3<T> operator - (void) const
    {
    	return Vector3<T>(-x,-y,-z);
    }

    // addition
    static void add(const Vector3<T>& v1, const Vector3<T>& v2, Vector3<T>& v)
    {
    	v(v1.x+v2.x, v1.y+v2.y, v1.z+v2.z);
    }
    Vector3<T> operator +(const Vector3<T> &v) const
    {
    	return Vector3<T>(x+v.x, y+v.y, z+v.z);
    }

    // subtraction
    static void substract(const Vector3<T>& v1, const Vector3<T>& v2, Vector3<T>& v);
    Vector3<T> operator -(const Vector3<T> &v) const
    {
    	return Vector3<T>(x-v.x, y-v.y, z-v.z);
    }


    // uniform scaling
    static void multByScalar(T scalar, Vector3<T>& v);
    Vector3<T> operator *(const T& num) const
    {
    	Vector3<T> temp(*this);
    	return temp*=num;
    }

    // uniform scaling
    static void divByScalar(T scalar, Vector3<T>& v);
    Vector3<T> operator /(const T& num) const
    {
    	Vector3<T> temp(*this);
    	return temp /= num;
    }

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v)
    {
    	x+=v.x; y+=v.y; z+=v.z;
    	return *this;
    }

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v)
    {
    	x-=v.x; y-=v.y; z-=v.z;
    	return *this;
    }

    // uniform scaling
    Vector3<T> &operator *=(const T num)
    {
    	x*=num;
    	y*=num;
    	z*=num;
    	return *this;
    }

    // uniform scaling
    Vector3<T> &operator /= (const T& num)
    {
    	x /= num;
    	y /= num;
    	z /= num;
    	return *this;
    }

    // dot product
    T operator *(const Vector3<T>& v) const;

    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const;

    // gets the norm of this vector squared
    T      norm_squared() const
    {
    	return (T)(*this * *this);
    }

    // gets the norm of this vector
    T    norm(void) const;

    // normalizes this vector
    T normalize(int nIter=1, T guess=1.);

    // returns the normalized version of this vector
    Vector3<T> normalized(int nIter=1, T guess=1.) const;

    // zero the vector
    void zero()
    {
    	x = y = z = 0.0;
    }

    // reflects this vector about n
    void reflect(const Vector3<T> &n)
    {
    	Vector3<T> orig(*this);
    	project(n);
    	*this= *this*2 - orig;
    }

    // projects this vector onto v
    void project(const Vector3<T> &v)
    {
    	*this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    Vector3<T> projected(const Vector3<T> &v)
    {
    	return v * (*this * v)/(v*v);
    }

    // computes the angle between 2 arbitrary vectors
    T angle(const Vector3<T> &v1, const Vector3<T> &v2)
    {
    	return (T)acos((v1*v2) / (v1.norm()*v2.norm()));
    }

    // computes the angle between this vector and another vector
    T angle(const Vector3<T> &v2)
    {
//    	return (T)acos(((*this)*v2) / (this->norm()*v2.norm()));
    	return (T)atan2( ((*this)%v2).norm(), (*this)*v2);
    }

    // computes the angle between 2 arbitrary normalized vectors
    T angle_normalized(const Vector3<T> &v1, const Vector3<T> &v2)
    {
    	return (T)acos(v1*v2);
    }

    // check if any elements are NAN
    bool is_nan(void) const
    {
    	return isnan(x) || isnan(y) || isnan(z);
    }

    // check if any elements are infinity
    bool is_inf(void) const
    {
    	return isinf(x) || isinf(y) || isinf(z);
    }

    // rotate by a standard rotation
//    void rotate(enum Rotation rotation);

};

// multiplication term by term
template <typename T>
Vector3<T>& Vector3<T>::operator *= (const Vector3<T>& v)
{
	this->x *= v.x;
	this->y *= v.y;
	this->z *= v.z;
	return *this;
}

// division term by term
template <typename T>
Vector3<T>& Vector3<T>::operator /= (const Vector3<T>& v)
{
	this->x /= v.x;
	this->y /= v.y;
	this->z /= v.z;
	return *this;
}

// vector cross product
template <typename T>
Vector3<T> Vector3<T>::operator %(const Vector3<T> &v) const
{
    Vector3<T> temp(
    		y*v.z - z*v.y,
			z*v.x - x*v.z,
			x*v.y - y*v.x);
    return temp;
}

// dot product
template <typename T>
T Vector3<T>::operator *(const Vector3<T>& v) const
{
    return x*v.x + y*v.y + z*v.z;
}

template <typename T>
T Vector3<T>::norm(void) const {
	return sqrt(*this * *this);
}

template <typename T>
T Vector3<T>::normalize(int nIter, T guess)
{
	T res = iter_invSqrt(guess, nIter, *this * *this);
	*this *= res;
	return res;
}

template <typename T>
Vector3<T> Vector3<T>::normalized(int nIter, T guess) const
{
	T invNrm = iter_invSqrt(guess, nIter, *this * *this);
	Vector3<T> res (*this);
	res *= invNrm;
	return res;
}


// only define for signed numbers
//template void Vector3<float>::rotate(enum Rotation);
//template float Vector3<float>::norm(void) const;
//template Vector3<float> Vector3<float>::operator %(const Vector3<float> &v) const;
//template float Vector3<float>::operator *(const Vector3<float> &v) const;

} /* namespace math */

namespace math {
typedef Vector3<int16_t> Vector3i;
typedef Vector3<uint16_t> Vector3ui;
typedef Vector3<int32_t> Vector3l;
typedef Vector3<uint32_t> Vector3ul;
typedef Vector3<float> Vector3f;
}

#endif // VECTOR3_H
