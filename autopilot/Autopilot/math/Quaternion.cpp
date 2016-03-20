/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * quaternion.cpp
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <math/Quaternion.hpp>
#include <math/MathUtils.hpp>

namespace math {

// constructor creates a quaternion equivalent
// to roll=0, pitch=0, yaw=0
Quaternion::Quaternion() :
	scalar(1.),
	vector(0.,0.,0.)
	{
}

// Copy existing quaternion
Quaternion::Quaternion(const Quaternion& q):
	scalar(q.scalar),
	vector(q.vector)
{
}

// setting constructor
Quaternion::Quaternion(const float& q1, const float& q2, const float& q3, const float& q4) :
	scalar(q1),
	vector(q2,q3,q4)
	{
}

// setting constructor
Quaternion::Quaternion(const float& scal, const Vector3<float>& vect) :
	scalar(scal),
	vector(vect)
	{
}

// function call operator
void Quaternion::operator ()(const float& q1, const float& q2, const float& q3, const float& q4)
{
	scalar = q1;
	vector(q2,q3,q4);
}

// function call operator
void Quaternion::operator ()(const float& scal, const Vector3<float>& vect)
{
	scalar = scal;
	vector(vect);
}

// Copy existing quaternion
void Quaternion::operator ()(const Quaternion& q)
{
	scalar = q.scalar;
	vector = q.vector;
}

// function addition
void Quaternion::add(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes) {
	qRes.scalar = q1.scalar + q2.scalar;
	Vector3<float>::add(q1.vector, q2.vector, qRes.vector);
}

// function addition
Quaternion Quaternion::operator + (const Quaternion& q) const
{
	return Quaternion(this->scalar + q.scalar, this->vector + q.vector);
}


// return the rotation matrix equivalent for this quaternion
void Quaternion::to_dcm(Matrix3<float>& m) const
{
    float q3q3 = vector.y * vector.y;
    float q3q4 = vector.y * vector.z;
    float q2q2 = vector.x * vector.x;
    float q2q3 = vector.x * vector.y;
    float q2q4 = vector.x * vector.z;
    float q1q2 = scalar * vector.x;
    float q1q3 = scalar * vector.y;
    float q1q4 = scalar * vector.z;
    float q4q4 = vector.z * vector.z;

    m.a.x = 1.-2.*(q3q3 + q4q4);
    m.a.y =    2.*(q2q3 - q1q4);
    m.a.z =    2.*(q2q4 + q1q3);
    m.b.x =    2.*(q2q3 + q1q4);
    m.b.y = 1.-2.*(q2q2 + q4q4);
    m.b.z =    2.*(q3q4 - q1q2);
    m.c.x =    2.*(q2q4 - q1q3);
    m.c.y =    2.*(q3q4 + q1q2);
    m.c.z = 1.-2.*(q2q2 + q3q3);
}

// return the rotation matrix equivalent for this quaternion
void Quaternion::from_dcm(const Matrix3<float>& m)
{
	float den;
	float tmp1 = 1.+m.a.x+m.b.y+m.c.z;
	float tmp2 = 1.+m.a.x-m.b.y-m.c.z;
	if (tmp1>tmp2)
	{
		scalar = 0.5 * sqrt(tmp1);
		den = 0.25 / scalar;
		vector.x = (m.c.y - m.b.z) * den ;
		vector.y = (m.a.z - m.c.x) * den ;
		vector.z = (m.b.x - m.a.y) * den ;
	}
	else
	{
		vector.x = 0.5 * sqrt(tmp2);
		den = 0.25 / vector.x;
		vector.y = (m.a.y + m.b.x) * den ;
		vector.z = (m.a.z + m.c.x) * den ;
		scalar   = (m.c.y - m.b.z) * den ;
	}
}

//// convert a vector from earth to body frame
//void Quaternion::earth_to_body(Vector3<float>& v)
//{
//    Matrix3f m;
//    // we reverse z before and afterwards because of the differing
//    // quaternion conventions from APM conventions.
//    v.z = -v.z;
//    rotation_matrix(m);
//    v = m * v;
//    v.z = -v.z;
//}

// create a quaternion from Euler angles
void Quaternion::from_euler(const float& roll, const float& pitch, const float& yaw)
{
    float cr2 = cos(roll*0.5);
    float cp2 = cos(pitch*0.5);
    float cy2 = cos(yaw*0.5);
    float sr2 = sin(roll*0.5);
    float sp2 = sin(pitch*0.5);
    float sy2 = sin(yaw*0.5);

    scalar = cr2*cp2*cy2 + sr2*sp2*sy2;
    vector.x = sr2*cp2*cy2 - cr2*sp2*sy2;
    vector.y = cr2*sp2*cy2 + sr2*cp2*sy2;
    vector.z = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion 3-1-3 convention
void Quaternion::to_euler(float& roll, float& pitch, float& yaw) const
{
//	roll = (atan2(2.*(scalar*vector.x + vector.y*vector.z),
//				   1. - 2.*(vector.x*vector.x + vector.y*vector.y)));
//	// we let safe_asin() handle the singularities near 90/-90 in pitch
//	pitch = safe_asin(2.*(scalar*vector.y - vector.z*vector.x));
//
//	yaw = atan2(2.*(scalar*vector.z + vector.x*vector.y),
//				 1. - 2.*(vector.y*vector.y + vector.z*vector.z));
//	roll = (atan2((scalar*vector.y + vector.x*vector.z),
//				   -(vector.x*vector.y - scalar*vector.z)));
//	// we let safe_asin() handle the singularities near 90/-90 in pitch
//	pitch = acos(-scalar*scalar-vector.x*vector.x+vector.y*vector.y+vector.z*vector.z);
//
//	yaw = atan2((scalar*vector.y - vector.x*vector.z),
//				(vector.x*vector.y + scalar*vector.z));

	roll = atan2(2.*(scalar*vector.z + vector.x*vector.y),
			1.-2.*(vector.y*vector.y + vector.z*vector.z));
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	pitch = asin(2.*(scalar*vector.y-vector.z*vector.x));

	yaw = (atan2(2.*(scalar*vector.x + vector.y*vector.z),
			   1.-2.*(vector.x*vector.x + vector.y*vector.y)));

}



/*
// function call operator
void Quaternion::operator ()(const float scal, const float vectX, const float vectY, const float vectZ);

// function call operator
void Quaternion::operator ()(const float scal, const Vector3<float>& vect);
*/
Quaternion Quaternion::operator - (const Quaternion& q) const
{
	return Quaternion(scalar - q.scalar, vector - q.vector);
}


/** @brief Substract and store */
Quaternion& Quaternion::operator -= (const Quaternion& q)
{
	scalar -= q.scalar;
	vector -= q.vector;
	return *this;
}

/** @brief Addition and store */
Quaternion& Quaternion::operator += (const Quaternion& q)
{
	scalar += q.scalar;
	vector += q.vector;
	return *this;
}

/** @brief Multiplication */
void Quaternion::mult(const Quaternion& q1, const Quaternion& q2, Quaternion& qRes)
{
	qRes.scalar = q1.scalar*q2.scalar - (q1.vector*q2.vector);
	qRes.vector = (q1.vector*q2.scalar) + (q2.vector*q1.scalar) + (q1.vector%q2.vector);
}

Quaternion Quaternion::operator * (const Quaternion& q) const
{
	return Quaternion(
			scalar*q.scalar - (vector*q.vector),
			(vector*q.scalar) + (q.vector*scalar) + (vector%q.vector));
}


/** @brief Conjugate */
void Quaternion::conj(Quaternion& q)
{
	q.vector *= (-1.);
}

Quaternion Quaternion::operator ~ () const
{
	return Quaternion(scalar, - vector);
}

/** @brief Norm */
float Quaternion::norm() const
{
	return sqrt(scalar*scalar+vector*vector);
}

/** @brief Normalize */
float Quaternion::normalize(int8_t nIter, float guess)
{
	float invNrm = iter_invSqrt(guess, nIter, scalar * scalar + vector * vector);
	scalar *= invNrm;
	vector *= invNrm;
	return invNrm;
}

/** @brief Multiply and store */
Quaternion& Quaternion::operator *= (const Quaternion& q)
{
	scalar = scalar*q.scalar - (vector*q.vector);
	vector = (vector*q.scalar) + (q.vector*scalar) + (vector%q.vector);
	return *this;
}

/** @brief Rotation using
 * The rotation of V_I such that  V_B = rotate(V_I).
 *
 * The mathematical operation is:
 * V_B = (~q_BI) x V_I x q_BI
 *
 * Where q_BI is the rotation from B to I;
 */
Vector3<float> Quaternion::rotateQconjVQ (const Vector3<float>& v_I) const
{
	return (~*this).rotateQVQconj(v_I);
}

/** @brief Rotation using
 * The rotation of V_I such that  V_B = rotate(V_I).
 *
 * The mathematical operation is:
 * V_I = q_BI x V_B x (~q_BI)
 *
 * Where q_BI is the rotation from B to I;
 */
Vector3<float> Quaternion::rotateQVQconj (const Vector3<float>& v_B) const
{
//	Quaternion  qVect_B(0., v_B);
//	s1*s2 - v1*v2
//	v2*s1 + v1*s2 + v1%v2

	math::Vector3f tmp1 = v_B*scalar + vector%v_B;
	return vector*(vector*v_B) + tmp1*scalar + vector%tmp1;

//	Quaternion  qVect_I = (*this)*qVect_B*(~(*this));
//	return qVect_I.vector;
}


bool Quaternion::is_nan(void) const
{
	return (isnan(scalar) || vector.is_nan());
}

/*
void Quaternion::rotation_matrix(Matrix3f& m);

// create a quaternion from Euler angles
void Quaternion::from_euler(float roll, float pitch, float yaw);

// create eulers from a quaternion
void Quaternion::to_euler(float& roll, float& pitch, float& yaw);
*/

/** @brief Multiply by scalar and store */
Quaternion& Quaternion::operator *= (const float& scal)
{
	scalar *= scal;
	vector *= scal;
	return *this;
}

/** @brief Divide by scalar and store */
Quaternion& Quaternion::operator /= (const float& scal)
{
	float val = 1. / scal;
	*this *= val;
	return *this;
}

/** @brief Multiply by scalar */
Quaternion Quaternion::operator * (const float& scal)
{
	Quaternion res = *this;
	res *= scal;
	return res;
}

/** @brief Divide by scalar */
Quaternion Quaternion::operator / (const float& scal)
{
	Quaternion res = *this;
	res /= scal;
	return res;
}

}
