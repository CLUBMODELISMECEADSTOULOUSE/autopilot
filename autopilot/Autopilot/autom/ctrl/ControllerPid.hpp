/*
 * ControllerPid.h
 *
 *  Created on: 16 janv. 2013
 *      Author: Aberzen
 */

#ifndef CONTROLLERPID_H_
#define CONTROLLERPID_H_

#include <math/MathUtils.hpp>

namespace autom {



/**
 * @brief PID controller class
 *
 * This controller performs a proportional, derivative and integral control
 * with rate bias support and integral term saturation.
 *
 * The output value is computed in 3 steps.
 * 0) Assumption
 * ctrlErr = guid - meas
 *
 * 1) Compute the integral term
 * intCtrlErr = intCtrlErr + ctrlErr * dt
 *
 * 2) Saturate the integral term
 * intCtrlErr = sign(intCtrlErr) * max(abs(intCtrlErr), maxI);
 *
 * 3) Compute output
 *
 *   3a) Normal case: if abs(ctrlErr)<=rbThd
 *   out = Kp * ctrlErr + Kd * derivCtrlErr + Ki * integral(ctrlErr)
 *
 *   3b) Rate bias case: else abs(ctrlErr)>rbThd
 *   out = Krb * (derivCtrlErr + sign(ctrlErr)*rb)
 *
 */

template<typename T>
class ControllerPid {

public:
	typedef struct {
		T Kp;
		T Kd;
		T Ki;
		T maxI;
	} Parameter ;
public:
	ControllerPid(
			/* Parameters */
			const Parameter& param);
	virtual ~ControllerPid();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Compute command */
	virtual void computeCommand(
			const T& ctrlErr,
			const T& derivCtrlErr,
			const T& errCommandPrev,
			T& command);

	inline void setParam(const Parameter& param);

protected:
	/** @brief Controller parameters */
	const Parameter& _param;

	/** @brief Previous control error (used for integral term computation) */
	T _ctrlErrPrev;

	/** @brief Current integral of the control error */
	T _intCtrlErr;
};

template <typename T>
void ControllerPid<T>::setParam(
		const Parameter& param)
{
	_param = param;
}

template<typename T>
ControllerPid<T>::ControllerPid(
		/* Parameters */
		const Parameter& param)
: _param(param),
  _ctrlErrPrev((T)0),
  _intCtrlErr((T)0)
{
}

template<typename T>
ControllerPid<T>::~ControllerPid(){
}

/** @brief Init the process */
template<typename T>
void ControllerPid<T>::initialize()
{
	this->_intCtrlErr = 0;
	this->_ctrlErrPrev = 0;
}

/** @brief Compute the controller value */
template<typename T>
void ControllerPid<T>::computeCommand(
		const T& ctrlErr,
		const T& derivCtrlErr,
		const T& errCommandPrev,
		T& command)
{
	/* 0) Anti windup
	 * Remove command error from integral term
	 * _intCtrlErr = dem
	 * errCommandPrev = dem - est
	 * _intCtrlErr = _intCtrlErr - errCommandPrev
	 */
	if (_intCtrlErr < ((T) 0))
	{
		_intCtrlErr = math_min(((T) 0),_intCtrlErr-errCommandPrev);
	}
	else
	{
		_intCtrlErr = math_max(((T) 0),_intCtrlErr-errCommandPrev);
	}

	/* 1) Compute integral term */
	_intCtrlErr +=  this->_param.Ki * ctrlErr;

	/* 2) Saturation of integral term */
	_intCtrlErr  = math_min(_intCtrlErr,  this->_param.maxI);
	_intCtrlErr  = math_max(_intCtrlErr, -this->_param.maxI);

	/* 3) Compute the command summing the different terms */
	command = _param.Kp * ctrlErr
			+ _intCtrlErr
			+ _param.Kd * derivCtrlErr;
}

} /* namespace autom */
#endif /* CONTROLLERPID_H_ */
