/*
 * ControllerPid3Axes.hpp
 *
 *  Created on: 10 juin 2013
 *      Author: Aberzen
 */

#ifndef CONTROLLERPIS3AXES_HPP_
#define CONTROLLERPIS3AXES_HPP_

#include <math/Vector3.hpp>
#include <autom/ctrl/ControllerPid.hpp>

namespace autom {

template <typename T>
class ControllerPid3Axes {
public:
	ControllerPid3Axes(
			const math::Vector3<T>& Kp,
			const math::Vector3<T>& Kd,
			const math::Vector3<T>& Ki,
			const math::Vector3<T>& maxI) ;
	virtual ~ControllerPid3Axes();

	/** @brief Init the process */
	virtual void initialize();

	/** @brief Setter method of controller parameters */
	inline void setParam(
			const math::Vector3<T>& Kp,
			const math::Vector3<T>& Kd,
			const math::Vector3<T>& Ki,
			const math::Vector3<T>& maxI);

	/** @brief Compute the command */
	virtual void computeCommand(
			const math::Vector3<T>& ctrlErr,
			const math::Vector3<T>& derivCtrlErr,
			math::Vector3<T>& command);

protected:

	/** @brief PID controller for axis X */
	ControllerPid<T> _ctrl_x;

	/** @brief PID controller for axis Y */
	ControllerPid<T> _ctrl_y;

	/** @brief PID controller for axis Z */
	ControllerPid<T> _ctrl_z;
};

/** @brief Setter method of controller parameters */
template <typename T>
void ControllerPid3Axes<T>::setParam(
		const math::Vector3<T>& Kp,
		const math::Vector3<T>& Kd,
		const math::Vector3<T>& Ki,
		const math::Vector3<T>& maxI)
{
	_ctrl_x.setParam(
			Kp.x,
			Kd.x,
			Ki.x,
			maxI.x);
	_ctrl_y.setParam(
			Kp.y,
			Kd.y,
			Ki.y,
			maxI.y);
	_ctrl_z.setParam(
			Kp.z,
			Kd.z,
			Ki.z,
			maxI.z);
}

/**
 * \brief Constructor for the PID 3 axes controller class
 *
 * \details Instantiate an object for the class ControllerPid3Axes
 *
 * \param[in]  dt		Constant reference to the sampling period
 * \param[in]  param	Constant rReference to the parameter structure
 */
template <typename T>
ControllerPid3Axes<T>::ControllerPid3Axes (
		/* Parameters */
		const math::Vector3<T>& Kp,
		const math::Vector3<T>& Kd,
		const math::Vector3<T>& Ki,
		const math::Vector3<T>& maxI)
: _ctrl_x(
		Kp.x,
		Kd.x,
		Ki.x,
		maxI.x),
  _ctrl_y(
		Kp.y,
		Kd.y,
		Ki.y,
		maxI.y),
  _ctrl_z(
		Kp.z,
		Kd.z,
		Ki.z,
		maxI.z)
{
} ;

template <typename T>
ControllerPid3Axes<T>::~ControllerPid3Axes() {
}

/** @brief Init the process */
template <typename T>
void ControllerPid3Axes<T>::initialize()
{
	/* Initialize per axis controllers */
	_ctrl_x.initialize();
	_ctrl_y.initialize();
	_ctrl_z.initialize();
}

/** @brief Compute the command */
template <typename T>
void ControllerPid3Axes<T>::computeCommand(
		const math::Vector3<T>& ctrlErr,
		const math::Vector3<T>& derivCtrlErr,
		math::Vector3<T>& command)
{
	/* Execute the contollers */
	_ctrl_x.computeCommand(ctrlErr.x, derivCtrlErr.x, command.x);
	_ctrl_y.computeCommand(ctrlErr.y, derivCtrlErr.y, command.y);
	_ctrl_z.computeCommand(ctrlErr.z, derivCtrlErr.z, command.z);
}

} /* namespace autom */
#endif /* CONTROLLERPIS3AXES_HPP_ */
