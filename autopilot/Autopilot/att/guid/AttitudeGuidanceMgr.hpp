/*
 * AttitudeGuidanceMgr.hpp
 *
 *  Created on: 20 mars 2016
 *      Author: AdministrateurLocal
 */

#ifndef ATT_GUID_ATTITUDEGUIDANCEMGR_HPP_
#define ATT_GUID_ATTITUDEGUIDANCEMGR_HPP_

#include <att/guid/AttitudeGuidanceAccro.hpp>
#include <att/guid/AttitudeGuidanceAutoStab.hpp>

namespace attitude {

class AttitudeGuidanceMgr {
public:
	typedef enum {
		E_MODE_NONE = 0,
		E_MODE_AUTOSTAB,
		E_MODE_AUTOSTAB_NOYAW,
		E_MODE_ACCRO
	} Mode;

public:

	AttitudeGuidanceMgr();
	virtual ~AttitudeGuidanceMgr();

	/** @brief Initialize mode manager */
	void initialize();

	/** @brief Execute guidance manager */
	void execute();

	/** @brief Set new mode */
	void setMode(Mode mode);

protected:

	/** @brief Current mode identifier */
	Mode _currentModeIdentifier;

	/** @brief Current mode */
	AttitudeGuidanceMode* _currentMode;

	/** @brief Mode accro */
	AttitudeGuidanceAccro _modeAccro;

	/** @brief Mode auto stabilized */
	AttitudeGuidanceAutoStab _modeAutoStab;

};

} /* namespace infra */

#endif /* ATT_GUID_ATTITUDEGUIDANCEMGR_HPP_ */
