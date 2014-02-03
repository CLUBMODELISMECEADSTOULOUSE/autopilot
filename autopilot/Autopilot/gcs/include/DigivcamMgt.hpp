/*
 * DigivcamMgt.hpp
 *
 *  Created on: 5 juin 2013
 *      Author: Aberzen
 */

#ifndef DIGIVCAMMGT_HPP_
#define DIGIVCAMMGT_HPP_

#include <stdint.h>

namespace mavlink {

class DigivcamMgt {
public:
	DigivcamMgt();
	virtual ~DigivcamMgt();

	void configure(
			float extra_value,
			uint16_t shutter_speed,
			uint8_t mode,
			uint8_t aperture,
			uint8_t iso,
			uint8_t exposure_type,
			uint8_t command_id,
			uint8_t engine_cut_off,
			uint8_t extra_param
	);

	void control(
			float extra_value,
			uint8_t session,
			uint8_t zoom_pos,
			int8_t zoom_step,
			uint8_t focus_lock,
			uint8_t shot,
			uint8_t command_id,
			uint8_t extra_param
	);
};

} /* namespace mavlink */
#endif /* DIGIVCAMMGT_HPP_ */