#ifndef PWM_H__
#define PWM_H__

#include <stdint.h>
#include <hw/common/Driver.hpp>

namespace hw {

// Radio channels

// Note channels are from 0!
#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7
#define CH_9 8
#define CH_10 9
#define CH_11 10

//#define NUM_CHANNELS 8
#define MIN_CHANNELS 5      // for ppm sum we allow less than 8 channels to make up a valid packet


typedef uint16_t pwm_t;
template <int8_t PWM_NUM_FROM_DEVICE, int8_t PWM_NUM_TO_DEVICE>
class Pwm : public Driver
{
public:
	Pwm();
	virtual ~Pwm();

	virtual void write(uint8_t idx, const uint16_t& pwmValue) = 0;
	virtual void read(uint8_t idx, uint16_t& pwmValue) = 0;
	virtual void write(const pwm_t* pwmValues);
	virtual void read(pwm_t* pwmValues);

    virtual uint8_t getState() = 0;
    virtual void setFastOutputChannels(
    		uint32_t channelmask,
			uint16_t speed_hz = 400 ) = 0;
    virtual void enable_out(uint8_t) = 0;
    virtual void disable_out(uint8_t) = 0;

protected:

    uint16_t _map_speed(uint16_t speed_hz) {
        return 2000000UL / speed_hz;
    }
};

template <int8_t PWM_NUM_FROM_DEVICE, int8_t PWM_NUM_TO_DEVICE>
Pwm<PWM_NUM_FROM_DEVICE, PWM_NUM_TO_DEVICE>::Pwm()
: Driver()
{

}
template <int8_t PWM_NUM_FROM_DEVICE, int8_t PWM_NUM_TO_DEVICE>
Pwm<PWM_NUM_FROM_DEVICE, PWM_NUM_TO_DEVICE>::~Pwm()
{

}

template <int8_t PWM_NUM_FROM_DEVICE, int8_t PWM_NUM_TO_DEVICE>
void Pwm<PWM_NUM_FROM_DEVICE, PWM_NUM_TO_DEVICE>::write(const pwm_t* pwmValues)
{
	uint8_t idx;
	for (idx=0 ; idx<PWM_NUM_TO_DEVICE ; idx++)
	{
		write(idx, pwmValues[idx]);
	}
}
template <int8_t PWM_NUM_FROM_DEVICE, int8_t PWM_NUM_TO_DEVICE>
void Pwm<PWM_NUM_FROM_DEVICE, PWM_NUM_TO_DEVICE>::read(pwm_t* pwmValues)
{
	uint8_t idx;
	for (idx=0 ; idx<PWM_NUM_FROM_DEVICE ; idx++)
	{
		read(idx, pwmValues[idx]);
	}
}



}  /* namespace hw */

#endif

