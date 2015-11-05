#ifndef PWMAPM25_H__
#define PWMAPM25_H__


#include <hw/pwm/Pwm.hpp>

#define APM25_PWM_NUM_TO_DEVICE 11
#define APM25_PWM_NUM_FROM_DEVICE 8

namespace hw {

class PwmApm25 : public hw::Pwm<APM25_PWM_NUM_FROM_DEVICE, APM25_PWM_NUM_TO_DEVICE>
{
private:
public:
	PwmApm25();
	virtual ~PwmApm25();

	/** @brief Init the process */
	virtual bool initialize() ;

	virtual void write(uint8_t idx, const pwm_t& pwmValues);
	virtual void read(uint8_t idx, uint16_t& pwmValues);
	virtual void write(const pwm_t* pwmValues);
	virtual void read(pwm_t* pwmValues);

	/** @brief Reset the HW */
	virtual void reset() ;

    virtual uint8_t getState();
    virtual void setFastOutputChannels(uint32_t chmask, uint16_t speed_hz = 400);

    virtual void enable_out(uint8_t);
    virtual void  disable_out(uint8_t);

public:
    static void _timer5_capt_cb(void);

protected:
    void OutputCh(uint8_t ch, uint16_t pwm);

public:
    static volatile uint16_t        _PWM_RAW[APM25_PWM_NUM_FROM_DEVICE];
    static volatile uint8_t         _radio_status;

};

} /* namespace hw */
#endif
