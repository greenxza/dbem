#ifndef PWM_H_
#define PWM_H_

#include "core.h"

#define PWM_PATH "/sys/devices/ocp.3/"
#define PWM_MUX_PATH "/sys/devices/ocp.3/"

typedef struct {
	char *name;
	char *mux;
	int number;
} pwm_properties;


extern int pwm_open(pwm_properties *pwm);
extern int pwm_set_run(pwm_properties *pwm, int run);
extern int pwm_set_polarity(pwm_properties *pwm, int value);
extern int pwm_set_period(pwm_properties *pwm, int period);
extern int pwm_set_duty(pwm_properties *pwm, int duty);
extern int set_pwm_mode(pwm_properties *pwm);
extern int pwm_export(pwm_properties *pwm);

#endif /* PWM_H_ */
