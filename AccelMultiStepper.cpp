// AccelMultiStepper.cpp
//
// Copyright (C) 2015 Mike McCauley
// $Id: MultiStepper.cpp,v 1.3 2020/04/20 00:15:03 mikem Exp mikem $

#include "AccelMultiStepper.h"
#include "AccelStepper.h"

AccelMultiStepper::AccelMultiStepper(float max_speed, float acceleration)
    : _num_steppers(0)
{
	setMaxSpeed(max_speed);
	setAcceleration(acceleration);
}

void AccelMultiStepper::setMaxSpeed(float max_speed) { 
	_max_speed = max_speed;
	// update existing steppers
    uint8_t i;
	for (i = 0; i < _num_steppers; i ++) {
		_steppers[i]->setMaxSpeed(_max_speed);
	}
}
void AccelMultiStepper::setAcceleration(float acceleration) {
	_acceleration = acceleration;
	// update existing steppers
    uint8_t i;
	for (i = 0; i < _num_steppers; i ++) {
		_steppers[i]->setAcceleration(_max_speed);
	}
}
float AccelMultiStepper::maxSpeed() { return _max_speed; }
float AccelMultiStepper::acceleration() { return _acceleration; }

float AccelMultiStepper::getTime(long num_steps) {
	// calculate number of steps we take while accelerating to max speed
	float accelerating_steps = _max_speed * _max_speed / _acceleration / 2;
	// if we never reach set max speed
	if (num_steps < 2 * accelerating_steps) {
		// calculate max speed we reach
		float reached_speed = sqrt(num_steps * _acceleration);
		// calculate total time
		return 2 * num_steps / reached_speed;
	}
	// if we do reach max speed, calculate time traveled at max speed
	float max_speed_time = (num_steps - 2 * accelerating_steps) / _max_speed;
	// calculate total time
	return 2 * (_max_speed / _acceleration) + max_speed_time;
}

boolean AccelMultiStepper::addStepper(AccelStepper& stepper)
{
    if (_num_steppers >= ACCELMULTISTEPPER_MAX_STEPPERS)
	return false; // No room for more
	stepper.setMaxSpeed(_max_speed);
	stepper.setAcceleration(_acceleration);
    _steppers[_num_steppers++] = &stepper;
    return true;
}

void AccelMultiStepper::moveTo(long absolute[])
{
    uint8_t i;
	for (i = 0; i < _num_steppers; i ++) {
		_steppers[i]->moveTo(absolute[i]);
	}
}

// Returns true if any motor is still running to the target position.
boolean AccelMultiStepper::run()
{
    uint8_t i;
    boolean ret = false;
    for (i = 0; i < _num_steppers; i++)
    {
	if ( _steppers[i]->distanceToGo() != 0)
	{
	    _steppers[i]->run();
	    ret = true;
	}
	// Caution: it has een reported that if any motor is used with acceleration outside of
	// MultiStepper, this code is necessary, you get 
	// strange results where it moves in the wrong direction for a while then 
	// slams back the correct way.
#if 0
	else
	{
	    // Need to call this to clear _stepInterval, _speed and _n 
	    otherwise future calls will fail.
		_steppers[i]->setCurrentPosition(_steppers[i]->currentPosition());
	}
#endif
	
    }
    return ret;
}

// Blocks until all steppers reach their target position and are stopped
void AccelMultiStepper::runToPosition()
{ 
    while (run())
	;
}

