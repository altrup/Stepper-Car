// AccelMultiStepper.h

#ifndef AccelMultiStepper_h
#define AccelMultiStepper_h

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

#define ACCELMULTISTEPPER_MAX_STEPPERS 10

class AccelStepper;

/////////////////////////////////////////////////////////////////////
/// \class AccelMultiStepper AccelMultiStepper.h <AccelMultiStepper.h>
/// \brief Operate multiple AccelSteppers in a co-ordinated fashion
///
/// This class can manage multiple AccelSteppers (up to ACCELMULTISTEPPER_MAX_STEPPERS = 10), 
/// and cause them all to move to selected positions with same speed and acceleration 
// so that they arrive at their target position at the same time. NOTE: THE TARGET POSITIONS MUST HAVE SAME MAGNITUDE 
// This can be used to support devices with multiple steppers
/// on say multiple axes to cause linear diagonal motion. Suitable for use with X-Y plotters, flatbeds,
/// 3D printers etc
/// to get linear straight line movement between arbitrary 2d (or 3d or ...) positions.
class AccelMultiStepper
{
public:
    /// Constructor
    AccelMultiStepper(float velocity, float acceleration);

    /// Add a stepper to the set of managed steppers
    /// There is an upper limit of ACCELMULTISTEPPER_MAX_STEPPERS = 10 to the number of steppers that can be managed
    /// \param[in] stepper Reference to a stepper to add to the managed list
    /// \return true if successful. false if the number of managed steppers would exceed ACCELMULTISTEPPER_MAX_STEPPERS
    boolean addStepper(AccelStepper& stepper);

    // updates max speed and acceleration of all steppers
    void setMaxSpeed(float max_speed);
    void setAcceleration(float acceleration);
    // getters for max speed and acceleration
    float maxSpeed();
    float acceleration();

    // returns estimated time to travel num_steps
    float getTime(long num_steps);

    /// Set the target positions of all managed steppers 
    /// according to a coordinate array.
    /// New speeds will be computed for each stepper so they will all arrive at their 
    /// respective targets at very close to the same time.
    /// \param[in] absolute An array of desired absolute stepper positions. absolute[0] will be used to set
    /// the absolute position of the first stepper added by addStepper() etc. The array must be at least as long as 
    /// the number of steppers that have been added by addStepper, else results are undefined.
    void moveTo(long absolute[]);
    
    /// Calls runSpeed() on all the managed steppers
    /// that have not acheived their target position.
    /// \return true if any stepper is still in the process of running to its target position.
    boolean run();

    /// Runs all managed steppers until they acheived their target position.
    /// Blocks until all that position is acheived. If you dont
    /// want blocking consider using run() instead.
    void runToPosition();
    
private:
    /// Array of pointers to the steppers we are controlling.
    /// Fills from 0 onwards
    AccelStepper* _steppers[ACCELMULTISTEPPER_MAX_STEPPERS];

    /// Number of steppers we are controlling and the number
    /// of steppers in _steppers[]
    uint8_t       _num_steppers;

    // Store speed and acceleration to set to all added steppers
    float _max_speed;
    float _acceleration;
};

/// @example MultiStepper.pde
/// Use MultiStepper class to manage multiple steppers and make them all move to 
/// the same position at the same time for linear 2d (or 3d) motion.

#endif
