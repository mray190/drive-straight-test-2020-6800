/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"

/**
 * Before Running:
 * Open shuffleBoard, select File->Load Layout and select the 
 * shuffleboard.json that is in the root directory of this example
 */

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a new control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */

class Robot : public frc::TimedRobot {
  int left_device_id[3] = {1, 2, 3};
  int right_device_id[3] = {4, 5, 6};

  rev::CANSparkMax m_left_lead{left_device_id[0], rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right_lead{right_device_id[0], rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_left_follow_1{left_device_id[1], rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right_follow_1{right_device_id[1], rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_left_follow_2{left_device_id[2], rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_right_follow_2{right_device_id[2], rev::CANSparkMax::MotorType::kBrushless};
  
  rev::CANPIDController m_left_pid = m_left_lead.GetPIDController();
  rev::CANPIDController m_right_pid = m_right_lead.GetPIDController();
  rev::CANEncoder m_left_encoder = m_left_lead.GetEncoder();
  rev::CANEncoder m_right_encoder = m_right_lead.GetEncoder();
  
  // default PID coefficients
  double kP = 5e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000156, kMaxOutput = 1, kMinOutput = -1;

  // default smart motion coefficients
  double kMaxVel = 2000, kMinVel = 0, kMaxAcc = 1500, kAllErr = 0;

  // motor max RPM
  const double MaxRPM = 5700;

 public:
  void RobotInit() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_left_lead.RestoreFactoryDefaults();
    m_right_lead.RestoreFactoryDefaults();

    // set PID coefficients
    m_left_pid.SetP(kP);
    m_left_pid.SetI(kI);
    m_left_pid.SetD(kD);
    m_left_pid.SetIZone(kIz);
    m_left_pid.SetFF(kFF);
    m_left_pid.SetOutputRange(kMinOutput, kMaxOutput);

    // set PID coefficients
    m_right_pid.SetP(kP);
    m_right_pid.SetI(kI);
    m_right_pid.SetD(kD);
    m_right_pid.SetIZone(kIz);
    m_right_pid.SetFF(kFF);
    m_right_pid.SetOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - SetSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - SetSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - SetSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - SetSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    m_left_pid.SetSmartMotionMaxVelocity(kMaxVel);
    m_left_pid.SetSmartMotionMinOutputVelocity(kMinVel);
    m_left_pid.SetSmartMotionMaxAccel(kMaxAcc);
    m_left_pid.SetSmartMotionAllowedClosedLoopError(kAllErr);

    m_right_pid.SetSmartMotionMaxVelocity(kMaxVel);
    m_right_pid.SetSmartMotionMinOutputVelocity(kMinVel);
    m_right_pid.SetSmartMotionMaxAccel(kMaxAcc);
    m_right_pid.SetSmartMotionAllowedClosedLoopError(kAllErr);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", kMaxVel);
    frc::SmartDashboard::PutNumber("Min Velocity", kMinVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", kAllErr);
    frc::SmartDashboard::PutNumber("Set Position", 0);
    frc::SmartDashboard::PutNumber("Set Velocity", 0);
  }

  void TeleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
    double minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
    double maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
    double allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP))   { m_left_pid.SetP(p); m_right_pid.SetP(p); kP = p; }
    if((i != kI))   { m_left_pid.SetI(i); m_right_pid.SetI(i); kI = i; }
    if((d != kD))   { m_left_pid.SetD(d); m_right_pid.SetD(d); kD = d; }
    if((iz != kIz)) { m_left_pid.SetIZone(iz); m_right_pid.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_left_pid.SetFF(ff); m_right_pid.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { m_left_pid.SetOutputRange(min, max); m_right_pid.SetOutputRange(min, max); kMinOutput = min; kMaxOutput = max; }
    if((maxV != kMaxVel)) { m_left_pid.SetSmartMotionMaxVelocity(maxV); m_right_pid.SetSmartMotionMaxVelocity(maxV); kMaxVel = maxV; }
    if((minV != kMinVel)) { m_left_pid.SetSmartMotionMinOutputVelocity(minV); m_right_pid.SetSmartMotionMinOutputVelocity(minV); kMinVel = minV; }
    if((maxA != kMaxAcc)) { m_left_pid.SetSmartMotionMaxAccel(maxA); m_right_pid.SetSmartMotionMaxAccel(maxA); kMaxAcc = maxA; }
    if((allE != kAllErr)) { m_left_pid.SetSmartMotionAllowedClosedLoopError(allE); m_right_pid.SetSmartMotionAllowedClosedLoopError(allE); allE = kAllErr; }

    double SetPoint, left_process_var, right_process_var;
    SetPoint = frc::SmartDashboard::GetNumber("Set Position", 0);
    /**
     * As with other PID modes, Smart Motion is set by calling the
     * SetReference method on an existing pid object and setting
     * the control type to kSmartMotion
     */
    m_left_pid.SetReference(SetPoint, rev::ControlType::kSmartMotion);
    m_right_pid.SetReference(SetPoint, rev::ControlType::kSmartMotion);
    left_process_var = m_left_encoder.GetPosition();
    right_process_var = m_right_encoder.GetPosition();
    
    frc::SmartDashboard::PutNumber("Set Point", SetPoint);
    frc::SmartDashboard::PutNumber("Left Process Variable", left_process_var);
    frc::SmartDashboard::PutNumber("Right Process Variable", right_process_var);
    frc::SmartDashboard::PutNumber("Left Output", m_left_lead.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Right Output", m_right_lead.GetAppliedOutput());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
