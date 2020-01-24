/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <iostream>
#include <fstream>
#include <frc/Timer.h>

class Robot : public frc::TimedRobot {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  rev::CANSparkMax::MotorType::kBrushless
   *  rev::CANSparkMax::MotorType::kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1, 2, 3 and 4. Change
   * these parameters to match your setup
   */
  static const int leftLeadDeviceID = 1, leftFollow1DeviceID = 2, leftFollow2DeviceID = 3, rightLeadDeviceID = 4, rightFollow1DeviceID = 5, rightFollow2DeviceID = 6;
  
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollow1Motor{leftFollow1DeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollow2Motor{leftFollow2DeviceID, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollow1Motor{rightFollow1DeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollow2Motor{rightFollow2DeviceID, rev::CANSparkMax::MotorType::kBrushless};

  /**
   * In order to use PID functionality for a controller, a CANPIDController object
   * is constructed by calling the GetPIDController() method on an existing
   * CANSparkMax object
   */
  rev::CANPIDController m_leftPIDController = m_leftLeadMotor.GetPIDController();
  rev::CANPIDController m_rightPIDController = m_rightLeadMotor.GetPIDController();

  // Encoder object created to display velocity values
  rev::CANEncoder m_leftEncoder = m_leftLeadMotor.GetEncoder();
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor.GetEncoder();

  // default PID coefficients
  double kP = 6e-5, kI = 1e-6, kD = 0, kIz = 0, kFF = 0.000015, kMaxOutput = 1.0, kMinOutput = -1.0;

  // motor max RPM
  const double MaxRPM = 5700;

  /**
   * In RobotInit() below, we will configure m_leftFollowMotor and m_rightFollowMotor to follow 
   * m_leftLeadMotor and m_rightLeadMotor, respectively. 
   * 
   * Because of this, we only need to pass the lead motors to m_robotDrive. Whatever commands are 
   * sent to them will automatically be copied by the follower motors
   */
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::Joystick m_stick{0};

  /**
   * Output stream to log motor current
   **/
  std::ofstream logging_file;
  
 public:

  void setIdleMode(rev::CANSparkMax::IdleMode mode) {
    m_leftLeadMotor.SetIdleMode(mode);
    m_leftFollow1Motor.SetIdleMode(mode);
    m_leftFollow2Motor.SetIdleMode(mode);

    m_rightLeadMotor.SetIdleMode(mode);
    m_rightFollow1Motor.SetIdleMode(mode);
    m_rightFollow2Motor.SetIdleMode(mode);
  }

  void InitDrives() {
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_leftLeadMotor.RestoreFactoryDefaults();
    m_leftFollow1Motor.RestoreFactoryDefaults();
    m_leftFollow2Motor.RestoreFactoryDefaults();
    m_rightLeadMotor.RestoreFactoryDefaults();
    m_rightFollow1Motor.RestoreFactoryDefaults();
    m_rightFollow2Motor.RestoreFactoryDefaults();

    /**
     * Set inversion to false to gaurentee motors go in the same direction
     **/
    m_leftLeadMotor.SetInverted(false);
    m_leftFollow1Motor.SetInverted(false);
    m_leftFollow2Motor.SetInverted(false);
    m_rightLeadMotor.SetInverted(false);
    m_rightFollow1Motor.SetInverted(false);
    m_rightFollow2Motor.SetInverted(false);
    
    /**
     * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
     * the Follow() method on the SPARK MAX you want to configure as a follower, and by passing
     * as a parameter the SPARK MAX you want to configure as a leader.
     * 
     * This is shown in the example below, where one motor on each side of our drive train is
     * configured to follow a lead motor.
     */
    m_leftFollow1Motor.Follow(m_leftLeadMotor);
    m_rightFollow1Motor.Follow(m_rightLeadMotor);

    m_leftFollow2Motor.Follow(m_leftLeadMotor);
    m_rightFollow2Motor.Follow(m_rightLeadMotor);

    // set motors to brake mode
    setIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    // set PID coefficients
    m_leftPIDController.SetP(kP);
    m_leftPIDController.SetI(kI);
    m_leftPIDController.SetD(kD);
    m_leftPIDController.SetIZone(kIz);
    m_leftPIDController.SetFF(kFF);
    m_leftPIDController.SetOutputRange(kMinOutput, kMaxOutput);

    m_rightPIDController.SetP(kP);
    m_rightPIDController.SetI(kI);
    m_rightPIDController.SetD(kD);
    m_rightPIDController.SetIZone(kIz);
    m_rightPIDController.SetFF(kFF);
    m_rightPIDController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
  }

  void LogOutput(double setpoint) {
    logging_file << frc::Timer::GetFPGATimestamp() << ",";
    logging_file << setpoint << ",";
    logging_file << m_leftEncoder.GetPosition() << ",";
    logging_file << m_rightEncoder.GetPosition() << ",";
    logging_file << m_leftEncoder.GetVelocity() << ",";
    logging_file << m_rightEncoder.GetVelocity() << ",";
    logging_file << m_leftLeadMotor.GetAppliedOutput() << ",";
    logging_file << m_rightLeadMotor.GetAppliedOutput() << ",";
    logging_file << m_leftLeadMotor.GetOutputCurrent() << ",";
    logging_file << m_leftFollow1Motor.GetOutputCurrent() << ",";
    logging_file << m_leftFollow2Motor.GetOutputCurrent() << ",";
    logging_file << m_rightLeadMotor.GetOutputCurrent() << ",";
    logging_file << m_rightFollow1Motor.GetOutputCurrent() << ",";
    logging_file << m_rightFollow2Motor.GetOutputCurrent();
    logging_file << "\n";
  }

  void TeleopInit() {
    logging_file.open("/home/lvuser/motor_output.csv");
    logging_file << "Time,SetPoint,LeftEncoder,RightEncoder,LeftSpeed,RightSpeed,LeftOutput,RightOutput,LeftLead,LeftFollow1,LeftFollow2,RightLead,RightFollow1,RightFollow2\n";
  }
  
  void DisabledInit() {
    logging_file.close();
  }

  void RobotInit() {
    InitDrives();
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

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if (p != kP) {
      m_leftPIDController.SetP(p);
      m_rightPIDController.SetP(p);
      kP = p;
    }
    if (i != kI) {
      m_leftPIDController.SetI(i);
      m_rightPIDController.SetI(i);
      kI = i;
    }
    if (d != kD) {
      m_leftPIDController.SetD(d);
      m_rightPIDController.SetD(d);
      kD = d;
    }
    if (iz != kIz) {
      m_leftPIDController.SetIZone(iz);
      m_rightPIDController.SetIZone(iz);
      kIz = iz;
    }
    if (ff != kFF) {
      m_leftPIDController.SetFF(ff);
      m_rightPIDController.SetFF(ff);
      kFF = ff;
    }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_leftPIDController.SetOutputRange(min, max); 
      m_rightPIDController.SetOutputRange(min, max); 
      kMinOutput = min;
      kMaxOutput = max; 
    }

    // read setpoint from joystick and scale by max rpm
    double SetPoint = MaxRPM * -m_stick.GetY();

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  rev::ControlType::kDutyCycle
     *  rev::ControlType::kPosition
     *  rev::ControlType::kVelocity
     *  rev::ControlType::kVoltage
     */
    
    m_leftPIDController.SetReference(SetPoint, rev::ControlType::kVelocity);
    m_rightPIDController.SetReference(SetPoint, rev::ControlType::kVelocity);

    frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
    frc::SmartDashboard::PutNumber("LeftProcessVariable", m_leftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("RightProcessVariable", m_rightEncoder.GetVelocity());

    LogOutput(SetPoint);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
