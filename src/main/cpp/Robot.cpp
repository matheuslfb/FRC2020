/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include "frc/WPILib.h"
#include <frc/util/Color.h>

#include <rev/ColorMatch.h>
#include <rev/ColorSensorV3.h>

using namespace frc;
using namespace rev;

WPI_TalonSRX *FL = new WPI_TalonSRX{3};
WPI_TalonSRX *RL = new WPI_TalonSRX(4);
WPI_TalonSRX *RR = new WPI_TalonSRX{1};
WPI_TalonSRX *FR = new WPI_TalonSRX(2);

Joystick *stickMain = new Joystick(0);
Joystick *stickRot = new Joystick(1);
Joystick *stickXbox = new Joystick(2);

PWM *gyroRate = new PWM(0);
PWM *gyroTemp = new PWM(1);

// ADXRS450_Gyro *gyro = new ADXRS450_Gyro(SPI::Port::kOnboardCS0);

float gyroAngle;

DifferentialDrive *driveTrain = new DifferentialDrive(*FL, *RL);

static constexpr auto i2cport = frc::I2C::Port::kOnboard;

ColorSensorV3 m_colorSensor{i2cport};
ColorMatch m_colorMatcher;

static constexpr Color kBlueTarget = Color(0.143, 0.427, 0.429);
static constexpr Color kGreeTarget = Color(0.197, 0.561, 0.240);
static constexpr Color kRedTarget = Color(0.561, 0.232, 0.114);
static constexpr Color kYellowTarget = Color(0.361, 0.524, 0.113);

float driveFoward, driveRot;

void Robot::RobotInit() {
 m_colorMatcher.AddColorMatch(kBlueTarget);
 m_colorMatcher.AddColorMatch(kGreeTarget);
 m_colorMatcher.AddColorMatch(kRedTarget);
 m_colorMatcher.AddColorMatch(kYellowTarget);

//  gyro->Reset();

//  gyro->Calibrate();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  Color detectedColor = m_colorSensor.GetColor();

  // Run the color match algorithm on our detected color
  std:: string colorString;
  double confidence = 0.0;
  Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

  if(matchedColor == kBlueTarget){
    colorString = "Blue";
  } else if (matchedColor == kRedTarget) {
    colorString = "Red";
  } else if (matchedColor == kGreeTarget) {
    colorString = "Green";
  } else if (matchedColor == kYellowTarget) {
    colorString = "Yellow";
  } else {
    colorString = "Unknown";
  }

  // gyroAngle= gyro->GetAngle();
  printf("Angle: %f", gyroAngle);


  // Open smart dashboard or shuffleboard to see the color detected by the sensor

  SmartDashboard::PutNumber("Red", detectedColor.red);
  SmartDashboard::PutNumber("Gree", detectedColor.green);
  SmartDashboard::PutNumber("Blue", detectedColor.blue);
  SmartDashboard::PutNumber("Confidence", confidence);
  SmartDashboard::PutString("Detected Color", colorString);

  uint32_t proximity = m_colorSensor.GetProximity();

  SmartDashboard::PutNumber("Proximity", proximity);

}



/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  driveFoward = stickMain->GetY();
  driveRot = stickRot->GetX();

  driveTrain->ArcadeDrive(driveFoward, driveRot, true);
  Wait(0.01);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
