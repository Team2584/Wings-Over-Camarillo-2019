/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <iostream>
#include <frc/encoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalSource.h>
#include "frc/WPILib.h"
#include <stdio.h>
#include <memory>
#include <chrono>
#include <thread>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "robotIO.h"
#include "Constants.h"
#include "frc/Watchdog.h"
#include <string>

using namespace frc; //to avoid using frc:: in declarations
using namespace std; //Same as above except to avoid std::

  //Drive Motor Setup
  static const int leftLeadDeviceID = 1, rightLeadDeviceID = 3, leftFollowDeviceID = 2 , rightFollowDeviceID = 4;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  //INIT INPUTS CLASS
  robotIO* inputs = new robotIO;

  //INIT COMPRESSOR
  Compressor *c = new Compressor(0);

  string _sb;

//INIT HATCH MECH SOLENOID
  DoubleSolenoid hatchSolenoid {1, 2};

  //Elevator Setup
  static const int ElevatorOneID = 5;
  rev::CANSparkMax m_ElevatorOne{ElevatorOneID, rev::CANSparkMax::MotorType::kBrushless};

  //CREATE ELEVATOR PID
  rev::CANPIDController m_pidController = m_ElevatorOne.GetPIDController();

  // Encoder object created to display position values
  rev::CANEncoder m_encoder = m_ElevatorOne.GetEncoder();

  // PID coefficients
  double kP = 0.65, kI = 0.00005, kD = 0.05, kIz = 0, kFF = 0, kMaxOutput = 0.3, kMinOutput = -0.80;

//LOGITECH CAMERA INIT
static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(272, 204);
        camera.SetFPS(22);
    }

void Robot::RobotInit() {
  //SET ROLLER ID
  static const int RollerID = 1;

  //TURN ON COMPRESSOR
  c->SetClosedLoopControl(true);

  //CREATE LIMIT SWITCH
  limitSwitch = new DigitalInput(1);

  //Roller Setup
  Roller = new WPI_VictorSPX(RollerID);

  //SET FOLLOWER MOTORS FOR DRIVE
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  //Set PID coefficients
  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetIZone(kIz);
  m_pidController.SetFF(kFF);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  //CAMERA INIT
  std::thread visionThread(VisionThread);
  visionThread.detach();


  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
  Robot::TeleopPeriodic();
}

  //POSITION SETUP
  double pos;

  double rotations;

  int maxPos = 2;
  int minPos = 0;
  
  //ELEVATOR MANUAL CONTROL SETUP
  double elevatorManual;

  //Setup Drive Function
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  double rollerSpeed = 0;

void Robot::TeleopInit() {
  //SET STARTING VALUES
  int pos = 0;
  rotations = m_encoder.GetPosition();
  double elevateMax = 5; //Arbitrary value Ignore
  double elevateMin = 0; //Arbitrary value Ignore
}

void Robot::TeleopPeriodic() {

  //CREATE ELEVATOR PID
  if(pos < 2 && inputs->getButtonTriangle_P()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = 0;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 0;
  }
  else if(pos == 0 && inputs->getButtonTriangle_P()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = 25;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 1;
  }
  else if(pos == 1 && inputs->getButtonTriangle_P()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = 50;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 2;
  }
  else if(pos == 2 && inputs->getButtonX_P()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = 25;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 1;
  }
  else if(pos == 1 && inputs->getButtonX_P()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = 0;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 0;
  }
  else{
    //Else Elevator not Moving
    elevatorManual = 0;
    }

    //SETS DESIRED POSITION
    rotations = rotations + (elevatorManual);
    
    //WRITE DESIRED POSITION TO PID CONTROLLER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);

  //    //  ////////  //////////  ////////  //    //
  //    //  //    //      //      //        //    //
  ////////  ////////      //      //        ////////
  //    //  //    //      //      //        //    //
  //    //  //    //      //      ////////  //    //

  //HATCH SOLENOID FORWARD
  if(inputs->getShoulderRight_P()){
  hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  //HATCH SOLENOID REVERSE
  else if(inputs->getShoulderLeft_P()){
    hatchSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  ////////  ////////  //        //       //////// ////////
  //    //  //    //  //        //       //       //    //
  ///////   //    //  //        //       //////   ///////
  //  //    //    //  //        //       //       //  //
  //    //  ///////   ////////  //////// //////// //    //

  if(inputs->getShoulderRight() > 0.05){
    Roller->Set(ControlMode::PercentOutput, inputs->getShoulderRight());
  }

  else if(inputs->getShoulderLeft() > 0.05){
    Roller->Set(ControlMode::PercentOutput, -inputs->getShoulderLeft());    
  }

  else{
    //Passive Speed -10%
    rollerSpeed = -0.1;
    Roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }

  //Drive Setup
  m_robotDrive.ArcadeDrive(-(inputs->getLStickY()) * 0.85, inputs->getRStickX()*0.65);

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
