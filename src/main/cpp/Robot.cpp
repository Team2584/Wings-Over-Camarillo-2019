/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Robot.h"
#include "Compressor.h"
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
#include <unistd.h>

using namespace frc;
using namespace std;

//INIT INPUTS CLASS
robotIO* inputs = new robotIO;


string _sb;

  //Drive Setup
  static const int leftLeadDeviceID = 2, rightLeadDeviceID = 4, leftFollowDeviceID = 3 , rightFollowDeviceID = 5;
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushless};

  //Elevator Setup
  static const int ElevatorOneID = 1;
  rev::CANSparkMax m_ElevatorOne{ElevatorOneID, rev::CANSparkMax::MotorType::kBrushless};

  //CREATE ELEVATOR PID
  rev::CANPIDController m_pidController = m_ElevatorOne.GetPIDController();

  // Encoder object created to display position values
  rev::CANEncoder m_encoder = m_ElevatorOne.GetEncoder();

  //INIT COMPRESSOR
  Compressor *c = new Compressor(0);

  //INIT HATCH MECH SOLENOID
  DoubleSolenoid hatchSolenoid {0, 1};

  // PID coefficients
  double kP = 0.1, kI = 0.0, kD = 0.25, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;

  PowerDistributionPanel *pdp = new PowerDistributionPanel();

  //LOGITECH CAMERA INIT
  static void VisionThread()
    {
        cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(272, 204);
        camera.SetFPS(22);
    }

void Robot::RobotInit() {

  static const int LeadID = 1, FollowID = 2, RollerID = 4;
  armLead = new WPI_TalonSRX(LeadID);
  armFollow = new WPI_VictorSPX(FollowID);
  roller = new WPI_VictorSPX(RollerID);
  //Set PID coefficients
  m_pidController.SetP(kP);
  m_pidController.SetI(kI);
  m_pidController.SetD(kD);
  m_pidController.SetIZone(kIz);
  m_pidController.SetFF(kFF);
  m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

  limitSwitch = new DigitalInput(1);

  //CAMERA INIT
  std::thread visionThread(VisionThread);
  visionThread.detach();

  //SET FOLLOWER MOTORS FOR DRIVE
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);
  
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
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

//POSITION SETUP
  double pos;
  int armPos;

  double rotations;

  int maxPos = 2;
  int minPos = 0;
  
  //ELEVATOR MANUAL CONTROL SETUP
  double elevatorManual;

  double stallCurrent;

  //Setup Drive Function
frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};
  double armSpeed;
  double rollerSpeed;
  double armPower;

  double armTarget;
  double armPosition;
  double armCurrent;
  double armManual;
  double elevatePos;

void Robot::TeleopInit() {

  int pos = 0; //Current elevator position
  armPos = 0;
  rotations = -60;
  double current;
  double stallCurrent = 100000;//SOME ARBITRARY NUMBER CHANGE LATER
  armTarget = armLead->GetSelectedSensorPosition(0);
  armPosition = armLead->GetSelectedSensorPosition(0);
}

void Robot::TeleopPeriodic() {
  elevatePos = m_encoder.GetPosition();

  armPosition = armLead->GetSelectedSensorPosition(0);

   c->SetClosedLoopControl(true);

  //GET ELEVATOR CURRENT
  double current = m_ElevatorOne.GetOutputCurrent	();

  /*if(inputs->getButtonX()){
    //Arm Speed Max
    armSpeed = 0.5;
    armLead->Set(ControlMode::PercentOutput, armSpeed);
  }

  else if(inputs->getButtonTriangle()){
    //Arm Speed Max Reversed
    armSpeed = -0.5;
    armLead->Set(ControlMode::PercentOutput, armSpeed);    
  }
  else{
    armSpeed = -0.05;
    armLead->Set(ControlMode::PercentOutput, armSpeed); 
  }*/

  



  if(inputs->getShoulderRight()){
    //Roller Speed Max
    rollerSpeed = 1;
    roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }

  else if(inputs->getShoulderLeft()){
    //Roller Speed Max Reversed
    rollerSpeed = -1;
    roller->Set(ControlMode::PercentOutput, rollerSpeed);    
  } 
  else{
    rollerSpeed = -0.1;
    roller->Set(ControlMode::PercentOutput, rollerSpeed);
  }


  //CREATE ELEVATOR PID
  if(pos == 0  && inputs->getButtonTriangle()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = -300;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 1;
  }
  else if(pos == 1 && inputs->getButtonTriangle()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = -495;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 2;
  }
  else if(pos == 2 && inputs->getButtonX()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = -285;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 1;
  }
  else if(pos == 1 && inputs->getButtonX()){
    //Ball Grab Position
    //Amount of NEO Rotations
    rotations = -60;//RANDOM NUMBER
    m_pidController.SetReference(rotations, rev::ControlType::kPosition);
    //Sets Current Shoulder Position Value
    pos = 0;
  }
  else if(limitSwitch->Get() == 0){
    rotations = (m_encoder.GetPosition()+1);
    pos = 2;
  }
  else if(current > stallCurrent){
    SmartDashboard::PutString("DB/String 0", "Elevator failure. Motor overstressed. Cutting power to elevator.");
    m_ElevatorOne.Set(0);
  }
  
  elevatorManual = inputs->getRStickY()*2;

  rotations = rotations + elevatorManual;

  m_pidController.SetReference(rotations, rev::ControlType::kPosition);
  
  //SETS DESIRED POSITION
  // rotations = rotations + elevatorManual;
    
  //WRITE DESIRED POSITION TO PID CONTROLLER
  if(inputs->getLStickY() < -0.1){
    armPower = inputs->getLStickY();
  }
  else if(inputs->getLStickY() > 0.1){
    armPower = inputs->getLStickY()*0.25;
  }
  else{
    armPower = -0.1;
  }
  armLead->Set(ControlMode::PercentOutput, armPower);
  
  

  /*if(armPos == 0 && inputs->getButtonX()){
    armTarget = 900;
    armPos = 1;
  }
  else if(armPos == 1 && inputs->getButtonX()){
    armTarget = 1600;
    armPos = 2;
  }
  else if(armPos == 2 && inputs->getButtonTriangle()){
    armTarget = 900;
    armPos = 1;
  }
  else if(armPos == 1 && inputs->getButtonTriangle()){
    armTarget = 0;
    armPos = 0;
  }

  armManual = inputs->getLStickY()*10;

  armTarget = armTarget + armManual;

  if(armTarget != armPosition){
    armCurrent = (armTarget - armPosition) * 0.02;
    armLead->Set(ControlMode::Current, armCurrent);
  }
  else{
    armCurrent = 0.0001;
    armLead->Set(ControlMode::Current, armCurrent);
  }*/

  //FAILSAFE CURRENT LIMITER
  
  

  //HATCH SOLENOID FORWARD
  if(inputs->getShoulderRight_P()){
  hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  //HATCH SOLENOID REVERSE
  else if(inputs->getShoulderLeft_P()){
    hatchSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else{
    hatchSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  }
  
  SmartDashboard::PutNumber("ElevatorHeight", m_encoder.GetPosition());
  SmartDashboard::PutNumber("ElevatorTarget", rotations);
  SmartDashboard::PutNumber("ElevatorCurrentDraw", current);
  SmartDashboard::PutNumber("Position", pos);
  SmartDashboard::PutNumber("Arm Position", armLead->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Arm Current", armCurrent);

    //dPad Steer
  if(inputs->getDPad_P() == 90){
    m_robotDrive.ArcadeDrive(0, .2);
  }
  else if(inputs->getDPad_P() == 270){
    m_robotDrive.ArcadeDrive(0, -.2);
  }
  else if(inputs->getDPad_P() == 0){
    m_robotDrive.ArcadeDrive(.3,0);
  }
  else if(inputs->getDPad_P() == 180){
    m_robotDrive.ArcadeDrive(-.3,0);
  }
  else{
  m_robotDrive.ArcadeDrive(-(inputs->getLStickY_P()) * 0.85, inputs->getRStickX_P()*0.65);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
