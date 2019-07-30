/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class robotIO {
 public:
  robotIO();

/////////////////////////////////////////MAIN CONTROLLER VARS///////////////////////////////////////////////////

void joystickInit();

bool getButtonX(); //Button 1
bool getButtonCircle(); //Button 2
bool getButtonTriangle(); //Button 3
bool getButtonSquare(); //Button 0
bool getShoulderLeft(); //Button 5
bool getShoulderRight(); //Button 4
bool getButtonShare(); //Button 8
bool getButtonOptions(); //Button 9
bool getButtonPS(); //Button 12
bool getButtonPressLStick(); //Button 11
bool getButtonPressRStick(); //Button 10
bool getBigButton(); //Button 13
int getDPad(); //D-Pad
double getRStickY(); //Axis 2
double getRStickX(); //Axis 5
double getLStickY(); //Axis 1
double getLStickX(); //Axis 0
double getRTrigger(); //Axis 4
double getLTrigger(); //Axis 3

/////////////////////////////////////////PARTNER CONTROLLER VARS///////////////////////////////////////////////////

bool getButtonX_P(); //Button 1
bool getButtonCircle_P(); //Button 2
bool getButtonTriangle_P(); //Button 3
bool getButtonSquare_P(); //Button 0
bool getShoulderLeft_P(); //Button 5
bool getShoulderRight_P(); //Button 4
bool getButtonShare_P(); //Button 8
bool getButtonOptions_P(); //Button 9
bool getButtonPS_P(); //Button 12
bool getButtonPressLStick_P(); //Button 11
bool getButtonPressRStick_P(); //Button 10
bool getBigButton_P(); //Button 13
int getDPad_P(); //D-Pad
double getRStickY_P(); //Axis 2
double getRStickX_P(); //Axis 5
double getLStickY_P(); //Axis 1
double getLStickX_P(); //Axis 0
double getRTrigger_P(); //Axis 4
double getLTrigger_P(); //Axis 3
};
