/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "frc/WPILib.h"
#include "rev/CANSparkMax.h"
#include "robotIO.h"

using namespace frc;
using namespace std;

/////////////////////////////////////////JOYSTICK SETUP/////////////////////////////////////////////////////////

Joystick * m_stick = new Joystick(1);
Joystick * m_partner = new Joystick(2);

/////////////////////////////////////////MAIN CONTROLLER VARS///////////////////////////////////////////////////

bool buttonX; //Button 1
bool buttonCircle; //Button 2
bool buttonTriangle; //Button 3
bool buttonSquare; //Button 0
bool shoulderLeft; //Button 5
bool shoulderRight; //Button 4
bool buttonShare; //Button 8
bool buttonOptions; //Button 9
bool buttonPS; //Button 12
bool buttonPressLStick; //Button 11
bool buttonPressRStick; //Button 10
bool bigButton; //Button 13
int dPad; //D-Pad
double rStickY; //Axis 2
double rStickX; //Axis 5
double lStickY; //Axis 1
double lStickX; //Axis 0
double rTrigger; //Axis 4
double lTrigger; //Axis 3

/////////////////////////////////////////PARTNER CONTROLLER VARS///////////////////////////////////////////////////

bool buttonX_P; //Button 1
bool buttonCircle_P; //Button 2
bool buttonTriangle_P; //Button 3
bool buttonSquare_P; //Button 0
bool shoulderLeft_P; //Button 5
bool shoulderRight_P; //Button 4
bool buttonShare_P; //Button 8
bool buttonOptions_P; //Button 9
bool buttonPS_P; //Button 12
bool buttonPressLStick_P; //Button 11
bool buttonPressRStick_P; //Button 10
bool bigButton_P; //Button 13
int dPad_P; //D-Pad
double rStickY_P; //Axis 2
double rStickX_P; //Axis 5
double lStickY_P; //Axis 1
double lStickX_P; //Axis 0
double rTrigger_P; //Axis 4
double lTrigger_P; //Axis 3

/////////////////////////////////////////CONTROLLS SETUP///////////////////////////////////////////////////

robotIO::robotIO() {
    Joystick * m_stick = new Joystick(3);
    Joystick * m_partner = new Joystick(4);
}

bool robotIO::getButtonX(){
    buttonX = m_stick->GetRawButton(1);
    return buttonX;
}

bool robotIO::getButtonCircle(){
    buttonCircle = m_stick->GetRawButton(2);
    return buttonCircle;
}

bool robotIO::getButtonTriangle(){
    buttonTriangle = m_stick->GetRawButton(3);
    return buttonTriangle;
}

bool robotIO::getButtonSquare(){
    buttonSquare = m_stick->GetRawButton(0);
    return buttonSquare;
}

bool robotIO::getButtonOptions(){
    buttonOptions = m_stick->GetRawButton(9);
    return buttonOptions;
}

bool robotIO::getButtonShare(){
    buttonShare = m_stick->GetRawButton(8);
    return buttonShare;
}

bool robotIO::getShoulderRight(){
    shoulderRight = m_stick->GetRawButton(5);
    return shoulderRight;
}

bool robotIO::getShoulderLeft(){
    shoulderLeft = m_stick->GetRawButton(4);
    return shoulderLeft;
}

bool robotIO::getBigButton(){
    bigButton = m_stick->GetRawButton(13);
    return bigButton;
}

bool robotIO::getButtonPressLStick(){
    buttonPressLStick = m_stick->GetRawButton(10);
    return buttonPressLStick;
}

bool robotIO::getButtonPressRStick(){
    buttonPressRStick = m_stick->GetRawButton(11);
    return buttonPressRStick;
}

int robotIO::getDPad(){
    dPad = m_stick->GetPOV();
    return dPad;
}

double robotIO::getLStickX(){
    lStickX = m_stick->GetRawAxis(0);
    return lStickX;
}

double robotIO::getLStickY(){
    lStickY = m_stick->GetRawAxis(1);
    return lStickY;
}

double robotIO::getRStickX(){
    rStickX = m_stick->GetRawAxis(5);
    return rStickX;
}

double robotIO::getRStickY(){
    rStickY = m_stick->GetRawAxis(2);
    return rStickY;
}

////////////////////////////////////////PARTNER CONTROLLS SETUP///////////////////////////////////////////////////

bool robotIO::getButtonX_P(){
    buttonX_P = m_partner->GetRawButtonPressed(1);
    return buttonX_P;
}

bool robotIO::getButtonCircle_P(){
    buttonCircle_P = m_partner->GetRawButtonPressed(2);
    return buttonCircle_P;
}

bool robotIO::getButtonTriangle_P(){
    buttonTriangle_P = m_partner->GetRawButtonPressed(3);
    return buttonTriangle_P;
}

bool robotIO::getButtonSquare_P(){
    buttonSquare_P = m_partner->GetRawButtonPressed(0);
    return buttonSquare_P;
}

bool robotIO::getButtonOptions_P(){
    buttonOptions_P = m_partner->GetRawButton(9);
    return buttonOptions_P;
}

bool robotIO::getButtonShare_P(){
    buttonShare_P = m_partner->GetRawButton(8);
    return buttonShare_P;
}

bool robotIO::getShoulderRight_P(){
    shoulderRight_P = m_partner->GetRawButtonPressed(5);
    return shoulderRight_P;
}

bool robotIO::getShoulderLeft_P(){
    shoulderLeft_P = m_partner->GetRawButtonPressed(4);
    return shoulderLeft_P;
}

bool robotIO::getBigButton_P(){
    bigButton_P = m_partner->GetRawButton(13);
    return bigButton_P;
}

bool robotIO::getButtonPressLStick_P(){
    buttonPressLStick_P = m_partner->GetRawButton(10);
    return buttonPressLStick_P;
}

bool robotIO::getButtonPressRStick_P(){
    buttonPressRStick_P = m_partner->GetRawButton(11);
    return buttonPressRStick_P;
}

int robotIO::getDPad_P(){
    dPad_P = m_partner->GetPOV();
    return dPad_P;
}

double robotIO::getLStickX_P(){
    lStickX_P = m_partner->GetRawAxis(0);
    return lStickX_P;
}

double robotIO::getLStickY_P(){
    lStickY_P = m_partner->GetRawAxis(1);
    return lStickY_P;
}

double robotIO::getRStickX_P(){
    rStickX_P = m_partner->GetRawAxis(5);
    return rStickX_P;
}

double robotIO::getRStickY_P(){
    rStickY_P = m_partner->GetRawAxis(2);
    return rStickY_P;
}


