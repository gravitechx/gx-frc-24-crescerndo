// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  CANSparkMax leftFeederMotor;
  CANSparkMax rightFeederMotor;
  AnalogInput sensor;

  /** Creates a new Feeder. */
  public Feeder() {
    leftFeederMotor = new CANSparkMax(Constants.Feeder.leftMotorPort, MotorType.kBrushless);
    rightFeederMotor = new CANSparkMax(Constants.Feeder.rightMotorPort, MotorType.kBrushless);
    sensor = new AnalogInput(Constants.Feeder.sensorPort);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FrontFeederSpeed", leftFeederMotor.get());
    SmartDashboard.putNumber("FrontFeederSpeed", rightFeederMotor.get());
    SmartDashboard.putNumber("Sensor3Value", sensor.getValue());
  } 

  public void spin(double speed, Intake intake) {
    intake.spin(speed);
    rightFeederMotor.set(speed);
    leftFeederMotor.set(-speed);
  }

  public void spinOnly(double speed) {
    rightFeederMotor.set(speed);
    leftFeederMotor.set(-speed);
  }
}
