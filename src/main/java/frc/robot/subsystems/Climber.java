// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax leftClimbMotor;
  CANSparkMax rightClimbMotor;

  public Climber() {
    leftClimbMotor = new CANSparkMax(Constants.Climber.motorPort1, MotorType.kBrushed);
    rightClimbMotor = new CANSparkMax(Constants.Climber.motorPort2, MotorType.kBrushed);

    leftClimbMotor.setIdleMode(IdleMode.kBrake);
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     SmartDashboard.putNumber("LeftClimberSpeed", leftClimbMotor.get());
    SmartDashboard.putNumber("RightClimberSpeed", rightClimbMotor.get());
  }

  public void spinLeft(double speed) {
    leftClimbMotor.set(speed);

  }

  public void spinRight(double speed) {
    rightClimbMotor.set(speed);
  }

}
