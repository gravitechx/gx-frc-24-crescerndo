// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry tz;
  NetworkTableEntry ta;

  double x;
  double y;
  double z;
  double area;
  double[] limelightTable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);  

  Feeder feeder;
  Intake intake;
  boolean a = false;
  boolean left = false;
  boolean right = false;
  boolean middle = false;
  boolean gone = false;
  
  /** Creates a new Limelight. */
  public Limelight(Feeder feeder, Intake intake) {
    this.feeder = feeder;
    this.intake = intake;

    // tx = table.getEntry("tx");
    // ty = table.getEntry("ty");
    // tz = table.getEntry("tz");
    // ta = table.getEntry("ta");
 
  }

  @Override
  public void periodic() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);  

    //post to smart dashboard periodically
    SmartDashboard.putNumber("Limelight0", limelightTable[0]);
    SmartDashboard.putNumber("Limelight1", limelightTable[1]);
    SmartDashboard.putNumber("Limelight2", limelightTable[2]);
    SmartDashboard.putNumber("Limelight3", limelightTable[3]);
    SmartDashboard.putNumber("Limelight4", limelightTable[4]);
    SmartDashboard.putNumber("Limelight5", limelightTable[5]);

    if (Constants.state == 0) {
      autoPos();    
    } else {
      if (Math.abs(limelightTable[0]) < .7 && Math.abs(limelightTable[0]) != 0) {
        if (Constants.primeShooter) {
          a = true;
          feeder.spin(.7, intake);

          if (Constants.state == 1) {
            Lights.setColor(0, 255, 0);
          }
        }
      } else {
        if (a && Constants.primeShooter) {
          a = false;
          feeder.spinOnly(0);

          if (Constants.state == 1) {
            Lights.setColor(0, 0, 255);
          }
        }
      }
    }

    // if (limelightTable[0] > -.1 && limelightTable[0] < .2 && Math.abs(limelightTable[0]) != 0) {
    // if (Math.abs(limelightTable[0]) != 0) {
    
  }

  public void autoPos() {
    // if (limelightTable[0] == 0 && !gone) {
    //   right = false;
    //   left = false;
    //   middle = false;
    //   gone = true;

    //   Lights.setColor(255, 0, 0);
    // } else {
    //   if (limelightTable[0] < -.01 && !left) {
    //     left = true;
    //     right = false;
    //     middle = false;
    //     gone = false;

    //     Lights.setColor(255, 0, 0);
    //     Lights.setSpecific(2, Constants.Lights.length, 0, 0, 255);
    //   } else if (limelightTable[0] > .01 && !right) {
    //     right = true;
    //     left = false;
    //     middle = false;
    //     gone = false;

    //     Lights.setColor(255, 0, 0);
    //     Lights.setSpecific(0, 24, 0, 0, 255);
    //   } else if (limelightTable[0] > -.01 && limelightTable[0] < -.01 && !middle) {
    //     right = false;
    //     left = false;
    //     middle = true;
    //     gone = false;

    //     Lights.setColor(0, 255, 0);
    //   }
    // }
  }
}
