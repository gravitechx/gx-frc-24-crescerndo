package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Mode {
  private int mode; // 0 Field, 1 Test, 2 Populated
  private double maxSpeed;
  private static Mode singleton;
  private double robotSpeed;

  public Mode() {
    mode = 0;
    maxSpeed = 1;
    robotSpeed = 1;
  }

  public void changeMode() {
    SmartDashboard.putBoolean("modeChanged", true);

    if (this.mode == 2) {
      this.mode = 0;
    } else {
      this.mode++;
    }

    if (mode == 0) {
      SmartDashboard.putString("Mode", "Field");
      robotSpeed = 1;
    } else if (mode == 1) {
      SmartDashboard.putString("Mode", "Test");
      robotSpeed = .25;
    } else if (mode == 2) {
      SmartDashboard.putString("Mode", "Event");
      robotSpeed = .1;
    }

    SmartDashboard.putNumber("robotSpeed", robotSpeed);
  }

  public void changeSpeed(double speed) {
    maxSpeed = speed * robotSpeed;

    SmartDashboard.putNumber("Top Speed", maxSpeed);
  }

  public int getMode() {
    return mode;
  }

  public double getSpeed() {
    return maxSpeed;
  }

  public static Mode getInstance() {
    if (singleton == null) singleton = new Mode();
    return singleton;
  }
}
