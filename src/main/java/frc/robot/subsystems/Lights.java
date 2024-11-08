// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  private static AddressableLED lights;
  private static AddressableLEDBuffer buffer;

  /** Creates a new Lights. */
  public Lights() {
    lights = new AddressableLED(Constants.Lights.port);
    buffer = new AddressableLEDBuffer(Constants.Lights.length);

    lights.setLength(buffer.getLength());
    // setColor(0, 255, 62);
    setColor(0, 0, 255);
  }

  @Override
  public void periodic() {}

  public static void setColor(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }

    lights.setData(buffer);
    lights.start();
  }

  public static void setSpecific(int start, int end, int r, int g, int b) {
    for (int i = start; i <= end; i++) {
      buffer.setRGB(i, r, g, b);
    }

    lights.setData(buffer);
    lights.start();
  }
}
