// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.auto.modes.DepTwoGrabOne;
//import frc.robot.auto.modes.ScoreTwo;
//import frc.robot.auto.modes.StraightToDepo;
//import frc.robot.auto.modes.TestAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Instantiating our robot subsystem
  private final Shooter shooter = new Shooter();
  private final Feeder feed = new Feeder();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Limelight limelight = new Limelight(feed, intake);
  private final Lights lights = new Lights();

  private final Joystick controller = new Joystick(Constants.OI.controllerPort);
  private final Joystick buttons = new Joystick(Constants.OI.buttonPanelPort);

  private final JoystickButton testNormalButton = new JoystickButton(buttons, Constants.OI.testNormalButton);
  private final JoystickButton testReverseButton = new JoystickButton(buttons, Constants.OI.testReverseButton);
  private final JoystickButton rightClimberUpButtonPanel = new JoystickButton(buttons, Constants.OI.rightClimberUpButtonPanel);
  private final JoystickButton rightClimberDownButtonPanel = new JoystickButton(buttons, Constants.OI.rightClimberDownButtonPanel);
  private final JoystickButton leftClimberUpButtonPanel = new JoystickButton(buttons, Constants.OI.leftClimberUpButtonPanel);
  private final JoystickButton leftClimberDownButtonPanel = new JoystickButton(buttons, Constants.OI.leftClimberDownButtonPanel);
 
  private final JoystickButton zeroGyro = new JoystickButton(controller, Constants.OI.resetGyro);
 
  private final JoystickButton intakeButton = new JoystickButton(controller, Constants.OI.intakeButton);
  private final JoystickButton feedButton = new JoystickButton(controller, Constants.OI.feedButton);
  private final JoystickButton shootButton = new JoystickButton(controller, Constants.OI.shootButton);
  private final JoystickButton rightClimberUpButton = new JoystickButton(controller, Constants.OI.leftClimberUpButton);
  private final JoystickButton rightClimberDownButton = new JoystickButton(controller, Constants.OI.leftClimberDownButton);
  private final JoystickButton leftClimberUpButton = new JoystickButton(controller, Constants.OI.rightClimberUpButton);
  private final JoystickButton leftClimberDownButton = new JoystickButton(controller, Constants.OI.rightClimberDownButton);

  private final JoystickButton increaseM1Button = new JoystickButton(buttons, 7);
  private final JoystickButton decreaseM1Button = new JoystickButton(buttons, 8);
  private final JoystickButton increaseM2Button = new JoystickButton(buttons, 9);
  private final JoystickButton decreaseM2Button = new JoystickButton(buttons, 10);
  private final JoystickButton shootOverRobotButton= new JoystickButton(controller, 5);
  
  private final JoystickButton reverseFeedButton = new JoystickButton(controller, Constants.OI.reverseFeed);
  
  private final Trigger trigger = new Trigger(() -> controller.getRawAxis(3) == 1);
  private final Trigger trigger2 = new Trigger(() -> controller.getRawAxis(3) == -1);

  // Swerve stuff
  private final Swerve s_Swerve = new Swerve();
  private final Mode modeControl = Mode.getInstance();

  private double motor1Speed = 1;
  private double motor2Speed = 1;
  // ^^ Shooter motor variables that are changed on the fly

  public RobotContainer() {
    configureAutoCommands();

    s_Swerve.setDefaultCommand(
      new TeleopSwerve(
        s_Swerve, 
        () -> -controller.getRawAxis(Constants.OI.translationAxis),
        () -> -controller.getRawAxis(Constants.OI.strafeAxis),
        () -> -controller.getRawAxis(Constants.OI.rotationAxis),
        () -> false
      )
    );

    configureBindings();
  }

  private void configureAutoCommands() {
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> intake.spin(.7)));
    NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.spin(0)));
    NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooter.spinOnly(.9, .9)));
    NamedCommands.registerCommand("StopShoot", new InstantCommand(() -> shooter.spinOnly(0, 0)));
    NamedCommands.registerCommand("Feed", new InstantCommand(() -> feed.spin(.7, intake)));
    NamedCommands.registerCommand("StopFeed", new InstantCommand(() -> feed.spin(0, intake)));
    NamedCommands.registerCommand("OnlyFeed", new InstantCommand(() -> feed.spinOnly(.7)));
    NamedCommands.registerCommand("StopOnlyFeed", new InstantCommand(() -> feed.spinOnly(0)));
    NamedCommands.registerCommand("Prep", new InstantCommand(() -> Constants.primeShooter = true));
    NamedCommands.registerCommand("StopPrep", new InstantCommand(() -> Constants.primeShooter = false));
    NamedCommands.registerCommand("FlipGyro", new InstantCommand(() -> s_Swerve.flipZeroGyro()));
  }

  private void configureBindings() {
    shootButton.onFalse(new InstantCommand(() -> shooter.spinOnly(.08, .35))); // speed:.06, speed3: 0.31

    shootOverRobotButton.onTrue(new InstantCommand(() ->shooter.spinOnly(.98,.32)));
    shootOverRobotButton.onFalse(new InstantCommand(() ->shooter.spinOnly(0.06,0.31)));

    reverseFeedButton.onTrue(new InstantCommand(() -> feed.spin(-.5, intake)));
    reverseFeedButton.onFalse(new InstantCommand(() -> feed.spin(0, intake)));
    reverseFeedButton.onTrue(new InstantCommand(() -> shooter.spinOnly(-.5, -.5)));
    reverseFeedButton.onFalse(new InstantCommand(() -> shooter.spinOnly(.08, .35)));

    intakeButton.onTrue(new InstantCommand(() -> intake.spin(1)));
    intakeButton.onFalse(new InstantCommand(() -> intake.spin(0)));
    feedButton.onTrue(new InstantCommand(() -> feed.spin(.3, intake)));
    feedButton.onFalse(new InstantCommand(() -> feed.spin(0, intake)));

    shootButton.onTrue(new InstantCommand(() -> shooter.spinOnly(motor1Speed, motor2Speed)));

    leftClimberUpButton.onTrue(new InstantCommand(() -> climber.spinLeft(1)));
    leftClimberDownButton.onTrue(new InstantCommand(() -> climber.spinLeft(-1)));
    rightClimberUpButton.onTrue(new InstantCommand(() -> climber.spinRight(1)));
    rightClimberDownButton.onTrue(new InstantCommand(() -> climber.spinRight(-1)));
    leftClimberUpButton.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    leftClimberDownButton.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    rightClimberUpButton.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    rightClimberDownButton.onFalse(new InstantCommand(() -> climber.spinRight(0)));

    trigger.onTrue(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    trigger.onFalse(new InstantCommand(() -> Lights.setColor(0, 0, 255)));
    trigger2.onTrue(new InstantCommand(() -> Lights.setColor(0, 255, 0)));
    trigger2.onFalse(new InstantCommand(() -> Lights.setColor(0, 0, 255)));

    // intakeButton.onTrue(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    feedButton.onFalse(new InstantCommand(() -> Lights.setColor(255, 0, 0)));
    // shootButton.onTrue(new InstantCommand(() -> Lights.setColor(0, 0, 255)));

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
 
    increaseM1Button.onTrue(new InstantCommand(() -> motor1Speed += .01));
    increaseM1Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m1Speed", motor1Speed)));
    decreaseM1Button.onTrue(new InstantCommand(() -> motor1Speed -= .01));
    decreaseM1Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m1Speed", motor1Speed)));
    increaseM2Button.onTrue(new InstantCommand(() -> motor2Speed += .01));
    increaseM2Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m2Speed", motor2Speed)));
    decreaseM2Button.onTrue(new InstantCommand(() -> motor2Speed -= .01));
    decreaseM2Button.onFalse(new InstantCommand(() -> SmartDashboard.putNumber("m2Speed", motor2Speed)));

    testNormalButton.onTrue(new InstantCommand(() -> shooter.spin(1, 1, feed, intake)));
    testNormalButton.onFalse(new InstantCommand(() -> shooter.spin(0, 0, feed, intake)));
    testReverseButton.onTrue(new InstantCommand(() -> shooter.spin(-1, -1, feed, intake)));
    testReverseButton.onFalse(new InstantCommand(() -> shooter.spin(0, 0, feed, intake)));

    rightClimberUpButtonPanel.onTrue(new InstantCommand(() -> climber.spinRight(-1)));
    rightClimberDownButtonPanel.onTrue(new InstantCommand(() -> climber.spinRight(1)));
    leftClimberUpButtonPanel.onTrue(new InstantCommand(() -> climber.spinLeft(1)));
    leftClimberDownButtonPanel.onTrue(new InstantCommand(() -> climber.spinLeft(-1)));
    rightClimberUpButtonPanel.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    rightClimberDownButtonPanel.onFalse(new InstantCommand(() -> climber.spinRight(0)));
    leftClimberUpButtonPanel.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
    leftClimberDownButtonPanel.onFalse(new InstantCommand(() -> climber.spinLeft(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoBuilder.buildAuto("Blue4NoteE");
    // s_Swerve.resetOdometry(PathPlannerPath.fromChoreoTrajectory("NewPath").getPreviewStartingHolonomicPose());
    // return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("NewPath"));
  }
}

