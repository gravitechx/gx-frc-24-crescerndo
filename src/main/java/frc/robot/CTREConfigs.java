// package frc.robot;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;

// public final class CTREConfigs {
//     public TalonFXConfiguration swerveAngleFXConfig;
//     public TalonFXConfiguration swerveDriveFXConfig;
//     public CANcoderConfiguration swerveCanCoderConfig;

//     public CTREConfigs(){
//         swerveAngleFXConfig = new TalonFXConfiguration();
//         swerveDriveFXConfig = new TalonFXConfiguration();
//         swerveCanCoderConfig = new CANcoderConfiguration();

//         /* Swerve Angle Motor Configurations */
//         CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs()
//             .withSupplyCurrentLimitEnable(Constants.Swerve.angleEnableCurrentLimit)
//             .withSupplyCurrentLimit(Constants.Swerve.angleCurrentLimit)
//             .withSupplyTimeThreshold(Constants.Swerve.angleCurrentThresholdTime)
//             .withSupplyCurrentThreshold(Constants.Swerve.angleCurrentThreshold);

//         // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
//         //     Constants.Swerve.angleEnableCurrentLimit, 
//         //     Constants.Swerve.angleContinuousCurrentLimit, 
//         //     Constants.Swerve.anglePeakCurrentLimit, 
//         //     Constants.Swerve.anglePeakCurrentDuration);

//         swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
//         swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
//         swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
//         // swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
//         // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
//         swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;
//         swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
//         swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

//         /* Swerve Drive Motor Configuration */
//         CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs()
//             .withSupplyCurrentLimitEnable(Constants.Swerve.driveEnableCurrentLimit)
//             .withSupplyCurrentLimit(Constants.Swerve.driveCurrentLimit)
//             .withSupplyTimeThreshold(Constants.Swerve.driveCurrentThresholdTime)
//             .withSupplyCurrentThreshold(Constants.Swerve.driveCurrentThreshold);

//         // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
//         //     Constants.Swerve.driveEnableCurrentLimit, 
//         //     Constants.Swerve.driveContinuousCurrentLimit, 
//         //     Constants.Swerve.drivePeakCurrentLimit, 
//         //     Constants.Swerve.drivePeakCurrentDuration);

//         swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
//         swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
//         swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
//         // swerveDriveFXConfig.Slot0.kF = Constants.Swerve.driveKF;
//         swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
//         swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
//         swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

//         swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
//         swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;

//         swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
        
//         /* Swerve CANCoder Configuration */
//         // swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
//         swerveCanCoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert;

//         // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
//         // swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
//         // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//         // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
//     }
// }

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.canCoderInvert;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    }
}