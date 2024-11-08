package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public final class CTREConfigs {
    public com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        // CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimitEnable(Constants.Swerve.angleEnableCurrentLimit)
        //     .withSupplyCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
        //     .withSupplyTimeThreshold(Constants.Swerve.anglePeakCurrentDuration)
        //     .withSupplyCurrentThreshold(Constants.Swerve.anglePeakCurrentLimit);

        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(Constants.Swerve.driveEnableCurrentLimit)
            .withSupplyCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
            .withSupplyTimeThreshold(Constants.Swerve.drivePeakCurrentDuration)
            .withSupplyCurrentThreshold(Constants.Swerve.drivePeakCurrentLimit);

        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.driveEnableCurrentLimit, 
        //     Constants.Swerve.driveContinuousCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
        // swerveDriveFXConfig.Slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        swerveDriveFXConfig.OpenLoopRamps = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        // swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
