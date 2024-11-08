package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Mode;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public Field2d field;
    public Orchestra music;

    public Swerve() {
        gyro = new AHRS(Port.kMXP);
        zeroGyro();
        field = new Field2d();
        music = new Orchestra();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants, this),
            new SwerveModule(1, Constants.Swerve.Mod1.constants, this),
            new SwerveModule(2, Constants.Swerve.Mod2.constants, this),
            new SwerveModule(3, Constants.Swerve.Mod3.constants, this)
        };

        music.loadMusic("music.chrp");

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics,
            getYaw(),
            getModulePositions()
            // new Pose2d(new Translation2d(), new Rotation2d(Units.degreesToRadians(180)))
        );

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::autoDrive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(2, 0, 0),
                new PIDConstants(5, 0, 0),
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.Swerve.swerveRadius,
                new ReplanningConfig() // true, true
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            SmartDashboard.putNumber("PathPlanner X", pose.getX());
            SmartDashboard.putNumber("PathPlanner Y", pose.getY());

        });

    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
        SmartDashboard.putBoolean("Connected", gyro.isConnected());

        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * Mode.getInstance().getSpeed() * Constants.topSpeed * Constants.stopSide, 
                                    translation.getY() * Mode.getInstance().getSpeed() * Constants.topSpeed, 
                                    rotation * Mode.getInstance().getSpeed() * Constants.topSpeed * Constants.stopSide, 
                                    Rotation2d.fromDegrees(Math.IEEEremainder(getYaw().getDegrees(), 360))
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * Mode.getInstance().getSpeed(), 
                                    translation.getY() * Mode.getInstance().getSpeed(), 
                                    rotation * Mode.getInstance().getSpeed())
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void autoDrive(ChassisSpeeds speed) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void stop() {
        SwerveModuleState[] modStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(modStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.reset();
        // gyro.setAngleAdjustment(174);
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(360 - gyro.getAngle());
        // return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getAngle()) : Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        // SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
        SmartDashboard.putBoolean("Calibrating", gyro.isCalibrating());
        swerveOdometry.update(getYaw(), getModulePositions());
        SmartDashboard.putData("Field", field);
        field.setRobotPose(swerveOdometry.getPoseMeters());

        SmartDashboard.putNumber("RobotX", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("RobotY", swerveOdometry.getPoseMeters().getY());


        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public void flipZeroGyro() {
        gyro.setAngleAdjustment(180);
        gyro.reset();
    }

    public void lock() {

    }
}