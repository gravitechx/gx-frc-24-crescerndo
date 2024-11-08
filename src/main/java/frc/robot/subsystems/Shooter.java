package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    CANSparkMax motor;
    CANSparkMax motor2; 

    public Shooter() {
        motor = new CANSparkMax(Constants.Shooter.kMotorPortTop, MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.Shooter.kMotorPortBottom, MotorType.kBrushless);
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooterspeed", motor.get());
    }

    public void spinOnly(double speed, double speed2) {
        motor.set(-speed);
        motor2.set(speed2);
    }

    public void spin(double speed, double speed2, Feeder feed, Intake intake) {
        motor.set(-speed);
        motor2.set(speed2);

        feed.spin((speed == 0) ? 0 : .75, intake);
    }


}
