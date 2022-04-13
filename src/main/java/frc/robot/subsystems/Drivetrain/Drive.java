package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;

import ca.team3161.lib.robot.LifecycleListener;
import ca.team3161.lib.robot.subsystem.Subsystem;
import edu.wpi.first.math.Pair;

public interface Drive extends Subsystem, LifecycleListener {
    void drive(double forward, double rotation);
    void resetEncoderTicks();
    Pair<Double, Double> getEncoderTicks();
    CANSparkMax getLeftSide();
    CANSparkMax getRightSide();
}

