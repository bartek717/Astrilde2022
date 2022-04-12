package frc.robot.subsystems.Climber;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberImpl extends RepeatingPooledSubsystem implements Climber {
     
    // Declare the motor controllers.
    private WPI_TalonSRX primaryClimberMotorController;
    private WPI_TalonSRX followerClimberMotorController;
    private CANSparkMax shoulderMotorController;

    public ClimberImpl(WPI_TalonSRX primaryClimberMotorController, WPI_TalonSRX followerClimberMotorController, CANSparkMax shoulderMotorController) {
        super(20, TimeUnit.MILLISECONDS);
        this.primaryClimberMotorController = primaryClimberMotorController;
        this.followerClimberMotorController = followerClimberMotorController;
        this.shoulderMotorController = shoulderMotorController;
    }

    @Override
    public void extendElbow(double speed) {
        // Read the climber's position.
        this.primaryClimberMotorController.set(speed);
    }

    // The Neo motor controller.
    @Override
    public void extendShoulder(double speed) {
        this.shoulderMotorController.set(speed);
    }

    @Override
    public void resetClimberPosition(){
        this.shoulderMotorController.getEncoder().setPosition(0);
        this.primaryClimberMotorController.setSelectedSensorPosition(0);
        this.followerClimberMotorController.setSelectedSensorPosition(0);
    }

    @Override
    public void defineResources() {
        require(primaryClimberMotorController);
        require(followerClimberMotorController);
        require(shoulderMotorController);
    }

    @Override
    public void task() throws Exception {
        // TODO Auto-generated method stub
        double leftClimberMotorControllerPosition = primaryClimberMotorController.getSelectedSensorPosition();
        double rightClimberMotorControllerPosition = followerClimberMotorController.getSelectedSensorPosition();
        double NEOPosition = this.shoulderMotorController.getEncoder().getPosition();
        SmartDashboard.putNumber("LEFT CLIMBER ENCODER TICKS", leftClimberMotorControllerPosition);
        SmartDashboard.putNumber("RIGHT CLIMBER ENCODER TICKS", rightClimberMotorControllerPosition);
        SmartDashboard.putNumber("NEO CLIMBER ENCODER TICKS", NEOPosition);
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        // TODO Auto-generated method stub
        
    }

}
