package frc.robot.subsystems.Climber;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Timer;

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

    // The outer climber refers to the climber arms that the Talons control.
    @Override
    public void extendOuterClimber(double speed) {
        // Read the climber's position.
        double leftClimberMotorControllerPosition = primaryClimberMotorController.getSelectedSensorPosition();
        double rightClimberMotorControllerPosition = followerClimberMotorController.getSelectedSensorPosition();

        // Ensure that the time entered is within Endgame to avoid potential accidents.
        if (Timer.getMatchTime() > 120) {
            this.primaryClimberMotorController.set(speed); 

            if (rightClimberMotorControllerPosition > 0 || leftClimberMotorControllerPosition > 0) {
                this.primaryClimberMotorController.set(0);
            }
        }

        // long now = System.nanoTime();
        // if(this.starttime < 0) this.starttime = now;
        //     this.shoulderMotorController.set(1); // test value
        // if(now > this.starttime + TimeUnit.SECONDS.toNanos(5)) this.shoulderMotorController.set(0);
    }

    // Possibly unnecessary.
    @Override
    public void retractOuterClimber(double speed) {
        if (Timer.getMatchTime() > 120) {
            this.primaryClimberMotorController.set(-speed);
            this.followerClimberMotorController.set(-speed);
        }

        // long now = System.nanoTime();
        // if(this.starttime < 0) this.starttime = now;
        // this.shoulderMotorController.set(1); // test value
        // if(now > this.starttime + TimeUnit.SECONDS.toNanos(5)) this.shoulderMotorController.set(0);
    }

    @Override
    public void angleOuter(double angle) {}

    // The Neo motor controller.
    @Override
    public void extendShoulderLifter(double speed) {
        if (Timer.getMatchTime() > 120) {
            this.shoulderMotorController.set(speed);

            if (shoulderMotorController.getEncoder().getPosition() > 0) {  // Utilize some value in place of 0. Test this by logging the encoder position values to the SmartDashboard and set an appropriate value.
                this.shoulderMotorController.set(0);
            }
        }
    }

    // Possibly unnecessary.
    @Override
    public void retractInnerLifter(double speed) {
        if (Timer.getMatchTime() > 120) {
            this.shoulderMotorController.set(-speed);
        }
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
        
    }

    @Override
    public void lifecycleStatusChanged(LifecycleEvent previous, LifecycleEvent current) {
        // TODO Auto-generated method stub
        
    }

}
