package frc.robot.subsystems.Climber;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import ca.team3161.lib.robot.LifecycleEvent;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberImpl extends RepeatingPooledSubsystem implements Climber {
     
    // Declare the motor controllers.
    private WPI_TalonSRX primaryClimberMotorController;
    private WPI_TalonSRX followerClimberMotorController;
    private CANSparkMax shoulderMotorController;
    private boolean climberDeployed = false;
    private boolean innerUp = false;

    public ClimberImpl(WPI_TalonSRX primaryClimberMotorController, WPI_TalonSRX followerClimberMotorController, CANSparkMax shoulderMotorController) {
        super(20, TimeUnit.MILLISECONDS);
        this.primaryClimberMotorController = primaryClimberMotorController;
        this.followerClimberMotorController = followerClimberMotorController;
        this.shoulderMotorController = shoulderMotorController;
    }

    @Override
    public void extendShoulder(double speed) {
        // Read the climber's position.
        this.primaryClimberMotorController.set(speed);

        // long now = System.nanoTime();
        // if(this.starttime < 0) this.starttime = now;
        //     this.shoulderMotorController.set(1); // test value
        // if(now > this.starttime + TimeUnit.SECONDS.toNanos(5)) this.shoulderMotorController.set(0);
    }

    // Possibly unnecessary.
    // @Override
    // public void retractOuterClimber(double speed) {
    //     if (Timer.getMatchTime() > 120) {
    //         this.primaryClimberMotorController.set(-speed);
    //         this.followerClimberMotorController.set(-speed);
    //     }

    //     // long now = System.nanoTime();
    //     // if(this.starttime < 0) this.starttime = now;
    //     // this.shoulderMotorController.set(1); // test value
    //     // if(now > this.starttime + TimeUnit.SECONDS.toNanos(5)) this.shoulderMotorController.set(0);
    // }

    // @Override
    // public void angleOuter(double angle) {}

    // The Neo motor controller.
    @Override
    public void extendElbow(double speed) {
        // if (Timer.getMatchTime() > 100) {
        // Obtain the shoulderMotorController encoder reading position.
        double position = this.shoulderMotorController.getEncoder().getPosition();
        double lowerSoftStop = -10;

        if (position <= lowerSoftStop && speed < 0) {
            this.shoulderMotorController.set(0);
        } else {
            this.shoulderMotorController.set(speed);
        }
        // double shoulderMotorControllerPosition = shoulderMotorController.getEncoder().getPosition();

        // if (shoulderMotorControllerPosition > 500000) {  // Utilize some value in place of 0. Test this by logging the encoder position values to the SmartDashboard and set an appropriate value.
        //     this.shoulderMotorController.set(0);
        // } else {
        // }
        // }
    }

    @Override
    public void resetClimberPosition(){
        this.shoulderMotorController.getEncoder().setPosition(0);
        this.primaryClimberMotorController.setSelectedSensorPosition(0);
        this.followerClimberMotorController.setSelectedSensorPosition(0);
    }

    // Possibly unnecessary.
    // @Override
    // public void retractInnerLifter(double speed) {
    //     if (Timer.getMatchTime() > 120) {
    //         this.shoulderMotorController.set(-speed);
    //     }
    // }

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

    @Override
    public void primeClimber() {
        double positionNEO = this.shoulderMotorController.getEncoder().getPosition();
        double leftClimberMotorControllerPosition = primaryClimberMotorController.getSelectedSensorPosition();
        double rightClimberMotorControllerPosition = followerClimberMotorController.getSelectedSensorPosition();
        double lowerSoftStop = -3.25;
        // if (positionNEO >= lowerSoftStop && !climberDeployed && !innerUp) {
        //     this.shoulderMotorController.set(-0.6);
        // }else{
        //     climberDeployed = true;
        // }
        
        // if(climberDeployed && positionNEO <= 10 && !innerUp){
        //     this.shoulderMotorController.set(0.6);
        // }else{
        //     innerUp = true;
        // }
        
        
        // if(climberDeployed && innerUp && leftClimberMotorControllerPosition < 100000){
        //     this.shoulderMotorController.set(0);
        //     this.primaryClimberMotorController.set(0.2);
        // }

        
    }

    @Override
    public void none() {
        this.shoulderMotorController.set(0);
        
    }

}
