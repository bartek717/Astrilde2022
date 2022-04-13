package frc.robot;


import frc.robot.XBoxOneController.XBoxOneAxis;
import frc.robot.XBoxOneController.XBoxOneButton;
import frc.robot.XBoxOneController.XBoxOneControl;

public final class ControllerBindings {
    public static final XBoxOneAxis Y_AXIS = XBoxOneAxis.Y;
    public static final XBoxOneAxis X_AXIS = XBoxOneAxis.X;
    public static final XBoxOneAxis LEFT_TRIGGER_AXIS = XBoxOneAxis.LEFT_TRIGGER_AXIS;
    public static final XBoxOneAxis RIGHT_TRIGGER_AXIS = XBoxOneAxis.RIGHT_TRIGGER_AXIS;

    public static final XBoxOneControl RIGHT_STICK = XBoxOneControl.RIGHT_STICK;
    public static final XBoxOneControl LEFT_STICK = XBoxOneControl.LEFT_STICK;

    public static final XBoxOneControl INTAKE = XBoxOneControl.LEFT_TRIGGER;
    public static final XBoxOneControl OUTAKE = XBoxOneControl.RIGHT_TRIGGER;
    public static final XBoxOneButton OVERRIDE_ELEVATOR_GATE = XBoxOneButton.SELECT; // FIXME bind to something else

    public static final XBoxOneButton SHOOT_FENDER = XBoxOneButton.A;
    public static final XBoxOneButton SHOOT_GENERAL = XBoxOneButton.X;
    public static final XBoxOneButton SHOOT_LAUNCH_CLOSE = XBoxOneButton.RIGHT_BUMPER;
    public static final XBoxOneButton DEPLOY_CLIMBER = XBoxOneButton.B;
    public static final XBoxOneButton NOT_AIM = XBoxOneButton.Y;

    public static final XBoxOneButton DRIVE_REVERSE = XBoxOneButton.LEFT_BUMPER;

    // Potential climber aspects (not required; one can instead utilize RIGHT_STICK and LEFT_STICK if they prefer in Robot.java).
    public static final XBoxOneControl CLIMBER = XBoxOneControl.LEFT_STICK;
    public static final XBoxOneControl CLIMBER_LIFTER = XBoxOneControl.RIGHT_STICK;
    public static final XBoxOneButton ELEVATOR_IN = XBoxOneButton.LEFT_BUMPER;
}
