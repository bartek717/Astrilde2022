package frc.robot;

import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;

public final class ControllerBindings {
    public static final LogitechAxis Y_AXIS = LogitechAxis.Y;
    public static final LogitechAxis X_AXIS = LogitechAxis.X;

    public static final LogitechControl RIGHT_STICK = LogitechControl.RIGHT_STICK;
    public static final LogitechControl LEFT_STICK = LogitechControl.LEFT_STICK;

    public static final LogitechButton INTAKE = LogitechButton.LEFT_TRIGGER;
    public static final LogitechButton OUTTAKE = LogitechButton.RIGHT_TRIGGER;
    public static final LogitechButton OVERRIDE_ELEVATOR_GATE = LogitechButton.SELECT; // FIXME bind to something else

    public static final LogitechButton SHOOT_FENDER = LogitechButton.A;
    public static final LogitechButton SHOOT_GENERAL = LogitechButton.X;
    public static final LogitechButton SHOOT_LAUNCH_CLOSE = LogitechButton.RIGHT_BUMPER;
    public static final LogitechButton AIM = LogitechButton.B;
    public static final LogitechButton NOT_AIM = LogitechButton.Y;

    public static final LogitechButton DRIVE_REVERSE = LogitechButton.LEFT_BUMPER;

}
