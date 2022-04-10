package frc.robot;

import ca.team3161.lib.utils.controls.JoystickMode;

public class PowerJoystickMode implements JoystickMode {

    private final double power;

    public PowerJoystickMode(double power) {
        this.power = power;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double adjust(final double raw) {
        final double negate;
        if (raw < 0.0d) {
            negate = -1.0d;
        } else {
            negate = 1.0d;
        }
        return negate * Math.abs(Math.pow(raw, power));
    }    
}
