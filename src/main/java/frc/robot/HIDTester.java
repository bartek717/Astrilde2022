package frc.robot;

import java.util.Collection;
import java.util.Set;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import ca.team3161.lib.utils.ComposedComponent;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HIDTester extends RepeatingPooledSubsystem implements ComposedComponent<GenericHID> {

    protected final GenericHID hid;
    protected final String label;

    public HIDTester(int port) {
        this.hid = new GenericHID(port);
        this.label = String.format("HIDTester(%d)", port);
        start();
    }

    @Override
    public void defineResources() { }

    @Override
    public void task() throws Exception {
        for (int i = 0; i < hid.getAxisCount(); i++) {
            SmartDashboard.putNumber(String.format("%s Axis[%d]", label, i), hid.getRawAxis(i));
        }
        for (int i = 0; i < hid.getButtonCount(); i++) {
            SmartDashboard.putBoolean(String.format("%s Button[%d]", label, i), hid.getRawButton(i));
        }
        for (int i = 0; i < hid.getPOVCount(); i++) {
            SmartDashboard.putNumber(String.format("%s POV[%d]", label, i), hid.getPOV(i));
        }
    }

    @Override
    public Collection<GenericHID> getComposedComponents() {
        return Set.of(hid);
    }

}
