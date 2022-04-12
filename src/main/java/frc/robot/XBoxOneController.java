/*
 * Copyright (c) 2014-2017, FRC3161
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package frc.robot;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Function;

import ca.team3161.lib.utils.Utils;
import ca.team3161.lib.utils.controls.AbstractController;
import ca.team3161.lib.utils.controls.LinearJoystickMode;
// import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.controls.Gamepad;

/**
 * A Gamepad implementation describing the Logitech DualAction gamepad.
 */
public class XBoxOneController extends AbstractController {

    /**
     * {@inheritDoc}.
     */
    public enum XBoxOneControl implements Control {
        LEFT_STICK(0),
        RIGHT_STICK(1),
        LEFT_TRIGGER(2),
        RIGHT_TRIGGER(3);

        private final int id;

        XBoxOneControl(final int id) {
            this.id = id;
        }

        @Override
        public int getIdentifier(final Axis axis) {
            Objects.requireNonNull(axis);
            // System.out.println(axis);
            if(this.id == 0 && axis.equals(XBoxOneAxis.X)){
                return 0;
            }
            if(this.id == 0 && axis.equals(XBoxOneAxis.Y)){
                return 1;
            }
            if(this.id == 1 && axis.equals(XBoxOneAxis.X)){
                return 4;
            }
            if(this.id == 1 && axis.equals(XBoxOneAxis.Y)){
                return 5;
            }

            return this.id;
            // return id * XBoxOneControl.values().length + axis.getIdentifier();
        }
    }

    /**
     * {@inheritDoc}.
     */
    public enum XBoxOneAxis implements Axis {
        X(0),
        Y(1),
        LEFT_TRIGGER_AXIS(2),
        RIGHT_TRIGGER_AXIS(3);

        private final int id;

        XBoxOneAxis(final int id) {
            this.id = id;
        }

        @Override
        public int getIdentifier() {
            return id;
        }
    }

    /**
     * {@inheritDoc}.
     */
    public enum XBoxOneButton implements Button {
        A(1),
        B(2),
        X(3),
        Y(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        START(7),
        SELECT(8),
        LEFT_STICK_CLICK(9),
        RIGHT_STICK_CLICK(10);

        private final int id;

        XBoxOneButton(final int id) {
            this.id = id;
        }

        @Override
        public int getIdentifier() {
            return this.id;
        }
    }

    public enum DpadDirection {
        NONE(-1),
        UP(0),
        UP_RIGHT(45),
        RIGHT(90),
        DOWN_RIGHT(135),
        DOWN(180),
        DOWN_LEFT(225),
        LEFT(270),
        UP_LEFT(315);

        private final int angle;

        DpadDirection(final int angle) {
            this.angle = angle;
        }

        public int getAngle() {
            return angle;
        }
    }

    /**
     * Create a new XBoxOneController gamepad/controller.
     *
     * @param port the USB port for this controller
     */
    public XBoxOneController(final int port) {
        this(port, 20, TimeUnit.MILLISECONDS);
    }

    /**
     * Create a new XBoxOneController gamepad/controller, with a specific polling frequency (for button bindings).
     * For example, to poll at 50Hz, you might use a period of 20 and a timeUnit of TimeUnit.MILLISECONDS.
     *
     * @param port     the USB port for this controller.
     * @param period   the timeout period between button mapping polls.
     * @param timeUnit the unit of the timeout period.
     */
    public XBoxOneController(final int port, final int period, final TimeUnit timeUnit) {
        super(port, period, timeUnit);
        for (final Control control : XBoxOneControl.values()) {
            for (final Axis axis : XBoxOneAxis.values()) {
                controlsModeMap.put(new Mapping(control, axis), new LinearJoystickMode());
            }
        }
    }

    protected static void validate(final Mapping mapping, final String message) {
        Objects.requireNonNull(mapping);
        if (!(mapping.getControl() instanceof XBoxOneControl) || !(mapping.getAxis() instanceof XBoxOneAxis)) {
            throw new IllegalArgumentException(message);
        }
    }

    protected static void validate(final Button button, final String message) {
        Objects.requireNonNull(button);
        if (!(button instanceof XBoxOneButton)) {
            throw new IllegalArgumentException(message);
        }
    }

    protected static void validate(final Binding binding, final String message) {
        Objects.requireNonNull(binding);
        binding.getButtons().stream().forEach((Gamepad.Button button) -> {
            if (!(button instanceof XBoxOneButton)) {
                throw new IllegalArgumentException(message);
            }
        });
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getValue(final Mapping mapping) {
        validate(mapping, "Gamepad on port " + getPort() + " getValue() called with invalid mapping " + mapping);
        return controlsModeMap.get(mapping).apply(backingHID.getRawAxis(mapping.getControl().getIdentifier(mapping.getAxis())));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean getButton(final Button button) {
        validate(button, "Gamepad on port " + getPort() + " getButton() called with invalid button " + button);
        return backingHID.getRawButton(button.getIdentifier());
    }

    public int getDpad() {
        return backingHID.getPOV();
    }

    public DpadDirection getDpadDirection() {
        for (DpadDirection direction : DpadDirection.values()) {
            if (direction.getAngle() == getDpad()) {
                return direction;
            }
        }
        return DpadDirection.NONE;
    }

    /**
     *
     */
    @Override
    protected Set<Button> getButtons() {
        return new HashSet<>(Arrays.asList(XBoxOneButton.values()));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setMode(final Mapping mapping, final Function<Double, Double> function) {
        Objects.requireNonNull(function);
        validate(mapping, "Gamepad on port " + getPort() + " setMode() called with invalid mapping " + mapping);
        controlsModeMap.put(mapping, function);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setMode(final Function<Double, Double> function) {
        for (XBoxOneControl control : XBoxOneControl.values()) {
            for (XBoxOneAxis axis : XBoxOneAxis.values()) {
                setMode(control, axis, function);
            }
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void map(final Mapping mapping, final Consumer<Double> consumer) {
        Objects.requireNonNull(consumer);
        validate(mapping, "Gamepad on port " + getPort() + " map() called with invalid mapping " + mapping);
        controlsMapping.put(mapping, Utils.safeExec(mapping.toString(), consumer));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void bind(final Binding binding, final Runnable action) {
        Objects.requireNonNull(action);
        validate(binding, "Gamepad on port " + getPort() + " bind() called with invalid binding" + binding);
        buttonBindings.put(binding, () -> Utils.safeExec(binding.toString(), action));
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void unbind(final Binding binding) {
        validate(binding, "Gamepad on port " + this.port + " unbind() called with invalid binding " + binding);
        buttonBindings.remove(binding);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean hasBinding(final Binding binding) {
        validate(binding, "Gamepad on port " + this.port + " hasBinding() called with invalid binding " + binding);
        return buttonBindings.containsKey(binding);
    }
}
