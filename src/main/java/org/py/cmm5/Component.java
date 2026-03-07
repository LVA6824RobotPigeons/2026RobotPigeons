package org.py;

public class Component {

    public String name;
    public double value = 0.0;
    public double previousValue = 0.0; // Gets processed by InputLogger

    public Controls.BinaryComponents binaryComponent;
    public Controls.ThresholdComponents thresholdComponent;
    public Controls.JoystickComponents joystickComponent;
    public Controls.ComponentTypes type;

    public Component(String name, Controls.BinaryComponents component) {
        this.name = name;
        type = Controls.ComponentTypes.Binary;
        binaryComponent = component;
    }
    public Component(String name, Controls.ThresholdComponents component) {
        this.name = name;
        type = Controls.ComponentTypes.Threshold;
        thresholdComponent = component;
    }
    public Component(String name, Controls.JoystickComponents component) {
        this.name = name;
        type = Controls.ComponentTypes.Joystick;
        joystickComponent = component;
    }

    public void setValue(double value) {
        this.value = value;
    }

}
