package org.py.cmm5;

public class Controller {

    public int jid;

    public Controller(int jid) {
        this.jid = jid;
    }

    public Component[] components = {

            new Component("Button A", Controls.BinaryComponents.A),
            new Component("Button B", Controls.BinaryComponents.B),
            new Component("Button X", Controls.BinaryComponents.X),
            new Component("Button Y", Controls.BinaryComponents.Y),

            new Component("DPad Left", Controls.BinaryComponents.DL),
            new Component("DPad Right", Controls.BinaryComponents.DR),
            new Component("DPad Up", Controls.BinaryComponents.DU),
            new Component("DPad Down", Controls.BinaryComponents.DD),

            new Component("Joystick A Push", Controls.BinaryComponents.JA),
            new Component("Joystick B Push", Controls.BinaryComponents.JB),

            new Component("Special Button A", Controls.BinaryComponents.SA),
            new Component("Special Button B", Controls.BinaryComponents.SB),
            new Component("Big Button", Controls.BinaryComponents.BB),

            new Component("Right Bumper", Controls.BinaryComponents.LB),
            new Component("Right Bumper", Controls.BinaryComponents.RB),

            new Component("Left Trigger", Controls.ThresholdComponents.LT),
            new Component("Right Trigger", Controls.ThresholdComponents.RT),

            new Component("Joystick A - X", Controls.ThresholdComponents.AX),
            new Component("Joystick A - Y", Controls.ThresholdComponents.AY),
            new Component("Joystick B - X", Controls.ThresholdComponents.BX),
            new Component("Joystick B - Y", Controls.ThresholdComponents.BY)

    };
    Joystick[] joysticks = {

        new Joystick("Joystick A",Controls.JoystickComponents.A),
        new Joystick("Joystick B",Controls.JoystickComponents.B)

    };

    public void setValue(String name,double value) {
        getComponent(name).value = value;
    }

    public Component getComponent(String name) {
        for(Component component : components) if(component.name.equals(name)) return component; return null;
    }
    public Joystick getJoystick(String name) {
        for(Joystick joystick : joysticks) if(joystick.name.equals(name)) return joystick; return null;
    }
    public Component getComponent(Controls.BinaryComponents component) {
        for(Component component_ : components)
            if(component_.type == Controls.ComponentTypes.Binary && component_.binaryComponent == component)
                return component_;
        return null;
    }
    public Component getComponent(Controls.ThresholdComponents component) {
        for(Component component_ : components)
            if(component_.type == Controls.ComponentTypes.Threshold && component_.thresholdComponent == component)
                return component_;
        return null;
    }
    public Joystick getComponent(Controls.JoystickComponents component) {
        for(Joystick joystick : joysticks)
            if(joystick.component == component)
                return joystick;
        return null;
    }

}
