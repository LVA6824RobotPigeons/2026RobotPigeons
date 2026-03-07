package org.py;

public class Joystick {

    public String name;
    public Controls.JoystickComponents component;

    public DPoint value = new DPoint(0.0,0.0);
    public DPoint previousValue = new DPoint(0.0,0.0);

    public Joystick(String name, Controls.JoystickComponents component) {
        this.name = name;
        this.component = component;
    }

    public void setValue(DPoint value) {
        this.value = value;
    }

    public void setX(double x) {
        value.x = x;
    }
    public void setY(double y) {
        value.y = y;
    }

}
