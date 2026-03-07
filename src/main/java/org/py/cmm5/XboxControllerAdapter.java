package org.py.cmm5;

// Translates WPILib's XboxController Output to ControlManager's Controller Output

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;

public class XboxControllerAdapter {

    // These act more like pointers
    public XboxController xboxController;
    public Controller controller;

    public XboxControllerAdapter(XboxController xboxController, Controller controller) {
        this.xboxController = xboxController;
        this.controller = controller;
    }

    // Updates Controller's values from XboxController's values
    public void sync() {
        for(int i = 0; i < controller.components.length; i++) {
            Component c = controller.components[i];
            switch(c.type) {
                case Binary:
                    switch(c.binaryComponent) {
                        case A:  c.value = xboxController.getAButton()?1.0:0.0; break;
                        case B:  c.value = xboxController.getBButton()?1.0:0.0; break;
                        case X:  c.value = xboxController.getXButton()?1.0:0.0; break;
                        case Y:  c.value = xboxController.getYButton()?1.0:0.0; break;
//                        case DL: c.value = xboxController.povLeft()||xboxController.povUpLeft()||xboxController.povDownRight()?1.0:0.0; break;
//                        case DR: c.value = xboxController.povRight()||xboxController.povUpRight()||xboxController.povDownRight()?1.0:0.0; break;
//                        case DU: c.value = xboxController.povUp()||xboxController.povUpLeft()||xboxController.povRight()?1.0:0.0; break;
//                        case DD: c.value = xboxController.povDown()||xboxController.povDownLeft()||xboxController.povDownRight()?1.0:0.0; break;
                        case JA: c.value = xboxController.getLeftStickButton()?1.0:0.0; break;
                        case JB: c.value = xboxController.getRightStickButton()?1.0:0.0; break;
                        case SA: c.value = xboxController.getBackButton()?1.0:0.0; break;
                        case SB: c.value = xboxController.getStartButton()?1.0:0.0; break;
                        case LB: c.value = xboxController.getLeftBumperButton()?1.0:0.0; break;
                        case RB: c.value = xboxController.getRightBumperButton()?1.0:0.0; break;
                    } break;
                case Threshold:
                    switch(c.thresholdComponent) {
                        case LT: c.value = xboxController.getLeftTriggerAxis(); break;
                        case RT: c.value = xboxController.getRightTriggerAxis(); break;
                    } break;
            }
        }
        for(int i = 0; i < controller.joysticks.length; i++) {
            Joystick j = controller.joysticks[i];
            switch(j.component) {
                case A: j.value = new DPoint(xboxController.getLeftX(),xboxController.getLeftY());
                case B: j.value = new DPoint(xboxController.getRightX(),xboxController.getRightY());
            }
        }
    }



}
