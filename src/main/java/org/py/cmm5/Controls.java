// For any questions or recommendations, contact pyramidstudios.dev@gmail.com

package org.py.cmm5;

import java.util.ArrayList;
import java.util.function.Consumer;

public class Controls {

    public static final String model = "4.2";

    public static boolean errorLoggerOn = true;
    private static boolean controlsLoggerOn = false;
    private static boolean inputLoggerOn = false;
    private static final ArrayList<Integer> inputLoggerControllers = new ArrayList<>();

    private static final ArrayList<Control> linkedControls = new ArrayList<>();

    private static boolean suppressJoystickOutput = false;

    public static ArrayList<Controller> controllers = new ArrayList<>();

    private static final ArrayList<Control> controls = new ArrayList<>();

    public enum BinaryComponents { A, B, X, Y, DL, DR, DU, DD, LB, RB, SA, SB, BB, JA, JB };
    public enum ThresholdComponents { LT, RT, AX, AY, BX, BY};
    public enum JoystickComponents { A, B };
    public enum ComponentTypes { Binary, Threshold, Joystick };


    public static Control addControl(String name, BinaryComponents component) {
        Control c = newControl(name,component);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, BinaryComponents component, String condition) {
        Control c = newControl(name,component,condition);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, BinaryComponents component, int[] controllers) {
        Control c = newControl(name,component,controllers);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, ThresholdComponents component, String condition, int[] controllers) {
        Control c = newControl(name,component,condition,controllers);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, ThresholdComponents component) {
        Control c = newControl(name,component);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, ThresholdComponents component, String condition) {
        Control c = newControl(name,component,condition);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, ThresholdComponents component, int[] controllers) {
        Control c = newControl(name,component,controllers);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, JoystickComponents component, String condition) {
        Control c = newControl(name,component,condition);
        controls.add(c);
        return c;
    }
    public static Control addControl(String name, JoystickComponents component, String condition, int[] controller) {
        Control c = newControl(name,component,condition);
        controls.add(c);
        return c;
    }

    public static Control newControl(String name, BinaryComponents component, String condition) {
        return newControl(name,component,condition,new int[0]).allControllers();
    }
    public static Control newControl(String name, BinaryComponents component) {
        return newControl(name,component,"ACTIVE",new int[0]).allControllers();
    }
    public static Control newControl(String name, BinaryComponents component, int[] controllers) {
        return newControl(name,component,"ACTIVE",controllers);
    }
    public static Control newControl(String name, BinaryComponents component, String condition, int[] controllers) {
        return new Control(name,component,condition,controllers);
    }
    public static Control newControl(String name, ThresholdComponents component, String condition) {
        return newControl(name,component,condition,new int[0]).allControllers();
    }
    public static Control newControl(String name, ThresholdComponents component) {
        return newControl(name,component,"GREATER_THAN:0",new int[0]).allControllers();
    }
    public static Control newControl(String name, ThresholdComponents component, int[] controllers) {
        return newControl(name,component,"GREATER_THAN:0",controllers);
    }
    public static Control newControl(String name, ThresholdComponents component, String condition, int[] controllers) {
        return new Control(name,component,condition,controllers);
    }
    public static Control newControl(String name, JoystickComponents component, String condition) {
        return newControl(name,component,condition,new int[0]).allControllers();
    }
    public static Control newControl(String name, JoystickComponents component, String condition, int[] controllers) {
        return new Control(name,component, condition, controllers);
    }

    public static void setController(Controller controller) {
        ArrayList<Controller> c = new ArrayList<>(); c.add(controller);
        controllers = c;
    }
    public static void setControllers(ArrayList<Controller> controllers) {
        Controls.controllers = controllers;
    }
    public static void addController(Controller controller) {
        controllers.add(controller);
    }

    public static Control getControl(String name) {
        for(Control control : controls) {
            if(control.name.equals(name)) return control;
        }
        if(errorLoggerOn) System.out.println("[ControlsManager:ErrorLogger] conditionResolve("+name+") : invalid control name");
        return null;
    }

    public static boolean conditionResolve(String name) {
        Control control = getControl(name);
        if(control != null) for(Integer controller : control.controllers) {
            if(controllers.size() >= controller-1)
                if(control.conditionResolve(controllers.get(controller-1))) return true;
        }
        return false;
    }

    public static double getJoystickAngle(JoystickComponents joystick, int controller) {
        return Math.atan2(controllers.get(controller-1).getComponent(joystick).value.y, controllers.get(controller-1).getComponent(joystick).value.x)*180.0/Math.PI;
    }

    public static void bindFunctionToControl(String name, boolean executeOnceWhenBecomeTrue, boolean executeOnInactive, Consumer<String> function) {
        Control c = getControl(name);
        if(c != null) c.bindFunction(function,executeOnceWhenBecomeTrue,executeOnInactive);

    }

    public static void processAll() {

        // Controls Processor
         for(Control control : controls) {
            boolean resolved = true;
             if(control.allControllers) {
                 for(Controller controller : controllers) {
                     if(!control.conditionResolve(controller)) resolved = false;
                     for(Control linked : control.linkedControls)
                         if(!linked.conditionResolve(controller)) resolved = false;
                 }
             } else {
                 for(Integer controller : control.controllers) {
                     if(controllers.size() >= controller) {
                         if(!control.conditionResolve(controllers.get(controller-1))) resolved = false;
                         for(Control linked : control.linkedControls)
                             if(!linked.conditionResolve(controllers.get(controller-1))) resolved = false;
                     }
                 }
             }
             if(controllers.isEmpty() || control.controllers.length == 0) resolved = false; // If no controllers
             control.conditionTrue = resolved;
            if(controlsLoggerOn && (control.conditionWasTrue != control.conditionTrue))
                System.out.println("[ControlsManager:ControlsLogger] Control \"" + control.name + "\" new state detected: " + control.conditionTrue);
            control.process(); // Contains boundFunction execution code
            control.conditionWasTrue = resolved;
        }

        // Input Processor
        if(!controllers.isEmpty()) for(int i = 0; i < controllers.size(); i++) {
            if(inputLoggerControllers.isEmpty() || inputLoggerControllers.contains(i+1)) {
                Controller controller = controllers.get(i);
                for(Component component : controller.components) {
                    if(inputLoggerOn && (component.value != component.previousValue)
                        && !(component.thresholdComponent == Controls.ThresholdComponents.AX
                        || component.thresholdComponent == Controls.ThresholdComponents.AY
                        || component.thresholdComponent == Controls.ThresholdComponents.BX
                        || component.thresholdComponent == Controls.ThresholdComponents.BY))
                        System.out.println("[ControlsManager:InputLogger] Controller " + (i+1) + " - " + component.name + " new state detected: " + component.value);
                    component.previousValue = component.value;
                }
                if(inputLoggerOn && !suppressJoystickOutput)
                    for(Joystick joystick : controller.joysticks) {
                        if(
                                (joystick.value.x != joystick.previousValue.x) ||
                                        (joystick.value.y != joystick.previousValue.y)
                        ) System.out.println("[ControlsManager:InputLogger] Controller " + (i+1) + " - " + joystick.name + " new state detected: " + joystick.value.toString());
                        joystick.previousValue.x = joystick.value.x;
                        joystick.previousValue.y = joystick.value.y;
                    }
            }
        }
    }


    public static void controlsLogger(boolean on) {
        controlsLoggerOn = on;
        System.out.println("[ControlsManager:ControlLogger] Controls Logger set to: " + on);
    }
    public static void inputLogger(boolean on, boolean suppressJoystickOutput) {
        inputLoggerOn = on;
        Controls.suppressJoystickOutput = suppressJoystickOutput;
        System.out.println("[ControlsManager:InputLogger] Input Logger set to: " + on);
    }
    public static void inputLogger(boolean on, boolean suppressJoystickOutput, int controller) {
        inputLoggerOn = on;
        inputLoggerControllers.add(controller);
        Controls.suppressJoystickOutput = suppressJoystickOutput;
        System.out.println("[ControlsManager:InputLogger] Input Logger set to: " + on + " (Controller " + controller + ")");
    }
    public static void errorLogger(boolean on) {
        errorLoggerOn = on;
        System.out.println("[ControlsManager:ErrorLogger] Error Logger set to: " + on);
    }


}

