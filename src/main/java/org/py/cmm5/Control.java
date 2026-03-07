package org.py.cmm5;

import java.util.ArrayList;
import java.util.function.Consumer;

public class Control {

    public Controls.BinaryComponents binaryComponent;
    public Controls.ThresholdComponents thresholdComponent;
    public Controls.JoystickComponents joystickComponent;

    public ArrayList<Control> linkedControls = new ArrayList<>();

    public int[] controllers;
    public boolean allControllers = false;
    public String name;
    
    private String condition;
    public boolean conditionTrue;
    public boolean conditionWasTrue;
    private Controls.ComponentTypes type;

    public Function boundFunction;

    public Control(String name, Controls.BinaryComponents component, String condition, int[] controllers) {
        binaryComponent = component;
        this.name = name;
        this.condition = condition;
        type = Controls.ComponentTypes.Binary;
        this.controllers = controllers;
    }
    public Control(String name, Controls.ThresholdComponents component, String condition, int[] controllers) {
        thresholdComponent = component;
        this.name = name;
        this.condition = condition;
        type = Controls.ComponentTypes.Threshold;
        this.controllers = controllers;
    }
    public Control(String name, Controls.JoystickComponents component, String condition, int[] controllers) {
        joystickComponent = component;
        this.name = name;
        this.condition = condition;
        type = Controls.ComponentTypes.Joystick;
        this.controllers = controllers;
    }
    public Control allControllers() {
        allControllers = true;
        return this;
    }

    public Control linkControl(Controls.BinaryComponents component) {
        linkedControls.add(new Control("link",component,"ACTIVE",new int[0]));
        return this;
    }
    public Control linkControl(Controls.BinaryComponents component, String condition) {
        linkedControls.add(new Control("link",component,condition,new int[0]));
        return this;
    }
    public Control linkControl(Controls.ThresholdComponents component) {
        linkedControls.add(new Control("link",component,"GREATER_THAN:0",new int[0]));
        return this;
    }
    public Control linkControl(Controls.ThresholdComponents component, String condition) {
        linkedControls.add(new Control("link",component,condition,new int[0]));
        return this;
    }
    public Control linkControl(Controls.JoystickComponents component, String condition) {
        linkedControls.add(new Control("link",component,condition,new int[0]));
        return this;
    }

    public void process() {
        if(boundFunction != null) {
            // ConditionTrue is set by ControlsLogger loop before all of this
            if(boundFunction.once && conditionTrue != conditionWasTrue)
                if(( boundFunction.onInactive && !conditionTrue)) boundFunction.execute();
                else if(!boundFunction.onInactive && conditionTrue) boundFunction.execute();
            if(conditionTrue && !boundFunction.once) boundFunction.execute();
        }
        // ConditionWasTrue set by controlsLogger as well
    }

    public void boundFunction() {
        if(boundFunction != null) boundFunction.execute();
    }
    public void bindFunction(Consumer<String> consumer,boolean once,boolean onInactive) {
        boundFunction = new Function(consumer,once,onInactive);
    }


    public boolean conditionResolve(Controller controller)  {
        switch(type) {
            case Binary:
                return complexConditionTrue(condition,controller.getComponent(binaryComponent).value,type);
            case Threshold:
                return complexConditionTrue(condition,controller.getComponent(thresholdComponent).value,type);
            case Joystick:
                return complexConditionTrue(condition,controller.getComponent(joystickComponent).value,type); // Value doesn't matter
        }
        if(Controls.errorLoggerOn) System.err.println("[ControlsManager:ErrorLogger] Unknown control type. (Type stored: " + type + ")");
        return false;
    }
    private boolean complexConditionTrue(String complex, double doubleValue, DPoint dpointValue, String valueType, Controls.ComponentTypes type) {
        ArrayList<String> conditions = new ArrayList<>();
        ArrayList<Character> splits = new ArrayList<>();
        String buffer = "";
        for(int i = 0; i < complex.length(); i++) {
            if(complex.charAt(i) == '|' || complex.charAt(i) == '&') {
                splits.add(complex.charAt(i));
                conditions.add(buffer);
                buffer = "";
            } else buffer += complex.charAt(i);
            if(i == complex.length()-1) conditions.add(buffer);
        }
        boolean outputBuffer = false;
        if(valueType.equals("dpoint")) outputBuffer = conditionTrue(conditions.get(0),dpointValue,type);
        else if(valueType.equals("double")) outputBuffer = conditionTrue(conditions.get(0),doubleValue,type);
        for(int i = 1; i < conditions.size(); i++) {
            boolean resolved = false;
            if(valueType.equals("dpoint")) resolved = conditionTrue(conditions.get(i),dpointValue,type);
            else if(valueType.equals("double")) resolved = conditionTrue(conditions.get(i),doubleValue,type);
            if(splits.get(i-1) == '|') outputBuffer = outputBuffer || resolved;
            else if(splits.get(i-1) == '&') outputBuffer = outputBuffer && resolved;
        }
        return outputBuffer;
    }
    private boolean complexConditionTrue(String complex, double value, Controls.ComponentTypes type) {
        return complexConditionTrue(complex,value,new DPoint(0.0,0.0),"double",type);
    }
    private boolean complexConditionTrue(String complex, DPoint value, Controls.ComponentTypes type) {
        return complexConditionTrue(complex,0.0,value,"dpoint",type);
    }
    private boolean conditionTrue(String condition, double value, Controls.ComponentTypes type) {
        String[] split = condition.split(":");
        switch(type) {
            case Threshold:
                switch(split[0]) {
                    case "RANGE":
                        return Double.parseDouble(split[1].split(",")[0]) <= value
                        && value <= Double.parseDouble(split[1].split(",")[1]);
                    case "LESS_THAN":
                        return value < Double.parseDouble(split[1]);
                    case "LESS_THAN_OR_EQUAL_TO":
                        return value <= Double.parseDouble(split[1]);
                    case "EQUAL_TO":
                        return value == Double.parseDouble(split[1]);
                    case "GREATER_THAN_OR_EQUAL_TO":
                        return value >= Double.parseDouble(split[1]);
                    case "GREATER_THAN":
                        return value > Double.parseDouble(split[1]);
                }
            case Binary:
                switch(split[0]) {
                    case "ACTIVE":
                        return value == 1.0;
                    case "INACTIVE":
                        return value == 0.0;
                }
        }
        if(Controls.errorLoggerOn) System.err.println("[ControlsManager:ErrorLogger] Unknown condition type: \"" + split[0] + "\"");
        return false;
    }
    private static boolean conditionTrue(String condition, DPoint value, Controls.ComponentTypes type) {
        if(type == Controls.ComponentTypes.Joystick) {
            boolean invert = (condition.charAt(0) == '!');
            String c = !invert?condition:condition.split("!")[1];
            boolean result = false;
            double angle = Math.atan2(value.y,value.x)*180.0/Math.PI;
            String direction = c.split(":")[0];
            switch(direction) {
                case "east":
                    result = -22.5 < angle && angle < 22.5;
                    break;
                case "southeast":
                    result = 22.5 < angle && angle < 67.5;
                    break;
                case "south":
                    result = 67.5 < angle && angle < 112.5;
                    break;
                case "southwest":
                    result = 112.5 < angle && angle < 157.5;
                    break;
                case "west":
                    result = (157.5 < angle && angle < 180.0) || (-180.0 < angle && angle < -157.5);
                    break;
                case "northwest":
                    result = -175.5 < angle && angle < -112.5;
                    break;
                case "north":
                    result = -112.5 < angle && angle < -67.5;
                    break;
                case "northeast":
                    result = -67.5 < angle && angle < -22.5;
                    break;
                default:
                    if(Controls.errorLoggerOn) System.err.println("[ControlsManager:ErrorLogger] Unknown direction given: \"" + condition + "\"");
            }
            if(c.split(":").length > 1 && Math.sqrt(value.x*value.x+value.y*value.y) < Double.parseDouble(c.split(":")[1])) result = false;
            return invert != result;
        }
        return false;
    }

    public Control setCondition(String complexCondition) {
        condition = complexCondition;
        return this;
    }
    public Control setComponent(Controls.BinaryComponents control) {
        binaryComponent = control;
        type = Controls.ComponentTypes.Binary;
        return this;
    }
    public Control setComponent(Controls.ThresholdComponents control) {
        thresholdComponent = control;
        type = Controls.ComponentTypes.Threshold;
        return this;
    }
    public Control setComponent(Controls.JoystickComponents control) {
        joystickComponent = control;
        type = Controls.ComponentTypes.Joystick;
        return this;
    }


}
