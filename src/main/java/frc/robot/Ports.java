package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import frc.robot.subsystems.led8.LED8;

public final class Ports {

    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // CANdle
    public static final CANdle kCANdle = new CANdle(0,kRoboRioCANBus);
    public static final LED8 kCandle = new LED8(kCANdle,Constants.LEDs.kNumberOfLights-1);

    // Talon FX IDs
    public static final int kIntakePivot = -1;//10;
    public static final int kIntakeRollers = -1;//11;
    public static final int kFloor = -1;//12;
    public static final int kFeeder = -1;//13;
    public static final int kShooterLeft = -1;//14;
    public static final int kShooterMiddle = -1;//15;
    public static final int kShooterRight = -1;//16;
    public static final int kHanger = -1;//18;

    // PWM Ports
    public static final int kHoodLeftServo = -1;//3;
    public static final int kHoodRightServo = -1;//4;

    // Controller Ports
    public static final int driver = 0;

}
