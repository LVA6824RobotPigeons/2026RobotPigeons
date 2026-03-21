package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import frc.robot.subsystems.led8.LED8;
import frc.robot.subsystems.led8.LED8Group;

public final class Ports {

    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("Systems");

    // CANdle
//    public static final CANdle kCANdle = new CANdle(0,kRoboRioCANBus);
//    public static final LED8 kCandle = new LED8(kCANdle,0,7);
//    public static final LED8 kStrip = new LED8(kCANdle,8,167);
//    public static final LED8Group kLights = new LED8Group().add(kCandle).add(kStrip);

    // Talon FX IDs
    public static final int kIntakePivot = 10;
    public static final int kIntakeRollers = 11;
    public static final int kFloor = 12;
    public static final int kFeeder = 13;
    public static final int kShooterLeft = 14;
    public static final int kShooterMiddle = 15;
    public static final int kShooterRight = 16;
    public static final int kHanger = 18;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;

    // Controller Ports
    public static final int driver = 0;

}
