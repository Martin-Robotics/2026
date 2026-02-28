package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    // public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 12;
    public static final int kFloor = 10;
    public static final int kFeeder = 14;
    public static final int kIntakeRollers = 20;//11;
    public static final int kHanger = 99;//18;
    //Climb - SparkMax Right ID 21
    //Climp - SparkMax Left  ID 22
    public static final int kShooterLeft = 11;
    public static final int kShooterMiddle = 2;
    public static final int kShooterRight = 5;

    // ======================== SWERVE DRIVE MOTOR IDs ========================
    // Front Left Module
    public static final int kFrontLeftDriveMotor = 13;
    public static final int kFrontLeftSteerMotor = 9;
    public static final int kFrontLeftEncoder = 16;

    // Front Right Module
    public static final int kFrontRightDriveMotor = 3;
    public static final int kFrontRightSteerMotor = 6;
    public static final int kFrontRightEncoder = 17;

    // Back Left Module
    public static final int kBackLeftDriveMotor = 1;
    public static final int kBackLeftSteerMotor = 8;
    public static final int kBackLeftEncoder = 19;

    // Back Right Module
    public static final int kBackRightDriveMotor = 4;
    public static final int kBackRightSteerMotor = 7;
    public static final int kBackRightEncoder = 18;

    // Gyro/Pigeon 2
    public static final int kPigeon2Id = 15;

    // PWM Ports
    public static final int kHoodLeftServo = 1;
    public static final int kHoodRightServo = 0;
}
