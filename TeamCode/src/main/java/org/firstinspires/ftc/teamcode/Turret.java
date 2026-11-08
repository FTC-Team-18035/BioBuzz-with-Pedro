package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    public double tx = 58; //cm The distance from the field wall to the hives along the x axis
    public double tay = 52; //cm The distance from the field wall to the first hive along the y axis
    public double tby = 92; //cm The distance from the field wall to the second hive along the y axis
    public double centerline = 72; //cm The center of the field, where the turret should switch its target to the other side's hive

    public double hoodServoMax = .5; // TODO: Determine actual max of the hood servo positions
    public double hoodServoMin = .5; // TODO: Determine actual min of the hood servo positions
    public double shooterMinAngle = 5; // degrees, minimum launch angle from horizontal
    public double shooterMaxAngle = 25; // degrees, maximum launch angle from horizontal
    public double hoodServoSetpoint;
    private double ty;
    private double distance;
    private double absAngle;
    private double launchVelocity;
    private double launchAngle;
    private double currentAngle;
    private double targetAngle;

    private double turretTargetAngle;
    private DcMotorEx flywheelA;
    private DcMotorEx flywheelB;
    private DcMotorEx feeder;

    public Servo turretAzimuthA;
    public Servo turretAzimuthB;
    public Servo shooterAngleA;
    public Servo shooterAngleB;

    private double kP = 0; // Proportional gain for shooter TODO: Replace with the tuned value from "FlywheelTuner.java"
    private double kF = 0; // Feedforward gain for shooter TODO: Replace with the tuned value from "FlywheelTuner.java"


    public Turret(HardwareMap hardwareMap) {

        // Configure shooter motors, servos, and PIDF controllers
        flywheelA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelA.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
        flywheelA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        this.turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        this.turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");
        this.shooterAngleA  = hardwareMap.get(Servo.class, "shooterAngleA");
        this.shooterAngleB  = hardwareMap.get(Servo.class, "shooterAngleB");
        this.feeder         = hardwareMap.get(DcMotorEx.class, "feeder");
    }

    public void initialize() {

        flywheelA.setVelocity(0);
        flywheelB.setVelocity(0);

        feeder.setPower(0);

        turretAzimuthA.setPosition(.5);
        turretAzimuthB.setPosition(.5);

        shooterAngleA.setPosition(.5);
        shooterAngleB.setPosition(.5);
    }

    public double target(double rx, double ry, double heading, double vx) {

        if (ry <= centerline) {
            ty = tay;
        }
        else if (ry > centerline) {
            tx = tby;
        }

        distance = Math.sqrt(Math.pow((tx-rx), 2) + Math.pow((ty-ry), 2)); // Calculate the distance to the correct target using the formula for distance between two points
        absAngle = Math.toDegrees(Math.atan2((ty-ry), (tx-rx))); // Calculate the angle from the robot's pose to the hive
        targetAngle = heading - absAngle + 180; // Calculate the turret target angle relative to the chassis heading

        launchVelocity = Trajectory.CalculateVelocity(distance);
        launchAngle = Trajectory.CalculateAngle(distance);

        //Calculate the aim offset based on the robot's translational velocity
        double velocityOffset = Math.atan(vx/(launchVelocity * Math.cos(launchAngle)));

        targetAngle += Math.toDegrees(velocityOffset); // Adjust the target angle based on the velocity offset

        if (targetAngle >= 360) {
            targetAngle = targetAngle - 360; // Wrap around if the angle exceeds 360 degrees
        } else if (targetAngle <= 0) {
            targetAngle = targetAngle + 360; // Wrap around if the angle is negative
        }

        double servoPosition = targetAngle / 360; // Convert angle (degrees) to servo position (0 to 1)

        setHoodAngle(launchAngle);
        runFlywheel(launchVelocity);

        return servoPosition;
    }

    private void runFlywheel(double launchVelocity) {
        double launchTPS = (launchVelocity * 28)/(7.2*Math.PI);
        flywheelA.setVelocity(launchTPS);
        flywheelB.setVelocity(launchTPS);
    }
    private void setHoodAngle(double launchAngle) {
        hoodServoSetpoint = ((launchAngle - shooterMinAngle) / (shooterMaxAngle - shooterMinAngle)) * (hoodServoMax - hoodServoMin) + hoodServoMin;
        shooterAngleA.setPosition(hoodServoSetpoint);
        shooterAngleB.setPosition(hoodServoSetpoint);
    }
    public void feed() {
        feeder.setPower(1);
    }

    public void stopFeeding() {
        feeder.setPower(0);
    }

    public void stopFlywheel() {
        flywheelA.setVelocity(0);
        flywheelB.setVelocity(0);
    }

    public void emergencyStop() {
        feeder.setPower(0);
        flywheelA.setPower(0);
        flywheelB.setPower(0);
    }

}