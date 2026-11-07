package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Turret {
    private final PIDFController flywheelAController;
    private final PIDFController flywheelBController;
    public double tax = 72; //cm The distance from the field wall to the first hive along the x axis TODO: Measure the actual distance
    public double tbx = 72; //cm The distance from the field wall to the second hive along the x axis TODO: Measure the actual distance
    public double ty = 72; //cm The distance from the field wall to the hives along the y axis TODO: Measure the actual distance
    public double centerline = 72; //cm The center of the field, where the turret should switch its target to the other side's hive
    private double tx;
    private double distance;
    private double absAngle;
    private double launchVelocity;
    private double launchAngle;
    private double currentAngle;
    private double targetAngle;
    private double hiveAngle;
    private double turretTargetAngle;
    private static DcMotorEx flywheelA;
    private DcMotorEx flywheelB;
    private DcMotorEx feeder;

    public Servo turretAzimuthA;
    public Servo turretAzimuthB;
    public Servo shooterAngleA;
    public Servo shooterAngleB;

    private final double kP = 0; // Proportional gain for shooter TODO: Tune
    private final double kD = 0; // Derivative gain for shooter TODO: Tune
    private final double kF = 0; // Feedforward gain for shooter TODO: Tune


    public Turret(HardwareMap hardwareMap) {
        // Initialize your PID controllers inside the constructor
        this.flywheelAController = new PIDFController(kP, 0, kD, kF);
        this.flywheelBController = new PIDFController(kP, 0, kD, kF);

        // Fetch hardware map configurations directly by their names in the configuration
        this.flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        this.flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");
        this.flywheelB.setDirection(DcMotorEx.Direction.REVERSE);

        this.turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        this.turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");
        this.shooterAngleA  = hardwareMap.get(Servo.class, "shooterAngleA");
        this.shooterAngleB  = hardwareMap.get(Servo.class, "shooterAngleB");
        this.feeder         = hardwareMap.get(DcMotorEx.class, "feeder");
    }

    public void initialize() {

        DcMotorEx[] motors = new DcMotorEx[]{flywheelA, flywheelB};
        for (DcMotorEx motor : motors) {
            MotorConfigurationType config = motor.getMotorType().clone();
            config.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(config);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        flywheelA.setPower(0);
        flywheelB.setPower(0);

        feeder.setPower(0);

        turretAzimuthA.setPosition(.5);
        turretAzimuthB.setPosition(.5);

        //shooterAngleA.setPosition(0);
        //shooterAngleB.setPosition(0);
    }

    public double target(double rx, double ry, double heading, double vy) {

        if (rx <= centerline) {
            tx = tax;
        }
        else if (rx > centerline) {
            tx = tbx;
        }

        distance = Math.sqrt(Math.pow((tx-rx), 2) + Math.pow((ty-ry), 2)); // Calculate the distance to the correct target using the formula for distance between two points
        absAngle = Math.toDegrees(Math.atan2((ty-ry), (tx-rx))); // Calculate the angle from the robot's pose to the hive
        targetAngle = heading - absAngle + 180; // Calculate the turret target angle relative to the chassis heading

        launchVelocity = Trajectory.CalculateVelocity(distance);
        launchAngle = Trajectory.CalculateAngle(distance);

        //Calculate the aim offset based on the robot's translational velocity
        double velocityOffset = Math.atan(vy/(launchVelocity * Math.cos(launchAngle)));

        targetAngle += Math.toDegrees(velocityOffset); // Adjust the target angle based on the velocity offset

        if (targetAngle > 360) {
            targetAngle -= 360; // Wrap around if the angle exceeds 360 degrees
        } else if (targetAngle < 0) {
            targetAngle += 360; // Wrap around if the angle is negative
        }

        double servoPosition = targetAngle / 360; // Convert angle (degrees) to servo position (0 to 1)

        return servoPosition;

        //runFlywheel(launchVelocity);
    }

    public void runFlywheel(double launchVelocity) {
        double launchTPS = (launchVelocity * 28)/(7.2*Math.PI);
        flywheelAController.setSetPoint(launchTPS);
        flywheelBController.setSetPoint(launchTPS);
    }
    public void feed() {
        feeder.setPower(1);
    }

    public void stopFeeding() {
        feeder.setPower(0);
    }

    public void stopFlywheel() {
        flywheelA.setPower(0);
        flywheelB.setPower(0);
    }

    public void fullStop() {
        feeder.setPower(0);
        flywheelA.setPower(0);
        flywheelB.setPower(0);
    }
    
}