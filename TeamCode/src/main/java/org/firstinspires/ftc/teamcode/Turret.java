package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Config
public class Turret {
    public double tx = 58; //in The distance from the field wall to the hives along the x axis
    public double tay = 52; //cm The distance from the field wall to the first hive along the y axis
    public double tby = 92; //cm The distance from the field wall to the second hive along the y axis
    public double centerline = 72; //cm The center of the field, where the turret should switch its target to the other side's hive

    public double hoodServoMax = .7762; //
    public double hoodServoMin = .5; //
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

    private double lastX = 0;
    private double lastY = 0;
    private double lastHeading = 0;

    public double nextX;
    public double nextY;
    public double nextHeading;
    public ElapsedTime loopTime;
    public double lastTime;
    public double currentTime;
    public double deltaT;
    public double xVelocity;
    public double yVelocity;
    public double headingVelocity;
    public double nextVelocity;
    public double lastVelocity = 0;
    public double acceleration;

    public double deltaX;
    public double deltaY;
    public double deltaHeading;
    public double deltaVelocity;

    private double turretTargetAngle;
    private DcMotorEx flywheelA;
    private DcMotorEx flywheelB;
    private DcMotorEx feeder;

    public Servo turretAzimuthA;
    public Servo turretAzimuthB;
    public Servo shooterAngleA;
    public Servo shooterAngleB;
    public static double launchVelocityMultiplier = 2.57;

    private double kP = 6.411;
    private double kF = .903;
    private double kD = 5.4;


    public Turret(HardwareMap hardwareMap) {

        flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");

        // Configure shooter motors, servos, and PIDF controllers
        flywheelA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelA.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, kD, kF);
        flywheelA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        this.turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        this.turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");
        this.shooterAngleA  = hardwareMap.get(Servo.class, "shooterAngleA");
        this.shooterAngleB  = hardwareMap.get(Servo.class, "shooterAngleB");
        this.feeder         = hardwareMap.get(DcMotorEx.class, "feeder");
    }

    public void initialize() {
        loopTime.reset();

        flywheelA.setVelocity(0);
        flywheelB.setVelocity(0);

        feeder.setPower(0);

        turretAzimuthA.setPosition(.5);
        turretAzimuthB.setPosition(.5);

        shooterAngleA.setPosition(.5);
        shooterAngleB.setPosition(.5);
    }

    public double target(double rx, double ry, double heading, double vx) {
        // Get the current to calculate the elapsed time from the last loop
        currentTime = loopTime.now(TimeUnit.MICROSECONDS);
        deltaT = currentTime-lastTime;

        // Call the method for predictive SOTM tracking. Function explained in its declaration
        calculateNextPose(lastX, lastY, lastHeading, lastVelocity, rx, ry, heading, vx, deltaT);

        // Figure out which hive to target based on the robot's position
        if (nextX <= centerline) {
            ty = tay;
        }
        else if (nextY > centerline) {
            ty = tby;
        }

        distance = Math.sqrt(Math.pow((tx-nextX), 2) + Math.pow((ty-nextY), 2)); // Calculate the distance to the correct target using the formula for distance between two points
        absAngle = Math.toDegrees(Math.atan2((ty-nextY), (tx-nextX))); // Calculate the angle from the robot's pose to the hive
        targetAngle = nextHeading - absAngle + 180; // Calculate the turret target angle relative to the chassis heading

        // Use the trajectory class to calculate the required launch velocity and angle to goal
        launchVelocity = Trajectory.CalculateVelocity(distance);
        launchAngle = Trajectory.CalculateAngle(distance);

        //Calculate the aim offset based on the robot's translational velocity
        double velocityOffset = Math.atan(nextVelocity/(launchVelocity * Math.cos(launchAngle)));

        targetAngle += Math.toDegrees(velocityOffset); // Adjust the target angle based on the velocity offset

        if (targetAngle >= 360) {
            targetAngle = 180; // Wrap around if the angle exceeds 360 degrees
        } else if (targetAngle <= 0) {
            targetAngle = 180; // Wrap around if the angle is negative
        }

        double servoPosition = targetAngle / 360; // Convert angle (degrees) to servo position (0 to 1)

        setHoodAngle(launchAngle); // Runs the hood servos to the calculated launch angle
        runFlywheel(launchVelocity); // Runs the flywheel motors to the calculated launch velocity

        // Set all current values used to their last counterparts for calculations in the next loop
        lastTime = currentTime;
        lastX = rx;
        lastY = ry;
        lastHeading = heading;
        lastVelocity = vx;

        return servoPosition;
    }

    private void calculateNextPose(double lastX, double lastY, double lastHeading, double lastVelocity, double x, double y, double heading, double velocity, double deltaT){
        /*
        This method uses the "last" and "current" robot poses and X velocities to predict what the robot's position is going to be the next loop, to
        compensate for system latency (both hardware and software).
         */

        xVelocity = (x-lastX)/deltaT; // inches per microsecond
        yVelocity = (y-lastY)/deltaT; // inches per microsecond
        headingVelocity = (heading-lastHeading)/deltaT; // radians per microsecond
        acceleration = (velocity-lastVelocity)/deltaT; // cm per microsecond^2

        deltaX = xVelocity*deltaT;
        deltaY = yVelocity*deltaT;
        deltaHeading = headingVelocity*deltaT;
        deltaVelocity = acceleration*deltaT;

        nextX = deltaX + x;
        nextY = deltaY + y;
        nextHeading = deltaHeading + heading;
        nextVelocity = deltaVelocity + velocity;
    }

    private void runFlywheel(double launchVelocity) {
        /*
        The following line converts the output of the Trajectory class (cm/s) into output acceptable to the flywheel PIDF controller (ticks per second).
        Note that because we are using a hooded shooter, the linear velocity of the pollen (launch velocity) is exactly half of the tangential
        velocity of the flywheel due to rolling ball physics. This is already taken into account.
         */
        double launchTPS = ((launchVelocity * 28)/(7.2*Math.PI))*launchVelocityMultiplier;

       // double launchTPS = (launchVelocity * launchAdditive)/(7.2*Math.PI);

        // Set the flywheel motor controllers to the correct calculated velocity
        flywheelA.setVelocity(launchTPS);
        flywheelB.setVelocity(launchTPS);
    }
    private void setHoodAngle(double launchAngle) {
        // Maps the launch angle input to the corresponding value within the hood servos' range
        hoodServoSetpoint = ((launchAngle - shooterMinAngle) / (shooterMaxAngle - shooterMinAngle)) * (hoodServoMax - hoodServoMin) + hoodServoMin;

        // Sets both hood servos to the calculated position
        shooterAngleA.setPosition(hoodServoSetpoint);
        shooterAngleB.setPosition(hoodServoSetpoint);
    }

    public void stopFlywheel() {
        flywheelA.setVelocity(0);
        flywheelB.setVelocity(0);
    }
}
