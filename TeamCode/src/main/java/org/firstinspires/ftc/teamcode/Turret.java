package org.firstinspires.ftc.teamcode;

//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;
public class Turret {
        private static final double SERVO_MIN = 0.1809;
        private static final double SERVO_MAX = 0.2306;
        private final DcMotorEx flywheelA;
        private final DcMotorEx flywheelB;
        private final Servo shooterAngleA;
        private final Servo shooterAngleB;
        //private final PIDFController leftController;
        //private final PIDFController rightController;
        private double targetVelocity;
        private double currentServoPosition;

        public Turret(HardwareMap hardwareMap) {
            flywheelA = hardwareMap.get(DcMotorEx.class, "left_shooter");
            flywheelB = hardwareMap.get(DcMotorEx.class, "right_shooter");
            shooterAngleA = hardwareMap.get(Servo.class, "shooter_horizontal");
            shooterAngleB = hardwareMap.get(Servo.class, "shooter_vertical");
            //leftController = new PIDFController(new PIDFCoefficients(0.005, 0, 0, 0.000544));
            //rightController = new PIDFController(new PIDFCoefficients(0.005, 0, 0, 0.000544));
            configureMotors();
            setServoPosition(0.205); //TODO determine servo init position
            targetVelocity = 0.0;
        }
    //Trajectory calculation based on targetDistance
    //-----------------------------------------------------
    public static double[] Calculate(double targetDistance) {
        double distance = targetDistance + 15;
        // define values used for all calculations
        double g = 980.0; // cm/s^2 magnitude of acceleration due to gravity
        double y0 = 53.0; // cm, target final height in goal
        double m = Math.min(-1.5, (-200.0 - y0) / distance); // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE
        double a = -(y0 / Math.pow(distance, 2)) + (m / distance); // The "a" value in the parabola equation
        double b = ((2.0 * y0) / distance) - m; // The "b" value in the parabola equation
        double rawlaunchAngle = Math.atan(b); // The launch angle in radians
        //Degraded (hopefully) double launchVelocity = (1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a));
        double launchVelocity = ((1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a))) * 1.23787177960363;  //12.21730476; //The launch velocity in cm/s

        double launchAngle = Math.toDegrees(rawlaunchAngle) * .00392157;
        return new double[]{launchVelocity, launchAngle}; // these are the target for the shooter's flywheel and servo angle adjustments
    }

    //-----------------------------------------------------
    private void configureMotors() {
        DcMotorEx[] motors = new DcMotorEx[]{flywheelA, flywheelB};
        for (DcMotorEx motor : motors) {
            MotorConfigurationType config = motor.getMotorType().clone();
            config.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(config);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double clampServo(double position) {
        return Range.clip(position, SERVO_MIN, SERVO_MAX);
    }

    public void setServoPosition(double position) {           //Use for hood angle adjustment
        currentServoPosition = clampServo(position);
        shooterAngleA.setPosition(currentServoPosition);
        shooterAngleB.setPosition(currentServoPosition);
    }

    public void setTarget(double velocity, double anglePosition) {

        targetVelocity = velocity;
        setServoPosition(anglePosition);
    }

    public void stop() {
        targetVelocity = 0.0;
        flywheelA.setPower(0.0);
        flywheelB.setPower(0.0);
        //leftController.reset();
        //rightController.reset();
        //setFeedPower(0);
    }

    /*public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        // Apply identical gains to both controllers (clone to avoid shared state).
        leftController.setCoefficients(new PIDFCoefficients(coefficients.P, coefficients.I, coefficients.D, coefficients.F));
        rightController.setCoefficients(new PIDFCoefficients(coefficients.P, coefficients.I, coefficients.D, coefficients.F));
    }*/

   /* public void resetControllers() {
        leftController.reset();
        rightController.reset();
    }*/

    public double getVelocityA() {
        return flywheelA.getVelocity();
    }

    public  double getVelocityB() {
        return flywheelB.getVelocity();
    }
    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getServoPosition() {
        return currentServoPosition;
    }

    public void update() {
        //leftController.setTargetPosition(targetVelocity);
       //rightController.setTargetPosition(-targetVelocity);
        //leftController.updateFeedForwardInput(targetVelocity);
        //rightController.updateFeedForwardInput(-targetVelocity);
        //leftController.updatePosition(leftShooter.getVelocity());
        //rightController.updatePosition(rightShooter.getVelocity());
        //double leftPower = Range.clip(leftController.run(), -1.0, 1.0);
        //double rightPower = Range.clip(rightController.run(), -1.0, 1.0);
        //flywheelA.setPower(leftPower);
        //flywheelB.setPower(rightPower);
        // Keep servos locked to the last commanded, clamped position for safety
        shooterAngleA.setPosition(currentServoPosition);
        shooterAngleB.setPosition(currentServoPosition);
    }
}

//TODO add a method for a rotating turret that will line up with the correct hive cell using limelight
//TODO if under any circumstances that the april tag is not visible, stop the shooter
//TODO add a timer where if the april tag is not visible foe x amount of time, use pinpoint to face centerfield
//TODO prevent wraparound

    /**
     * Shooter subsystem handling dual flywheel motors and angle servos.
     * Target velocity is applied to both motors independently with separate PIDF controllers.
     */
