package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class MainTeleOP extends LinearOpMode {

    private ElapsedTime loopTime;
    private double loopFrequency;

    private boolean flowerPickerState = false; // True = down, false = up
    private double down = .5; // TODO: Determine the actual down position for the flower picker
    private double up = .5; // TODO: Determine the actual up position for the flower picker

    private Turret Turret;
    private Follower follower;

    // Define motors and servos
    private DcMotorEx intake;
    private DcMotorEx feeder;

    private Servo flowerPickerA;
    private Servo flowerPickerB;

    private Servo turretAzimuthA;
    private Servo turretAzimuthB;


    @Override
    public void runOpMode() {
        // Create the Pedro follower object
        follower = Constants.create(hardwareMap);

        // Create the turret object
        Turret = new Turret(hardwareMap);

        // Configure motors and servos for use
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");

        follower.setPose(new Pose(72,7,Math.toRadians(90))); // Set robot's starting pose on field TODO: Make automatic using Limelight

        Turret.initialize(); // Zero the turret and hood

        // Raise the flower picker for start
        flowerPickerA.setPosition(up);
        flowerPickerB.setPosition(up);

        waitForStart();

        while (opModeIsActive()) {
            loopTime.reset();
            loopTime.startTime();

            // Get the robot's current pose and velocity for SOTM
            double rx = follower.localizer.pose().x();
            double ry = follower.localizer.pose().y();
            double heading = Math.toDegrees(follower.localizer.pose().heading());
            double vx = follower.localizer.velocity().vx;

            double turretAzimuthPosition = Turret.target(rx, ry, heading, vx); // Calculate turret angle for SOTM (pre-compensated for translational velocity)
            // Set the turret angle
            turretAzimuthA.setPosition(turretAzimuthPosition);
            turretAzimuthB.setPosition(turretAzimuthPosition);

            // Calculate drive powers for field centric control based on joystick input and robot heading, using Pedro's field centric driving
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            // Handle flower picker
            if (gamepad1.dpadUpWasPressed()) {
                flowerPickerState = !flowerPickerState;
            }

            // Handle feeder
            if (gamepad1.left_trigger_pressed) {
                feeder.setPower(1);
            }
            else {
                feeder.setPower(0);
            }

            // Handle intake
            if (gamepad1.left_bumper) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }

            // Handle intake and feeder simultaneously
            if (gamepad1.right_trigger_pressed) {
                intake.setPower(1);
                feeder.setPower(1);
            }
            else {
                intake.setPower(0);
                feeder.setPower(0);
            }

            // Update flower picker position
            if (flowerPickerState) {
                flowerPickerA.setPosition(down);
                flowerPickerB.setPosition(down);
            }
            else {
                flowerPickerA.setPosition(up);
                flowerPickerB.setPosition(up);
            }

            // Update follower for field centric driving and SOTM
            ManualDrive.driveOrHold(follower, powers);
            follower.update();
            follower.localizer.update();

            // Add telemetry for turret targeting
            telemetry.addData("Robot heading", heading);
            telemetry.addData("Turret commanded position", turretAzimuthPosition);

            // Add telemetry for looptimes
            loopFrequency = 1000/loopTime.milliseconds();
            telemetry.addData("Current loop frequency (hz)", loopFrequency);

            // Update telemetry
            telemetry.update();
        }
    }
}