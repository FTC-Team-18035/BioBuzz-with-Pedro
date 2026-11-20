package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class MainTeleOP extends LinearOpMode {
    private boolean flowerPickerState = false; // True = down, false = up
    private double down = .2487;
    private double up = .6685;


    private Turret turret;
    private Follower follower;

    // Define motors and servos
    private DcMotorEx intake;
    private DcMotorEx feeder;

    private Servo flowerPickerA;
    private Servo flowerPickerB;


    @Override
    public void runOpMode() {
        // duplicate telemetry to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Create the Pedro follower object
        follower = Constants.create(hardwareMap);

        // Create the turret object
        turret = new Turret(hardwareMap);

        // Configure motors and servos for use
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        if (StoredData.autoWasRun) {
            follower.setPose(StoredData.lastAutoPose); // use last pose of Auto to set starting position
        } else{
            follower.setPose(new Pose(72, 7, Math.toRadians(90))); // Set robot's starting pose on field TODO: Make automatic using Limelight
        }

        turret.initialize(); // Zero the turret and hood

        // Raise the flower picker for start
        flowerPickerA.setPosition(up);
        flowerPickerB.setPosition(1-up);

        waitForStart();

            StoredData.autoWasRun = false; // toggle

        while (opModeIsActive()) {

            // Get the robot's current pose and velocity for SOTM
            double rx = follower.localizer.pose().x();
            double ry = follower.localizer.pose().y();
            double heading = Math.toDegrees(follower.localizer.pose().heading());
            double vx = follower.localizer.velocity().vx;

            turret.target(rx, ry, heading, vx); // Calculate turret angle for SOTM (pre-compensated for translational velocity)

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

            //Update flower picker position
           if (flowerPickerState) {
               flowerPickerA.setPosition(down);
               flowerPickerB.setPosition(1-down);
           }
            else {
                flowerPickerA.setPosition(up);
               flowerPickerB.setPosition(1-up);
           }

            // Update follower for field centric driving and SOTM
            ManualDrive.driveOrHold(follower, powers);
            follower.update();
            follower.localizer.update();

            // Update telemetry from classes
            turret.updateTelemetry();
            // telemetry items
            telemetry.addData("target x", rx);
            telemetry.addData("target y", ry);
            telemetry.addData("heading", heading);
            telemetry.addData("velocity",vx);
            telemetry.update();
        }
    }
}