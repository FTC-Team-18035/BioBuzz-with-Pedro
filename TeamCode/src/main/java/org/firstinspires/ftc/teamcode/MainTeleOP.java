package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
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
        // Create the Pedro follower

        follower = Constants.create(hardwareMap);

        Turret = new Turret(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");

        follower.setPose(new Pose(72,7,Math.toRadians(90))); // Make sure this is the robot's actual starting pose

        Turret.initialize(); // Zero the turret and hood

        waitForStart();

        while (opModeIsActive()) {
            double rx = follower.localizer.pose().x();
            double ry = follower.localizer.pose().y();
            double heading = Math.toDegrees(follower.localizer.pose().heading());
            double vy = follower.localizer.velocity().vy;

            double turretAzimuthPosition = Turret.target(rx, ry, heading, vy); // Run SOTM
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

            if (gamepad1.left_bumper) {
                intake.setPower(1);
            }
            else if (gamepad1.left_trigger_pressed) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }

            if (gamepad1.right_bumper) {
                feeder.setPower(1);
            }
            else if (gamepad1.right_trigger_pressed) {
                feeder.setPower(-1);
            }
            else {
                feeder.setPower(0);
            }

            telemetry.addData("Robot heading", heading);
            telemetry.addData("Turret commanded position", turretAzimuthPosition);
            telemetry.update();

            ManualDrive.driveOrHold(follower, powers);
            follower.update();
            follower.localizer.update();
        }
    }
}