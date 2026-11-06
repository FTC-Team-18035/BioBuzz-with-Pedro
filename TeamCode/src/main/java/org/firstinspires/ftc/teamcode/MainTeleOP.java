package org.firstinspires.ftc.teamcode;

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

    private Servo flowerPickerA;
    private Servo flowerPickerB;

    @Override
    public void runOpMode() {
        // Create the Pedro follower
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        follower.setPose(new Pose(0,0,Math.toRadians(90))); // Make sure this is the robot's actual starting pose

        Turret.Initialize(); // Zero the turret and hood

        waitForStart();

        while (opModeIsActive()) {

            Turret.target(follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()), follower.localizer.velocity().vy ); // Run SOTM

            // Calculate drive powers for field centric control based on joystick input and robot heading, using Pedro's field centric driving
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );

            ManualDrive.driveOrHold(follower, powers);
            follower.update();
        }
    }
}