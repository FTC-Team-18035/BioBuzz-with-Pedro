package org.firstinspires.ftc.teamcode.indevelopment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.FlowerPicker;
import org.firstinspires.ftc.teamcode.StoredData;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class Limelight_TestTeleOP extends LinearOpMode {
    private boolean flowerPickerState = false; // True = down, false = up
    private long lastTagSeenTimeMs = 0;

    private Turret turret;
    private Follower follower;
    private Limelight3A limelight;
    private FlowerPicker flowerPicker;

    // Define motors and servos
    private DcMotorEx intake;
    private DcMotorEx feeder;
    //private Servo flowerPickerA;
    //private Servo flowerPickerB;


    @Override
    public void runOpMode() {
        // duplicate telemetry to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Create the Pedro follower object
        follower = Constants.create(hardwareMap);

        // Create the turret object
        turret = new Turret(hardwareMap, telemetry);
        // Get limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0); // AprilTag pipeline TODO Field map and April Tag Cluster pipeline

        // Configure motors and servos for use
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        if (StoredData.autoWasRun) {
            follower.setPose(StoredData.lastAutoPose); // use last pose of Auto to set starting position
        } else {
            // this pose is centered and facing the near hive
            follower.setPose(new Pose(58, 8, Math.toRadians(90))); // Set robot's starting pose on field TODO - Make automatic using Limelight
        }

        turret.initialize(); // Zero the turret and hood - 0 power feeder

        intake.setPower(0);
        feeder.setPower(0);

        // Raise the flower picker for start
        flowerPicker.raise();

        waitForStart();

        StoredData.autoWasRun = false; // toggle
        limelight.start(); // TODO - determine when (if ever) to have limelight off




            while (opModeIsActive()) {
                // Poll limelight
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                        double lx = result.getTx();
                        double ly = result.getTy();
                        telemetry.addData("lx", lx);
                        telemetry.addData("ly", ly);
                        telemetry.addData("Botpose", botpose.toString());
                    }
                }

                // Get the robot's current pose and velocity for SOTM
                double rx = follower.localizer.pose().x();
                double ry = follower.localizer.pose().y();
                double heading = Math.toDegrees(follower.localizer.pose().heading());
                double vx = follower.localizer.velocity().vx;

                boolean tagValid = result != null && result.isValid()
                        && !result.getFiducialResults().isEmpty();

                // Limelight logic - use to refine targeting?
                //                  determine which side of hive is raised
                //                  set robot pose at match beginning
                //                  reset pose when at a strong location and button pushed

                if (tagValid) {   // set tag timestamp
                    lastTagSeenTimeMs = System.currentTimeMillis();
                }

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
                // Update flower picker position
                if (flowerPickerState) {
                    flowerPicker.lower();
                } else {
                    flowerPicker.raise();
                }

                // Handle feeder
                if (gamepad1.left_trigger_pressed) {
                    feeder.setPower(1);
                } else {
                    feeder.setPower(0);
                }

                // Handle intake
                if (gamepad1.left_bumper) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                // Handle intake and feeder simultaneously
                if (gamepad1.right_trigger_pressed) {
                    intake.setPower(1);
                    feeder.setPower(1);
                } else {
                    intake.setPower(0);
                    feeder.setPower(0);
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