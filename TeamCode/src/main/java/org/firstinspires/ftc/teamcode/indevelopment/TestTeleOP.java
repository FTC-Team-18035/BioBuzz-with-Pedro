package org.firstinspires.ftc.teamcode.indevelopment;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Trajectory;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Flower picker test control")

public class TestTeleOP extends LinearOpMode {
    // unused hardware should be inactivated
    //private Follower follower;

    // Define motors and servos
    private Servo flowerPickerA;
    private Servo flowerPickerB;  /*
    private Servo turretAzimuthA;
    private Servo turretAzimuthB;
    private Servo shooterAngleA;
    private Servo shooterAngleB;
    private DcMotorEx intake;
    private DcMotorEx feeder;
    private Limelight3A limelight;  */


    @Override
    public void runOpMode() {
        // Create the Pedro follower

        //follower = Constants.create(hardwareMap);

        //intake = hardwareMap.get(DcMotorEx.class, "intake");
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        //feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        //turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");
        //turretAzimuthB.setDirection(Servo.Direction.REVERSE);

        //shooterAngleA = hardwareMap.get(Servo.class, "shooterAngleA");
        //shooterAngleB = hardwareMap.get(Servo.class, "shooterAngleB");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        //turretAzimuthA.setPosition(.5);
        //turretAzimuthB.setPosition(.5);

        //shooterAngleA.setPosition(0);
        //shooterAngleB.setPosition(0);

        flowerPickerA.setPosition(.5);  // set to safe midpoint position
        flowerPickerB.setPosition(.5);
        /**
         * limelight = hardwareMap.get(Limelight3A.class, "limelight");
         * limelight.pipelineSwitch(6); // AprilTag pipeline
         * limelight.start();
        */
        // TODO: Add Limelight, will be using placeholder for now

    waitForStart();

    while (opModeIsActive()) {

        handleFlowerpickers();

        float touch = (gamepad1.touchpad_finger_1_x + 1) / 2;
/*
        if (gamepad1.left_trigger_pressed) {
            intake.setPower(.5);
        }else {
            intake.setPower(0);
        }

        if (gamepad1.right_trigger_pressed) {
            feeder.setPower(.5);
        }else {
            feeder.setPower(0);
        }

        if (gamepad1.dpad_up) {
            turretAzimuthA.setPosition(1);
            turretAzimuthB.setPosition(1);
        }else {
            turretAzimuthA.setPosition(0);
            turretAzimuthB.setPosition(0);
        }

        if (gamepad1.dpad_left) {
            shooterAngleA.setPosition(1);
            shooterAngleB.setPosition(1);
        }else {
            shooterAngleA.setPosition(0);
            shooterAngleB.setPosition(0);
        }
*/

        // Limelight.getTX and all that good stuff
        double distance = 0.0; // Placeholder for Limelight TX value

        double LaunchVelocity= Trajectory.CalculateVelocity(distance);
        double LaunchAngle=Trajectory.CalculateAngle(distance);

        // Turret.target(Limelight.getTX(), follower.pose().velocity().x, LaunchVelocity, LaunchAngle);



        // Calculate drive powers for field centric control based on joystick input and robot heading, using Pedro's field centric driving
    /*    DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);
        follower.update();

     */


        //telemetry.addLine("Dpad up/down = turretAzimuthA/B on");
        //telemetry.addLine("Dpad left/right = shooterAngleA/B on");
        //telemetry.addLine("Trigger left/right = intake/feeder on");


        telemetry.addLine("Left stick y forward/back = flowerPicker up/down");
        telemetry.addData("Flower picker servo position",flowerPickerA.getPosition());
        telemetry.update();
    }
}


private void handleFlowerpickers() {
    float normalizedleftsticky = (gamepad1.left_stick_y + 1) / 2;
     flowerPickerA.setPosition(normalizedleftsticky);
     flowerPickerB.setPosition(normalizedleftsticky);  // set to stick value 0 to 1

     }
}