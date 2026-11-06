package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This Utility class is to test non drivetrain motors and servos and display current positions
 */

@Utility (name="Motor/Servo Operation and Value Display")
public class ServoPositionDisplay extends LinearOpMode {

    private DcMotorEx intake;
    private DcMotorEx feeder;
    private Servo flowerPickerA;
    private Servo flowerPickerB;
    private Servo turretAzimuthA;
    private Servo turretAzimuthB;
    private Servo shooterAngleA;
    private Servo shooterAngleB;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feeder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretAzimuthA = hardwareMap.get(Servo.class, "turretAzimuthA");
        turretAzimuthB = hardwareMap.get(Servo.class, "turretAzimuthB");
        turretAzimuthB.setDirection(REVERSE);

        shooterAngleA = hardwareMap.get(Servo.class, "shooterAngleA");
        shooterAngleB = hardwareMap.get(Servo.class, "shooterAngleB");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        // initial servo starting positions
        initialize();

        // display instructions on driver hub
        telemetry.addLine("Hold X button to run intake/feeder");
        telemetry.addLine("Use left stick x to aim the turret");
        telemetry.addLine("Use right stick y forward to lower the flower picker");
        telemetry.addLine("Use left trigger for shooter angle");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // get controller input
            double positionturret = (gamepad1.left_stick_x + 1) / 2;
            double positionpicker = Math.min(0, -gamepad1.right_stick_y);
            float positionangle = gamepad1.left_trigger;
            double powerin;
            if (gamepad1.x) { powerin = 1; } else { powerin = 0; }

            // translate values to servo positions
            turretAzimuthA.setPosition(positionturret);
            turretAzimuthB.setPosition(positionturret);

            shooterAngleA.setPosition(positionangle);
            shooterAngleB.setPosition(positionangle);

            flowerPickerA.setPosition(positionpicker);
            flowerPickerB.setPosition(positionpicker);

            intake.setPower(powerin);
            feeder.setPower(powerin);

            // display values on driver station
            telemetry.addData("turretA Position", turretAzimuthA.getPosition());
            telemetry.addData("turretB Position", turretAzimuthB.getPosition());
            telemetry.addData("shooterA Position", shooterAngleA.getPosition());
            telemetry.addData("shooterB Position", shooterAngleB.getPosition());
            telemetry.addData("flowerPickerA Position", flowerPickerA.getPosition());
            telemetry.addData("flowerPickerB Position", flowerPickerB.getPosition());
            telemetry.update();

        }

        // return to start positions for shutdown
        initialize();

    }

    private void initialize() {
        // initial servo starting positions
        intake.setPower(0);
        feeder.setPower(0);
        turretAzimuthA.setPosition(.5);
        turretAzimuthB.setPosition(.5);
        shooterAngleA.setPosition(0);
        shooterAngleB.setPosition(0);
        flowerPickerA.setPosition(0);
        flowerPickerB.setPosition(0);
    }

}
