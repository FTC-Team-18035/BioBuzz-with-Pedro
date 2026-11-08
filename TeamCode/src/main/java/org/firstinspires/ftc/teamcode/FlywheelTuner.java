package org.firstinspires.ftc.teamcode;

import android.net.LinkAddress;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp
public class FlywheelTuner extends LinearOpMode {
    private DcMotorEx flywheelA;
    private DcMotorEx flywheelB;

    private double kP = 0; // Proportional gain for shooter TODO: Tune
    private double kF = 0; // Feedforward gain for shooter TODO: Tune
    private double curTargetVelocity;
    private double highVelocity = 2000;
    private double lowVelocity = 900;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    public void runOpMode() {

        flywheelA= hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB= hardwareMap.get(DcMotorEx.class, "flywheelB");

        flywheelA.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelA.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
        flywheelA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();

        // Main loop
        while(opModeIsActive()) {
            // Main code goes here, loops while opmode is running
            if (gamepad1.yWasPressed()) {
                if (curTargetVelocity == highVelocity) {
                    curTargetVelocity = lowVelocity;
                } else { curTargetVelocity = highVelocity; }
        }
            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (gamepad1.dpadLeftWasPressed()) {
                kF -= stepSizes[stepIndex];
            }
            if (gamepad1.dpadRightWasPressed()) {
                kF += stepSizes[stepIndex];
            }
            if (gamepad1.dpadDownWasPressed()) {
                kP += stepSizes[stepIndex];
            }
            if (gamepad1.dpadUpWasPressed()) {
                kP -= stepSizes[stepIndex];
            }

            pidfCoefficients = new PIDFCoefficients(kP, 0, 0, kF);
            flywheelA.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            flywheelB.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            flywheelA.setVelocity(curTargetVelocity);
            flywheelB.setVelocity(curTargetVelocity);

            double curVelocity = flywheelA.getVelocity();
            double error = curTargetVelocity - curVelocity;

            telemetry.addData("Target Velocity", curTargetVelocity);
            telemetry.addData("Current Velocity", "%.2f", curVelocity);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine("------------------------------");
            telemetry.addData("Tuning kP", "%.4f (D-Pad U/D)", kP);
            telemetry.addData("Tuning kF", "%.4f (D-Pad L/R)", kF);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.update();
    }
}}