package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class FlowerPicker extends LinearOpMode {
    Servo servoA,servoB;
    @Override
    public void runOpMode() {
        servoA = hardwareMap.get(Servo.class, "flowerPickerA");
        servoB = hardwareMap.get(Servo.class, "flowerPickerB");

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.a) {                  //flower picker servo control
                servoA.setPosition(1);        //A|B lower the flower picker
            } else {
                servoA.setPosition(0);
            }
            if (gamepad1.b) {
                servoB.setPosition(1);
            } else {
                servoB.setPosition(0);
            }
        }
    }
}
//TODO add a method for a servo  to flip to down position and to up position