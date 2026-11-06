package org.firstinspires.ftc.teamcode.indevelopment;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class Intake2_commitTest extends LinearOpMode {
    DcMotor intake;

    public void runOpMode() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                intake.setPower(1.0);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0.0);
            }
        }
    }
}
//This is not the real intake; this is just to see if commiting works....