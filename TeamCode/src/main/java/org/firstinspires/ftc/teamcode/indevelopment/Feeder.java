package org.firstinspires.ftc.teamcode.indevelopment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Utility

public class Feeder extends LinearOpMode {
    DcMotor feeder;
    @Override
    public void runOpMode() {
        feeder = hardwareMap.get(DcMotor.class, "feeder");

       // public void setFeedPower (double Power) {
         //   feeder.setPower(-Power);
        }
    }

//}








    //private final Servo vaultRelease;
//private final CRServo vaultFeed;
//private final CRServo vaultFeed2;

/*vaultRelease = hardwareMap.get(Servo.class, "vault_release");
vaultFeed = hardwareMap.get(CRServo.class, "vault_feed");
vaultFeed2 = hardwareMap.get(CRServo.class, "vault_feed2");*/

   /* public void setFeedPower(double power) {
        vaultFeed.setPower(-power);
        vaultFeed2.setPower(-power);
    }
}*/
//TODO need methods to turn feeder forward/off
//TODO add a reverse feeder method