package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    @Tuner
    // STEP 1 - comment out and uncomment next STEP when completed
    public static Procedure mecanumTuner() { return new MecanumTuner();}

    // STEP 2 - comment out and uncomment next STEP when completed
    //@Tuner
    //public static Procedure tests() { return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);    }

    // STEP 3 - comment out and uncomment next STEP when completed
    //@Tuner
    //public static Procedure pinpointTuner() { return new PinpointTuner();    }

    // STEP 4 - comment out and uncomment next STEP when completed
    //@Tuner
    //public static Procedure foresightTuner() { return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));}


}
