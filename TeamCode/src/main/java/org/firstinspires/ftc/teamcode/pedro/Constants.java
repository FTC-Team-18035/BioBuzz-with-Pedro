package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {

    // These are DECODE numbers to avoid project errors for the variables in the StarterTeleOp

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftDT");
        c.frontRightName.set("frontRightDT");
        c.backLeftName.set("backLeftDT");
        c.backRightName.set("backRightDT");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(false);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.189431528406819);
        c.yPodOffset.set(3.469794866606945);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3451635370225225);
                Controller secondaryTranslationalForward = Controller.proportional(0.12752870399025154);
                Controller primaryTranslationalLateral = Controller.proportional(0.5280537579034965);
                Controller secondaryTranslationalLateral = Controller.proportional(0.19510175368906582);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013358326974212539));
                c.brake.set(Controller.proportionalFeedforward(0.011354577928080659));

                c.headingFeedback.set(Controller.proportional(4.518371800146433));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.050133664968941144, 0.005514599530000562));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0847814368019726, 0.01694977551144783));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012769464847657932, 0.0026724861350045457));

                c.maxAchievableForwardVelocity.set(80.28876821101876);
                c.maxAchievableStrafeVelocity.set(65.28577498609434);
                c.naturalForwardDeceleration.set(55.43061297188598);
                c.naturalStrafeDeceleration.set(81.01096585829458);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );

    }

}