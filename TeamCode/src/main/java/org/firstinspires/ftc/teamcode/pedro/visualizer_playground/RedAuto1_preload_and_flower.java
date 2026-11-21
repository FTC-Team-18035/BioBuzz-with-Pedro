package org.firstinspires.ftc.teamcode.pedro.visualizer_playground;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.ALIGNFLOWER_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.ALIGNFLOWER_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.ALIGNFLOWER_Y;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.PICKFLOWER_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.PICKFLOWER_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.PICKFLOWER_Y;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.SHOOTA_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.SHOOTA_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.SHOOTA_Y;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.START_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.START_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.START_Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.FlowerPicker;
import org.firstinspires.ftc.teamcode.StoredData;
import org.firstinspires.ftc.teamcode.indevelopment.TurretWithOldCode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config // allow FTCDashboard editing of any public static variables
@Autonomous(name = "AutoPath1", group = "Autonomous")
public class RedAuto1_preload_and_flower extends LinearOpMode {
    private Follower follower;
    private TurretWithOldCode turret;
    private FlowerPicker flowerPicker;
    private DcMotorEx feeder = hardwareMap.get(DcMotorEx.class, "feeder");
    private DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"intake");

    public static double preloadShootTime = 500; // ms delay of auto routine to shoot preload
    public static double flowerPickTime = 1000; // ms delay of auto routine to pick flower
    public static double moveFlowerPicker = 200; // ms delay of auto routine to lower/raise flowerpicker

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Start position is touching the wall and aimed at the Red Hive
    private final Pose start = poseFactory.of(START_X, START_Y, START_HEADING);
    // Move away from wall and prepare to shoot preload
    private final Pose shoota = poseFactory.of(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING);
    // Proceed to flower picker in an arc
    private final Pose alignFlower = poseFactory.of(ALIGNFLOWER_X, ALIGNFLOWER_Y, ALIGNFLOWER_HEADING);
    private final Pose point2Control1 = poseFactory.of(58, 36, 0);
    private final Pose point2Control2 = poseFactory.of(39, 47, 0);
    // Proceed to Flower after picker/intake engaged
    private final Pose pickFlower = poseFactory.of(PICKFLOWER_X, PICKFLOWER_Y, PICKFLOWER_HEADING);
    // Return to shooting position in front of Hive
    private final Pose point4Start = poseFactory.of(17, 47, 180);  // redundant - alignFlower
    private final Pose shootSecondTime = poseFactory.of(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),

                // shooting actions here - these are written as Commands instead of our normal JAVA
                instant(()-> { turret.target(SHOOTA_X,SHOOTA_Y,SHOOTA_HEADING,0);
                               feeder.setPower(1);}),
                waitMs(preloadShootTime),  // waits for all balls to exit
                instant(()-> turret.stopFlywheel()),  // shuts down flywheel to save battery power

                follow(follower, path2()),
                // lower flower picker
                instant(()-> flowerPicker.lower()),
                waitMs(flowerPickTime),
                instant(()-> { intake.setPower(1);
                               feeder.setPower(1);}),

                follow(follower, path3()),
                // engage flower picker for collection
                waitMs(moveFlowerPicker),
                instant(()-> { intake.setPower(0);
                               feeder.setPower(0);}),

                follow(follower, path4()),
                instant(()-> flowerPicker.raise()),
                waitMs(moveFlowerPicker),


                follow(follower,path5()),
                // shooting actions again
                instant(()-> { turret.target(SHOOTA_X,SHOOTA_Y,SHOOTA_HEADING,0);
                               feeder.setPower(1);}),
                waitMs(preloadShootTime),               // waits for all balls to exit
                instant(()-> { turret.stopFlywheel();   // shuts down flywheel to save battery power
                               feeder.setPower(0);})    // shuts down feeder to end routine

                // end of autonomous actions
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        // duplicate telemetry to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // create Pedro follower
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();
        // Create the turret object
        turret = new TurretWithOldCode(hardwareMap, telemetry);
        // Create flower picker object
        flowerPicker = new FlowerPicker(hardwareMap);

        intake.setPower(0);
        feeder.setPower(0);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }
            // Update telemetry from classes
            turret.updateTelemetry();

            telemetry.update();
        }

        intake.setPower(0);
        feeder.setPower(0);
        // Store final follower pose to be read in teleOp and toggle opmode tracker
        StoredData.lastAutoPose = follower.pose();
        StoredData.autoWasRun = true;
    }

    public Path path1() {   // MOVE and score
        return Paths.line(start, shoota).linear(start, shoota);
    }

    public Path path2() {   // align to flower before lowering flowerpicker
        return Paths.curve(shoota, point2Control1, point2Control2, alignFlower).tangent(); }

    public Path path3() {   // move to pick flower
        return Paths.line(alignFlower, pickFlower).constant(pickFlower);
    }

    public Path path4() {   // move away from flower before
        return Paths.line(pickFlower, alignFlower).linear(pickFlower, alignFlower);
    }

    public Path path5() {   // return to score in hiveA
        return Paths.line(alignFlower, shootSecondTime).linear(alignFlower, shootSecondTime);
    }
}


