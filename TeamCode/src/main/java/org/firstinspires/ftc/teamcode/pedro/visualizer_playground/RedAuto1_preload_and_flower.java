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
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class RedAuto1_preload_and_flower extends LinearOpMode {
    private Follower follower;
    private Turret turret;
    private FlowerPicker flowerPicker;
    private DcMotorEx intake = hardwareMap.get(DcMotorEx.class,"intake");

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Start position is touching the wall and aimed at the Red Hive
    private final Pose start = poseFactory.of(START_X, START_Y, START_HEADING);
    // Move away from wall and prepare to shoot preload
    private final Pose shoota = poseFactory.of(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING);
    // Proceed to flower picker in an arc
    private final Pose alignFlower = poseFactory.of(ALIGNFLOWER_X, ALIGNFLOWER_Y, ALIGNFLOWER_HEADING);
    private final Pose point2Control1 = poseFactory.of(57.8932, 35.4753, 0);
    private final Pose point2Control2 = poseFactory.of(39.383, 46.9364, 0);
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
                instant (()-> turret.target(SHOOTA_X,SHOOTA_Y,SHOOTA_HEADING,0)),  // This is called a lambda
                waitMs(250),  // waits for all balls to exit
                instant(()-> turret.stopFlywheel()),  // shuts down flywheel to save battery power

                follow(follower, path2()),
                // lower flower picker
                instant(()-> flowerPicker.lower()),
                waitMs(200),
                instant(()-> intake.setPower(1)),
                // missing feeder actions here

                follow(follower, path3()),
                // engage flower picker for collection
                waitMs(500),
                instant(()-> intake.setPower(0)),

                follow(follower, path4()),
                instant(()-> flowerPicker.raise()),
                waitMs(100),


                follow(follower,path5()),
                // shooting actions again
                instant (()-> turret.target(SHOOTA_X,SHOOTA_Y,SHOOTA_HEADING,0)),  // This is called a lambda
                waitMs(250),  // waits for all balls to exit
                instant(()-> turret.stopFlywheel())  // shuts down flywheel to save battery power

                // end of autonomous actions
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        // duplicate telemetry to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

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

            telemetry.update();
        }
        // Store final follower pose to be read in teleOp and toggle opmode tracker
        StoredData.lastAutoPose = follower.pose();
        StoredData.autoWasRun = true;
    }

    public Path path1() {
        return Paths.line(start, shoota).linear(start, shoota);
    }

    public Path path2() {
        return Paths.curve(shoota, point2Control1, point2Control2, alignFlower).tangent(); }

    public Path path3() {
        return Paths.line(alignFlower, pickFlower).constant(pickFlower);
    }

    public Path path4() {
        return Paths.line(pickFlower, alignFlower).linear(pickFlower, alignFlower);
    }

    public Path path5() {
        return Paths.line(alignFlower, shootSecondTime).linear(alignFlower, shootSecondTime);
    }
}


