package org.firstinspires.ftc.teamcode.pedro.visualizer_playground;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    private final Pose start = poseFactory.of(58, 8, 90);
    // Move away from wall and prepare to shoot preload
    private final Pose shoota = poseFactory.of(58, 16.5, 90);
    // Proceed to flower picker in an arc
    private final Pose point2 = poseFactory.of(22.6693, 47, -179.9826);
    private final Pose point2Control1 = poseFactory.of(57.8932, 35.4753, 0);
    private final Pose point2Control2 = poseFactory.of(39.383, 46.9364, 0);
    // Proceed to Flower after picker/intake engaged
    private final Pose nearFlower = poseFactory.of(17, 47, 180);
    // Return to shooting position in front of Hive
    private final Pose point4Start = poseFactory.of(17, 47, 180);  // redundant - nearFlower
    private final Pose point4 = poseFactory.of(58, 29, 90);


    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),

                // shooting actions here - these are written as Commands instead of our normal JAVA
                instant (()-> turret.target(58,16.5,90,0)),  // This is called a lambda
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
                instant (()-> turret.target(58,29,90,0)),  // This is called a lambda
                waitMs(250),  // waits for all balls to exit
                instant(()-> turret.stopFlywheel())  // shuts down flywheel to save battery power

                // end of autonomous actions
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
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
        return Paths.curve(shoota, point2Control1, point2Control2, point2).tangent(); }

    public Path path3() {
        return Paths.line(point2, nearFlower).constant(nearFlower);
    }

    public Path path4() {
        return Paths.line(nearFlower, point2).linear(nearFlower, point2);
    }

    public Path path5() {
        return Paths.line(point2, point4).linear(point2, point4);
    }
}


