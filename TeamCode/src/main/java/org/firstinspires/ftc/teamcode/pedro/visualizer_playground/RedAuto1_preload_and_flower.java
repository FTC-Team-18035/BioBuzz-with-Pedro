package org.firstinspires.ftc.teamcode.pedro.visualizer_playground;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class RedAuto1_preload_and_flower extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Start position is touching the wall and aimed at the Red Hive
    private final Pose start = poseFactory.of(58.468, 7.7943, 90);
    // Move away from wall and prepare to shoot preload
    private final Pose path1 = poseFactory.of(58.4113, 16.5196, 90);
    // Proceed to flower picker in an arc
    private final Pose point2 = poseFactory.of(22.6693, 46.9313, -179.9826);
    private final Pose point2Control1 = poseFactory.of(57.8932, 35.4753, 0);
    private final Pose point2Control2 = poseFactory.of(39.383, 46.9364, 0);
    // Proceed to Flower after picker/intake engaged
    private final Pose point3 = poseFactory.of(17, 46.9673, 180);
    // Return to shooting position in front of Hive
    private final Pose point4Start = poseFactory.of(17, 46.9673, 180);
    private final Pose point4 = poseFactory.of(58, 29, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4())
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
    }

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }
}


