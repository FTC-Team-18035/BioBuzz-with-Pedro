package org.firstinspires.ftc.teamcode.pedro.visualizer_playground;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.ALIGNFLOWERA_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.ALIGNFLOWERA_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.ALIGNFLOWERA_Y;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.PICKFLOWERA_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.PICKFLOWERA_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.PICKFLOWERA_Y;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.SHOOTA_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.SHOOTA_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.SHOOTA_Y;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.STARTA_HEADING;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.STARTA_X;
import static org.firstinspires.ftc.teamcode.pedro.visualizer_playground.CoordinatesOfValue.STARTA_Y;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config // allow FTCDashboard editing of any public static variables
@Autonomous(name = "AutoPath1", group = "Autonomous")
public class RedAuto1_preload_and_flower extends OpMode {
    /**
     * Hardware objects creation
     */
    private Follower follower;
    private Turret turret;
    private DcMotorEx feeder;
    private DcMotorEx intake;
    private Servo flowerPickerA;
    private Servo flowerPickerB;
    // private Limelight3A limelight;  // placeholder as of 11-24
    /**
     * Pretuned tuned values
     */
    private double down = .2487;  // flowerPicker tuned value
    private double up = .6685;
    /**
     * These are public static to be able to change via FTC Dashboard
     */
    public static double preloadShootTime = 500; // ms delay of auto routine to shoot preload
    public static double flowerPickTime = 1000; // ms delay of auto routine to pick flower
    public static double moveFlowerPicker = 200; // ms delay of auto routine to lower/raise flowerpicker

    /**
     * The Pedro PoseFactory creates all of the desired pose points for our actions taken
     */
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // StartA position is touching the wall and aimed at the Red Hive
    private final Pose startA = poseFactory.of(STARTA_X, STARTA_Y, STARTA_HEADING);
    // Move away from wall and prepare to shoot preload
    private final Pose shootA = poseFactory.of(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING);
    // Proceed to flower picker in an arc
    private final Pose alignFlowerA = poseFactory.of(ALIGNFLOWERA_X, ALIGNFLOWERA_Y, ALIGNFLOWERA_HEADING);
    private final Pose point2Control1 = poseFactory.of(58, 36, 0);
    private final Pose point2Control2 = poseFactory.of(39, 47, 0);
    // Proceed to Flower after picker/intake engaged
    private final Pose pickFlowerA = poseFactory.of(PICKFLOWERA_X, PICKFLOWERA_Y, PICKFLOWERA_HEADING);
    // Return to shooting position in front of Hive
    private final Pose shootSecondTime = poseFactory.of(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING);


    // Autonomous routine command sequence
    public Command autoRoutine() {
        return sequential(
                follow(follower, startA_shootA()),

                // shooting actions here - these are written as Commands instead of our normal JAVA
                instant(() -> {
                    turret.target(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING, 0);
                    feeder.setPower(1);
                    }),
                waitMs(preloadShootTime),  // waits for all balls to exit
                instant(() -> turret.stopFlywheel()),  // shuts down flywheel to save battery power

                follow(follower, shootA_alignFlowerA()),
                // lower flower picker
                instant(() -> {
                    flowerPickerA.setPosition(down);
                    flowerPickerB.setPosition(1-down);
                    }),
                waitMs(flowerPickTime),
                instant(() -> {
                    intake.setPower(1);
                    feeder.setPower(1);
                    }),

                follow(follower, alignFlowerA_pickFlowerA()),
                // engage flower picker for collection
                waitMs(moveFlowerPicker),
                instant(() -> {
                    intake.setPower(0);
                    feeder.setPower(0);
                    }),

                follow(follower, pickFlowerA_alignFlowerA()),
                instant(() -> {
                    flowerPickerA.setPosition(up);
                    flowerPickerB.setPosition(1-up);}),
                //
                waitMs(moveFlowerPicker),


                follow(follower, alignFlowerA_shootSecondTime()),
                // shooting actions again
                instant(() -> {
                    turret.target(SHOOTA_X, SHOOTA_Y, SHOOTA_HEADING, 0);
                    feeder.setPower(1);
                }),
                waitMs(preloadShootTime),               // waits for all balls to exit
                instant(() -> {
                    turret.stopFlywheel();   // shuts down flywheel to save battery power
                    feeder.setPower(0);
                })    // shuts down feeder to end routine

                // end of autonomous actions
        );
    }

    /**
     * Sample helper Command
     *  - the code within the autoRoutine would simply be "lowerFlowerPicker(),"
     */
    public Command lowerFlowerPicker(){
        return parallel(
        instant(()->{flowerPickerA.setPosition(down);}),
        instant(()->{flowerPickerB.setPosition(1-down);})
        );
    }


    @Override
    public void init() {
        Scheduler.reset();
        // duplicate telemetry to the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // create Pedro follower for our Drivetrain and initialize to the starting pose
        follower = Constants.create(hardwareMap);
        follower.setPose(startA);
        follower.update();
        // Create the turret object for alignment, hood, and flywheel and set to starting position
        turret = new Turret(hardwareMap, telemetry);
        turret.initialize();
        // Map all hardware that isn't created within the turret class
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");
        // Set initial mechanisms to starting positions
        intake.setPower(0);
        feeder.setPower(0);
        flowerPickerA.setPosition(up);
        flowerPickerB.setPosition(up);
    }

    public void start() {

        schedule(autoRoutine());
    }

    public void loop() {
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


    public Path startA_shootA() {   // MOVE and score
        return Paths.line(startA, shootA).linear(startA, shootA);
    }

    public Path shootA_alignFlowerA() {   // align to flower before lowering flowerpicker
        return Paths.curve(shootA, point2Control1, point2Control2, alignFlowerA).tangent();
    }

    public Path alignFlowerA_pickFlowerA() {   // move to pick flower
        return Paths.line(alignFlowerA, pickFlowerA).constant(pickFlowerA);
    }

    public Path pickFlowerA_alignFlowerA() {   // move away from flower before
        return Paths.line(pickFlowerA, alignFlowerA).linear(pickFlowerA, alignFlowerA);
    }

    public Path alignFlowerA_shootSecondTime() {   // return to score in hiveA
        return Paths.line(alignFlowerA, shootSecondTime).linear(alignFlowerA, shootSecondTime);
    }
}

