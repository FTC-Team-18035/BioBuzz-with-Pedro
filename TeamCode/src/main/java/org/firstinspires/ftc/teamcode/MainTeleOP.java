import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import Trajectory;
//import Turret;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class MainTeleOP extends LinearOpMode {

    private Follower follower;

    // Define motors and servos
    private DcMotorEx intake;
    private DcMotorEx feeder;

    private Servo flowerPickerA;
    private Servo flowerPickerB;

    @Override
    public void runOpMode() {
        // Create the Pedro follower
        follower = Constants.create(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        feeder = hardwareMap.get(DcMotorEx.class, "feeder");

        flowerPickerA = hardwareMap.get(Servo.class, "flowerPickerA");
        flowerPickerB = hardwareMap.get(Servo.class, "flowerPickerB");

        // TODO: Add Limelight, will be using placeholder for now

    waitForStart();

    while (opModeIsActive()) {
        // Limelight.getTX and all that good stuff
        double distance = 0.0; // Placeholder for Limelight TX value

        LaunchVelocity=Trajectory.CalculateVelocity(distance);
        LaunchAngle=Trajectory.CalculateAngle(distance);

        // Turret.target(Limelight.getTX(), follower.pose().velocity().x, LaunchVelocity, LaunchAngle);

        // Calculate drive powers for field centric control based on joystick input and robot heading, using Pedro's field centric driving
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);
        follower.update();
    }
}
}