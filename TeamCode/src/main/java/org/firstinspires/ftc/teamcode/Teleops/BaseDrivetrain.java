package org.firstinspires.ftc.teamcode.Teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Drivetrain", group = "TeleOp") // SIDE RED/BLUE

public class BaseDrivetrain extends LinearOpMode { // SIDE
    private Drivetrain drivetrain;
    private Follower follower;

    @Override
    public void runOpMode() {
        // initializes movement motors
        drivetrain = new Drivetrain(hardwareMap, gamepad1, telemetry, true, false);
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
                follower.update();
            drivetrain.Teleop(0);

            telemetry.addData("X:", follower.pose().x());
            telemetry.addData("Y:", follower.pose().y());
            telemetry.addData("Heading:", follower.pose().heading());
            telemetry.update();
        }
    }
}