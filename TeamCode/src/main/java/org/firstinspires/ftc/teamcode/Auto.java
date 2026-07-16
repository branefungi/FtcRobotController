package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "Autos")
public class SummerCampAuto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    IMU imu;
    double yaw;
    ElapsedTime timer;

    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);;
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // Write the movements under this comment
        forward(3);

    }



    private void forward(double distance){
        timer.reset();

        while(timer.seconds() < 1 * distance) {
            yaw = Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw());

            frontLeft.setPower(0.6 + yaw/180);
            frontRight.setPower(0.6 - yaw/180);
            backLeft.setPower(0.6 + yaw/180);
            backRight.setPower(0.6 - yaw/180);
        }


        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void backward(double distance){
        frontLeft.setPower(-0.6);
        frontRight.setPower(-0.6);
        backLeft.setPower(-0.6);
        backRight.setPower(-0.6);

        sleep(Math.round(1000 * distance));

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void left(double distance){
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        sleep(Math.round(1300 * distance));

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void right(double distance){
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        sleep(Math.round(1300 * distance));

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
