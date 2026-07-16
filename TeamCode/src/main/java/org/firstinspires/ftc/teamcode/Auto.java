package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", group = "Autos")
public class Auto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontRight.setDirection(DcMotor.Direction.REVERSE);;
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        
        // Write the movements under this comment
        forward();
        
    }



    private void forward(){
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);
        sleep(1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void backward(){
        frontLeft.setPower(-0.6);
        frontRight.setPower(-0.6);
        backLeft.setPower(-0.6);
        backRight.setPower(-0.6);

        sleep(1000);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void left(){
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        sleep(1300);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void right(){
        frontLeft.setPower(0.6);
        frontRight.setPower(0.6);
        backLeft.setPower(0.6);
        backRight.setPower(0.6);

        sleep(1300);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
