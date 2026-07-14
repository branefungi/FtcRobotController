package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
public class ReferenceAuto extends LinearOpMode{

    @Override
    public void runOpMode() {

        // create a timer
        ElapsedTime myTimer = new ElapsedTime();

        // make the timer start counting up from 0
        myTimer.reset();

        // a double containing how much time the timer has measured
        myTimer.seconds();

        
        
        // pause the program, every 1000 is a 1 second pause
        sleep(1000);


        // set up a sensor the track the robot's angle
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        // the next two lines should match the control hub's orientation
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // reset the measured angle to 0 degrees
        imu.resetYaw();

        // a double containing the angle the robot is facing 
        Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw());




    }
}
