package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class ReferenceAuto extends LinearOpMode{

    @Override
    public void runOpMode() {

        // Setup a motor
        // Change "motorName" to match the name of the motor (FL, FR, BL, or BR)
        DcMotor motorName;
        motorName = hardwareMap.get(DcMotor.class, "motorName");
        motorName.setDirection(DcMotorSimple.Direction.FORWARD); //

        waitForStart();

        while(opModeIsActive()){



          
        }

    
    }
}
