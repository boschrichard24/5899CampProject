package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class HardwareInit extends LinearOpMode {

    public void initAll(DcMotor oneM, DcMotor twoM, DcMotor threeM, DcMotor fourM){
        oneM = hardwareMap.get(DcMotor.class, "leftShooter");
        twoM = hardwareMap.get(DcMotor.class, "rightShooter");
        threeM = hardwareMap.get(DcMotor.class, "leftShooter");
        fourM = hardwareMap.get(DcMotor.class, "rightShooter");

        oneM.setDirection(DcMotor.Direction.FORWARD);
        twoM.setDirection(DcMotor.Direction.REVERSE);
        threeM.setDirection(DcMotor.Direction.FORWARD);
        fourM.setDirection(DcMotor.Direction.REVERSE);
    }

    public void runOpMode() {
        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {






        }
    }

}

