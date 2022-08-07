package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Shooter", group="Linear Opmode")
public class Shooter extends LinearOpMode {

    protected DcMotor leftShooter   = null;
    protected DcMotor rightShooter  = null;


    public void runOpMode() {
        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {
            leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
            rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
            leftShooter.setDirection(DcMotor.Direction.FORWARD);
            rightShooter.setDirection(DcMotor.Direction.REVERSE);
            if (gamepad1.a) {
                leftShooter.setPower(1.5);
                rightShooter.setPower(1.5);
            }
            if (gamepad1.x) {
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }
        }
    }

}

