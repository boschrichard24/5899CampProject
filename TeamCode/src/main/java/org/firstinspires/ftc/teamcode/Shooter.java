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

    // MOTOR VALUES \\
    private boolean motorsOn = false; // Toggle bool for flywheel motors
    private boolean prevMotorsOnVal = false; // "Previous 'motorsOn' Value" dhdjdjjdjj
    final double motorSpeed = 1.5;

    public void runOpMode() {
        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeIsActive()) {
            leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
            rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
            leftShooter.setDirection(DcMotor.Direction.FORWARD);
            rightShooter.setDirection(DcMotor.Direction.REVERSE);

            // True if button is down (Bool value is returned from gamepad buttons/bumpers)
            boolean buttonDown = gamepad1.a;
            // Reset prevMOV so the block below can run if needed
            if (!buttonDown) { prevMotorsOnVal = !motorsOn; }

            // Toggle logic (This is called only once as the button is pressed)
            if (buttonDown && motorsOn != prevMotorsOnVal) {
                motorsOn = !motorsOn;
                prevMotorsOnVal = motorsOn; // Reset prevMOV
            }

            // Set motor powers
            if (motorsOn) {
                leftShooter.setPower(motorSpeed);
                rightShooter.setPower(motorSpeed);
            } else {
                leftShooter.setPower(0.0);
                rightShooter.setPower(0.0);
            }
        }
    }
}

