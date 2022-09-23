package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MainRobotController", group = "TeleOp")
public class MainRobotController extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;

    private final ElapsedTime runtime = new ElapsedTime();

    // Encoder Values \\
    // Neverest 40 motor spec: quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120; // Neverest 40 motor encoder All drive motor gearings
    private static final double DRIVE_GEAR_REDUCTION1 = .5; // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    // Movement Variables \\
    double max = 1.0;
    double maxPower;
    double powerLim = 1;
    double moveDir = -1;

    // Movement motor powers \\
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Previous frames motor powers \\
    double prevLeftFrontPower;
    double prevRightFrontPower;
    double prevLeftBackPower;
    double prevRightBackPower;

    double[] prevMotorsPowers = {prevLeftFrontPower, prevRightFrontPower, prevLeftBackPower, prevRightBackPower};

    /*This function determines the number of ticks a motor
    would need to move in order to achieve a certain degree*/
    private int getCountsPerDegree(double degrees, int motorNumber) {
        int ans;
        if (motorNumber == 1) { ans = (int) (degrees * COUNTS_PER_DEGREE1); }
        else { return 1; }
        return ans;
    }

    private void lerpVal(double val, double prevVal, double motorPow, int minDiff, int divider) {
        if (Math.abs(val - prevVal) >= minDiff) {
            motorPow = (val + prevVal) / divider;
            prevVal = motorPow;
        } else {
            motorPow = val;
            prevVal = val;
        }
    }

    @Override
    public void runOpMode() {
        // Preparing all hardware \\
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");

        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Toggle variables
        boolean changed3 = false;
        boolean changed4 = false;

        // Wait for driver to press PLAY
        waitForStart();
        // Run until driver presses STOP
        while (opModeIsActive()) {
            // Input from the joysticks \\
            double fwdPower = this.gamepad1.left_stick_y * moveDir; // up and down
            double strafePower = this.gamepad1.left_stick_x * moveDir; // left and right
            double turnPower = this.gamepad1.right_stick_x; // turn of robot

            // Math to find the power for every motor \\
            leftFrontPower = (fwdPower - turnPower - strafePower) * powerLim;
            rightFrontPower = (fwdPower + turnPower - strafePower) * powerLim;
            leftBackPower = (fwdPower - turnPower + strafePower) * powerLim;
            rightBackPower = (fwdPower + turnPower + strafePower) * powerLim;

            maxPower = Math.abs(leftFrontPower);
            if (Math.abs(rightFrontPower) > maxPower) {
                maxPower = Math.abs(rightFrontPower);
            }
            if (Math.abs(leftBackPower) > maxPower) {
                maxPower = Math.abs(leftBackPower);
            }
            if (Math.abs(rightBackPower) > maxPower) {
                maxPower = Math.abs(rightBackPower);
            }
            if (maxPower > 1) {
                leftFrontPower = leftFrontPower / maxPower;
                rightFrontPower = rightFrontPower / maxPower;
                leftBackPower = leftBackPower / maxPower;
                rightBackPower = rightBackPower / maxPower;

            }

            //sets the power of the motors
            motorFwdLeft.setPower(leftFrontPower * max);
            motorFwdRight.setPower(rightFrontPower * max);
            motorBackLeft.setPower(leftBackPower * max);
            motorBackRight.setPower(rightBackPower * max);

            prevLeftFrontPower = leftFrontPower;
            prevRightFrontPower = rightFrontPower;
            prevLeftBackPower = leftBackPower;
            prevRightBackPower = rightBackPower;

            // SPEED CHANGE TOGGLE \\
            // (multiplied by speed, so halves speed or leaves it alone) \\
            if (gamepad1.b && !changed3) {
                if (powerLim == .5) { powerLim = 1; }
                else { powerLim = .5; }
                changed3 = true;
            } else if (!gamepad1.b) {
                changed3 = false;
            }

            // DIRECTION CHANGE TOGGLE \\
            if (gamepad1.a && !changed4) {
                moveDir *= -1; // 'moveDir' is 1 or -1
                changed4 = true;
            } else if (!gamepad1.a) {
                changed4 = false;
            }

            /*if (gamepad2.a && !ducky.isBusy()) {
                long millis = 1100;
                runtime.reset();
                while (runtime.milliseconds() <= millis) {
                    ducky.setPower(duckyPower);
                    duckyPower = duckyPower * 1.5;
                }
                ducky.setPower(0);
            } else if (gamepad2.x && !ducky.isBusy()) {
                long millis = 1100;
                runtime.reset();
                while (runtime.milliseconds() <= millis) {
                    ducky.setPower(-duckyPower);
                }
                millis = 600;
                runtime.reset();
                while (runtime.milliseconds() <= millis) {
                    ducky.setPower(-duckyPower - 1.5);
                }
                ducky.setPower(0);
            } */

            telemetry.addData("Wheel Position", motorFwdLeft.getCurrentPosition()); //to be used when the encoders are ready
            telemetry.addData("Max Speed", powerLim);
            telemetry.addData("Direction", moveDir);
            telemetry.update();
        }
    }
}
