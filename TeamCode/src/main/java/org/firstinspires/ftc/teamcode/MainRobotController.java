package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    double maxPower;
    double maxSpeed = 0.7;
    double powerLim = 1;
    double moveDir = -1;
    double minDiff = 0.1;
    double goalTime = 10.0;


    // Movement motor powers \\
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // left front, right front, left back, right back
    double[] prevPowers = { 0.0, 0.0, 0.0, 0.0 };

    /*This function determines the number of ticks a motor
    would need to move in order to achieve a certain degree*/
    private int getCountsPerDegree(double degrees, int motorNumber) {
        int ans;
        if (motorNumber == 1) { ans = (int) (degrees * COUNTS_PER_DEGREE1); }
        else { return 1; }
        return ans;
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
        boolean changed5 = false;
        boolean changed6 = false;

        // Wait for driver to press PLAY
        waitForStart();
        // Run until driver presses STOP
        while (opModeIsActive()) {
            // Input from the joysticks \\
            double fwdPower = this.gamepad1.left_stick_y * moveDir; // up and down
            double strafePower = this.gamepad1.left_stick_x * moveDir; // left and right
            double turnPower = this.gamepad1.right_stick_x; // turn of robot

            double[] powers = { leftFrontPower, rightFrontPower, leftBackPower, rightBackPower };

            boolean turning = Math.abs(turnPower) > minDiff; // True if turning
            boolean strafing = Math.abs(strafePower) > minDiff; // True if strafing

            boolean halfSpeed = false;

            // Math to find the power for every motor \\
            leftFrontPower = (fwdPower + turnPower - strafePower) * powerLim;
            rightFrontPower = (fwdPower - turnPower + strafePower) * powerLim;
            leftBackPower = (fwdPower + turnPower + strafePower) * powerLim;
            rightBackPower = (fwdPower - turnPower - strafePower) * powerLim;

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

            // Smoothing of motor power values with previous values
            for (int i=0; i<prevPowers.length; i++) {
                double diff = powers[i] - prevPowers[i]; // Used for smoothing
                double change = Math.abs(powers[i]) - Math.abs(prevPowers[i]); // Used for check if going forward or backward

                // "If there's enough joystick change and the robot isn't turning nor strafing..."
                if (change >= minDiff && !strafing && !turning) {
                    prevPowers[i] += diff / goalTime;
                } else {
                    prevPowers[i] = powers[i];
                }

                // Maxing out the speed
                if (Math.abs(prevPowers[i]) > maxSpeed) {
                    if (prevPowers[i] < 0) {
                        prevPowers[i] = maxSpeed * -1;
                    } else {
                        prevPowers[i] = maxSpeed;
                    }
                }
                telemetry.addData("Motor power", prevPowers[i]);
            }

            // Sets the power of the motors
            motorFwdLeft.setPower(prevPowers[0]);
            motorFwdRight.setPower(prevPowers[1]);
            motorBackLeft.setPower(prevPowers[2]);
            motorBackRight.setPower(prevPowers[3]);

            // SPEED CHANGE TOGGLE \\
            // (multiplied by speed, so halves speed or leaves it alone) \\
            if (gamepad1.b && !changed3) {
                if (powerLim == .5) {
                    powerLim = 1;
                    halfSpeed = false;
                }
                else {
                    powerLim = .5;
                    halfSpeed = true;
                }
                changed3 = true;
            } else if (!gamepad1.b) {
                changed3 = false;
            }

            // When robot is turning, ignore halving of speed (feels really slow)
            if (halfSpeed && turning) { powerLim = 1; }
            else if (halfSpeed && powerLim == 1) { powerLim = .5; }

            // DIRECTION CHANGE TOGGLE \\
            if (gamepad1.a && !changed4) {
                moveDir *= -1; // 'moveDir' is 1 or -1
                changed4 = true;
            } else if (!gamepad1.a) {
                changed4 = false;
            }

            /*if (gamepad1.y && !changed5 && !gamepad1.x && maxSpeed <= 1) {
                maxSpeed += 0.1;
                //goalTime++;
                changed5 = true;
            } else if (!gamepad1.y) {
                changed5 = false;
            }

            if (gamepad1.x && !changed6 && !gamepad1.y && maxSpeed > 0) {
                maxSpeed -= 0.1;
                //goalTime--;
                changed6 = true;
            } else if (!gamepad1.x) {
                changed6 = false;
            } */

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

            telemetry.addData("Fwd Power", fwdPower);
            //telemetry.addData("Smoothed RB Power", prevPowers[0]);
            telemetry.addData("MaxSpeed", maxSpeed);
            telemetry.addData("Speed", powerLim);
            telemetry.addData("Direction", moveDir);
            telemetry.update();
        }
    }
}
