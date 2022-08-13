package org.firstinspires.ftc.teamcode.Pineapplez;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Pinapple", group = "TeleOp")
public class ChassisOne extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;
    protected DcMotor clawArm = null;
    protected DcMotor ducky = null;

    protected Servo claw = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Encoder Values
    // Neverest 40 motor spec: quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120; // Neverest 40 motor encoder All drive motor gearings
    private static final double DRIVE_GEAR_REDUCTION1 = .5; // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    //Variables
    private double max = 1.0;
    double maxPower;
    double duckyPower = .57;
    double powerLim = 1;
    double moveDir = 1;
    private double clawMin = 0.722;
    private double clawMax = 1.16;
    private double clawPos = 0.0;

    /*This function determines the number of ticks a motor
    would need to move in order to achieve a certain degree*/
    private int getCountsPerDegree(double degrees, int motorNumber) {
        int ans = 0;
        if (motorNumber == 1) {
            ans = (int) (degrees * COUNTS_PER_DEGREE1);
        } else {
            return 1;
        }
        return ans;
    }

    @Override
    public void runOpMode() {
        //Prepares all the hardware
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        clawArm = hardwareMap.get(DcMotor.class, "clawArm");
        ducky = hardwareMap.get(DcMotor.class, "ducky");

        claw = hardwareMap.get(Servo.class, "claw");
        motorFwdLeft.setDirection(DcMotor.Direction.REVERSE);
        clawArm.setDirection(DcMotor.Direction.REVERSE);
        ducky.setDirection(DcMotor.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //variables
        boolean changed3 = false;
        boolean changed4 = false;
        boolean changed5 = false;
        boolean changed6 = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //movement controls
            //collects input from the joysticks
            double fwdBackPower = this.gamepad1.left_stick_y * moveDir;
            double strafePower = this.gamepad1.left_stick_x * moveDir;
            double turnPower = this.gamepad1.right_stick_x;

            //does math to figure the power that should be applied to every motor
            double leftFrontPower = (fwdBackPower - turnPower - strafePower) * powerLim;
            double rightFrontPower = (fwdBackPower + turnPower - strafePower) * powerLim;
            double leftBackPower = (fwdBackPower - turnPower + strafePower) * powerLim;
            double rightBackPower = (fwdBackPower + turnPower + strafePower) * powerLim;


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

            //collects input for shooter, and intake motors
            double intake = this.gamepad1.right_trigger - this.gamepad1.left_trigger;
            double shoot = this.gamepad2.right_trigger - this.gamepad2.left_trigger;

            //toggle for speed and direction of the bot for easier control
            if (gamepad1.b && !changed3) {//speed limiter toggle
                if (powerLim == .5) {
                    powerLim = 1;
                } else {
                    powerLim = .5;
                }
                changed3 = true;
            } else if (!gamepad1.b) {
                changed3 = false;
            }

            if (gamepad1.a && !changed4) {//direction change toggle
                if (moveDir == 1) {
                    moveDir = -1;
                } else {
                    moveDir = 1;
                }
                changed4 = true;
            } else if (!gamepad1.a) {
                changed4 = false;
            }

            if (gamepad2.left_bumper) {
                clawArm.setPower(0.75);
            } else if (!gamepad2.left_bumper) {
                clawArm.setPower(0);
            }


            if (gamepad2.right_bumper) {
                clawArm.setPower(-0.75);
            } else if (!gamepad2.right_bumper) {
                clawArm.setPower(0);
            }


            if (gamepad2.a && !ducky.isBusy()) {
                long millis = 1100;
                runtime.reset();
                while (runtime.milliseconds() <= millis) {
                    ducky.setPower(duckyPower);
                    duckyPower = duckyPower * 1.5;
                }

                /*millis = 600;
                runtime.reset();
                while(runtime.milliseconds() <= millis){
                    ducky.setPower(duckyPower+1.5);
                }*/
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
            }

            // TOGGLE FOR CLASS %%%%%%%%%%%%%%%%% \\
            if(gamepad2.b && clawPos < clawMax){
                while(clawPos < clawMax) {
                    clawPos += 0.3;
                }
                claw.setPosition(clawPos);
            }
            else if(gamepad2.y && clawPos > clawMin){
                while(clawPos > clawMin) {
                    clawPos -= .3;
                }
                claw.setPosition(clawPos);
            }


            telemetry.addData("Wheel Position", motorFwdLeft.getCurrentPosition()); //to be used when the encoders are ready
            telemetry.addData("Max Speed", powerLim);
            telemetry.addData("Direction", moveDir);
            telemetry.addData("servo", clawPos);
            telemetry.update();
            }
        }
}