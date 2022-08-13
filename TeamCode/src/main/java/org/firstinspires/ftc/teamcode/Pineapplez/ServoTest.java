package org.firstinspires.ftc.teamcode.Pineapplez;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ServoTest", group = "TeleOp")

public class ServoTest extends LinearOpMode {
    //All hardware
    protected DcMotor motorFwdLeft = null;
    protected DcMotor motorFwdRight = null;
    protected DcMotor motorBackLeft = null;
    protected DcMotor motorBackRight = null;
    protected DcMotor clawArm = null;
    protected DcMotor ducky = null;

    protected Servo claw        = null;  // This is the open and close servo of the claw \\

    double servo = 0.5;

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
        claw.setDirection(Servo.Direction.FORWARD);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ducky.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                servo += 0.001;
            }
            else if(gamepad1.dpad_down){
                servo -= 0.001;
            }
            /*if(gamepad1.dpad_left){
                servo2 += 0.001;
            }
            else if(gamepad1.dpad_right){
                servo2 -= 0.001;
            }
             */
            claw.setPosition(servo);
            //ringPullPivotServo.setPosition(servo2);

            //telemetry.addData("unloadServo",wobbleArmServo.getPosition());
            telemetry.addData("servo", servo);
            telemetry.addData("claw", claw);
            /*
            telemetry.addData("Distance Left Top", distanceLeftTop.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Left Bottom", distanceLeftBottom.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Right", distanceFwdRight.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance Forward Left", distanceFwdLeft.getDistance(DistanceUnit.MM));
*/
            telemetry.update();
        }
    }
}
