package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
//@Disabled
public class robotCentricTeleop extends LinearOpMode {
    double DRIVE_MULTIPLIER = 1;
    double LIFT_MULTIPLIER = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        DcMotor frontRight  = hardwareMap.get(DcMotor.class, "FR");
        DcMotor backLeft  = hardwareMap.get(DcMotor.class, "BL");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "BR");
        DcMotor liftN = hardwareMap.get(DcMotor.class, "liftN");
        DcMotor liftR = hardwareMap.get(DcMotor.class, "liftR");
        Servo intake = hardwareMap.get(Servo.class, "intake");
        //Servo intakeSetup = hardwareMap.get(Servo.class, "intakeSetup");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y*DRIVE_MULTIPLIER; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1*DRIVE_MULTIPLIER; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x* DRIVE_MULTIPLIER;

            liftN.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setDirection(DcMotorSimple.Direction.REVERSE);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if(gamepad2.b){
                liftN.setPower(1*LIFT_MULTIPLIER);
                liftR.setPower(1*LIFT_MULTIPLIER);
            } else{

                liftN.setPower(0);
                liftR.setPower(0);
            }

            if(gamepad2.a){
                liftN.setPower(-.5*LIFT_MULTIPLIER);
                liftR.setPower(-.5*LIFT_MULTIPLIER);
            }else{

                liftN.setPower(0);
                liftR.setPower(0);
            }

            //open
            if(gamepad2.right_bumper){
                intake.setPosition(0.6);
            }
            //close
            if(gamepad2.left_bumper){
                intake.setPosition(0.85); //(0.98);
            }


            //slow mode lift
            if (gamepad2.dpad_right){
                LIFT_MULTIPLIER = 0.5;
            }else{
                LIFT_MULTIPLIER = 1;
            }
            // slow mode drive
            if (gamepad1.right_bumper){
                DRIVE_MULTIPLIER = 0.5;
            }else{
                DRIVE_MULTIPLIER = 1;
            }


        }
    }
}

