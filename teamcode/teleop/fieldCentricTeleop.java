package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class fieldCentricTeleop extends LinearOpMode {
    double DRIVE_MULTIPLIER = 1;
    double LIFT_MULTIPLIER = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeft = hardwareMap.dcMotor.get("FL");
        DcMotor frontRight = hardwareMap.dcMotor.get("FR");
        DcMotor backLeft = hardwareMap.dcMotor.get("BL");
        DcMotor backRight = hardwareMap.dcMotor.get("BR");
        DcMotor liftN = hardwareMap.get(DcMotor.class, "liftN");
        DcMotor liftR = hardwareMap.get(DcMotor.class, "liftR");
        Servo intake = hardwareMap.get(Servo.class, "intake");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            liftN.setDirection(DcMotorSimple.Direction.FORWARD);
            liftR.setDirection(DcMotorSimple.Direction.REVERSE);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if(gamepad2.b){
                liftN.setPower(-1*LIFT_MULTIPLIER);
                liftR.setPower(-1*LIFT_MULTIPLIER);
            } else{

                liftN.setPower(0);
                liftR.setPower(0);
            }

            if(gamepad2.a){
                liftN.setPower(1*LIFT_MULTIPLIER);
                liftR.setPower(1*LIFT_MULTIPLIER);
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
                intake.setPosition(0.98);
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