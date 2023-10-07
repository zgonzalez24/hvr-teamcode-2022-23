package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@Autonomous
@Disabled
public class leftScore2 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: neverest Motor Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 3.85827 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     LIFT_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.6;
    //declare motors
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor liftN;
    public DcMotor liftR;
    public Servo intake;
    //public Servo intakeSetup;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double open = 0.9;
    double close = 0.5;
    //double up = 0.1;
    //double down = 0.6;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backLeft  = hardwareMap.get(DcMotor.class, "BL");
        backRight  = hardwareMap.get(DcMotor.class, "BR");
        liftN = hardwareMap.get(DcMotor.class, "liftN");
        liftR = hardwareMap.get(DcMotor.class, "liftR");
        intake = hardwareMap.get(Servo.class, "intake");
        //intakeSetup = hardwareMap.get(Servo.class, "intakeSetup");

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        liftN.setPower(0);
        liftR.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftN.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftN.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftN.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftN.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT){
            //trajectory
            waitForStart();
            double open = 0.6;
            double close = 0.98;

            intake.setPosition(close); //close
            sleep(500);
            encoderLift(LIFT_SPEED,-5, -5, 4);
            //intakeSetup.setPosition(up); //up
            //move forward 4
            encoderDrive(DRIVE_SPEED,  -48,  -48, -48, -48, 8);
            //move signal out
            encoderDrive(DRIVE_SPEED,  -5,  -5, -5, -5, 8);
            //move back into place
            encoderDrive(DRIVE_SPEED,  3,  3, 3, 3, 8);
            //strafe right
            //encoderDrive(TURN_SPEED,  -14.5,  14.5, 14.5, -14.5, 8);
            encoderDrive(TURN_SPEED,  -15,  15, 15, -15, 8);

            sleep(500);
//            //turn right
//            encoderDrive(TURN_SPEED,  -19.5,  19.5, -19.5, 19.5, 8);
//            sleep(500);
//            //strafe left
//            encoderDrive(TURN_SPEED,  41,  -41, -41, 41, 8);//up
            encoderLift(LIFT_SPEED,-66, -66, 4);
            //forward 2 in
            encoderDrive(TURN_SPEED,  -6,  -6, -6, -6, 8);
            //open
            intake.setPosition(open);
            sleep(250);
            //back 2 in
            encoderDrive(TURN_SPEED,  6,  6, 6, 6, 8);
            //lift down
            encoderLift(LIFT_SPEED,58, 58, 4);
            //strafe right
            encoderDrive(DRIVE_SPEED,  16,  -16, -16, 16, 8);
            encoderDrive(DRIVE_SPEED,  -4,  4, 4, -4, 8);

            //turn left for stack
            encoderDrive(TURN_SPEED,  18.5,  -18.5, 18.5, -18.5, 8);

            //stack ACTION
            //forward
            encoderDrive(TURN_SPEED,  -25,  -25, -25, -25, 8);

            sleep(250);
            intake.setPosition(close);
            sleep(250);
            encoderLift(LIFT_SPEED,-13, -13, 4);

            encoderDrive(TURN_SPEED,  22,  22, 22, 22, 8);
            //turn right
            encoderDrive(TURN_SPEED,  -18.5,  18.5, -18.5, 18.5, 8);
            //strafe into pole
            //different
            encoderDrive(TURN_SPEED,  -16,  16, 16, -16, 8);
            encoderLift(LIFT_SPEED,-47, -47, 4);
            //forward 2 in
            encoderDrive(TURN_SPEED,  -5,  -5, -5, -5, 8);
            //open
            intake.setPosition(open);
            sleep(250);
            //back 2 in
            encoderDrive(TURN_SPEED,  4,  4, 4, 4, 8);
            //lift down
            encoderLift(LIFT_SPEED,55, 55, 4);
            //strafe left
            encoderDrive(DRIVE_SPEED,  38,  -38, -38, 38, 8);
            sleep(250);

//            //drive backward
//            encoderDrive(TURN_SPEED, 47, 47, 47, 47, 5);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory
            waitForStart();
            double open = 0.6;
            double close = 0.98;

            intake.setPosition(close); //close
            sleep(500);
            encoderLift(LIFT_SPEED,-5, -5, 4);
            //intakeSetup.setPosition(up); //up
            //move forward 4
            encoderDrive(DRIVE_SPEED,  -48,  -48, -48, -48, 8);
            //strafe right
            encoderDrive(DRIVE_SPEED,  -5,  -5, -5, -5, 8);
            //move back into place
            encoderDrive(DRIVE_SPEED,  3,  3, 3, 3, 8);

            //encoderDrive(TURN_SPEED,  -14.5,  14.5, 14.5, -14.5, 8);
            encoderDrive(TURN_SPEED,  -15,  15, 15, -15, 8);

            sleep(500);
//            //turn right
//            encoderDrive(TURN_SPEED,  -19.5,  19.5, -19.5, 19.5, 8);
//            sleep(500);
//            //strafe left
//            encoderDrive(TURN_SPEED,  41,  -41, -41, 41, 8);//up
            encoderLift(LIFT_SPEED,-66, -66, 4);
            //forward 2 in
            encoderDrive(TURN_SPEED,  -6,  -6, -6, -6, 8);
            //open
            intake.setPosition(open);
            sleep(250);
            //back 2 in
            encoderDrive(TURN_SPEED,  6,  6, 6, 6, 8);
            //lift down
            encoderLift(LIFT_SPEED,58, 58, 4);
            //strafe right
            encoderDrive(DRIVE_SPEED,  16,  -16, -16, 16, 8);
            encoderDrive(DRIVE_SPEED,  -4,  4, 4, -4, 8);

            //turn left for stack
            encoderDrive(TURN_SPEED,  18.5,  -18.5, 18.5, -18.5, 8);

            //stack ACTION
            //forward
            encoderDrive(TURN_SPEED,  -25,  -25, -25, -25, 8);

            sleep(250);
            intake.setPosition(close);
            sleep(250);
            encoderLift(LIFT_SPEED,-13, -13, 4);

            encoderDrive(TURN_SPEED,  22,  22, 22, 22, 8);
            //turn right
            encoderDrive(TURN_SPEED,  -18.5,  18.5, -18.5, 18.5, 8);
            //strafe into pole
            //change form 18.5 to 16

            encoderDrive(TURN_SPEED,  -16,  16, 16, -16, 8);
            encoderLift(LIFT_SPEED,-47, -47, 4);
            //forward 2 in
            encoderDrive(TURN_SPEED,  -5,  -5, -5, -5, 8);
            //open
            intake.setPosition(open);
            sleep(250);
            //back 2 in
            encoderDrive(TURN_SPEED,  6,  6, 6, 6, 8);
            //lift down
            encoderLift(LIFT_SPEED,55, 55, 4);
            //strafe right
            encoderDrive(DRIVE_SPEED,  16,  -16, -16, 16, 8);
            sleep(250);


            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }else{
            waitForStart();
            double open = 0.6;
            double close = 0.98;

            intake.setPosition(close); //close
            sleep(500);
            encoderLift(LIFT_SPEED,-5, -5, 4);
            //intakeSetup.setPosition(up); //up
            //move forward 4
            encoderDrive(DRIVE_SPEED,  -48,  -48, -48, -48, 8);
            //strafe right
            encoderDrive(DRIVE_SPEED,  -5,  -5, -5, -5, 8);
            //move back into place
            encoderDrive(DRIVE_SPEED,  3,  3, 3, 3, 8);
            encoderDrive(TURN_SPEED,  -15,  15, 15, -15, 8);

            //encoderDrive(TURN_SPEED,  -14.5,  14.5, 14.5, -14.5, 8);
            sleep(500);
//            //turn right
//            encoderDrive(TURN_SPEED,  -19.5,  19.5, -19.5, 19.5, 8);
//            sleep(500);
//            //strafe left
//            encoderDrive(TURN_SPEED,  41,  -41, -41, 41, 8);//up
            encoderLift(LIFT_SPEED,-66, -66, 4);
            //forward 2 in
            encoderDrive(TURN_SPEED,  -6,  -6, -6, -6, 8);
            //open
            intake.setPosition(open);
            sleep(250);
            //back 2 in
            encoderDrive(TURN_SPEED,  6,  6, 6, 6, 8);
            //lift down
            encoderLift(LIFT_SPEED,58, 58, 4);
            //strafe right
            encoderDrive(DRIVE_SPEED,  16,  -16, -16, 16, 8);
            encoderDrive(DRIVE_SPEED,  -4,  4, 4, -4, 8);

            //turn left for stack
            encoderDrive(TURN_SPEED,  18.5,  -18.5, 18.5, -18.5, 8);

            //stack ACTION
            //forward
            encoderDrive(TURN_SPEED,  -25,  -25, -25, -25, 8);

            sleep(250);
            intake.setPosition(close);
            sleep(250);
            encoderLift(LIFT_SPEED,-13, -13, 4);

            encoderDrive(TURN_SPEED,  22,  22, 22, 22, 8);
            //turn right
            encoderDrive(TURN_SPEED,  -18.5,  18.5, -18.5, 18.5, 8);
            //strafe into pole
            encoderDrive(TURN_SPEED,  -16,  16, 16, -16, 8);
            encoderLift(LIFT_SPEED,-47, -47, 4);
            //forward 2 in
            encoderDrive(TURN_SPEED,  -5,  -5, -5, -5, 8);
            //open
            intake.setPosition(open);
            sleep(250);
            //back 2 in
            encoderDrive(TURN_SPEED,  6,  6, 6, 6, 8);
            //lift down
            encoderLift(LIFT_SPEED,55, 55, 4);
            //strafe right
            encoderDrive(DRIVE_SPEED,  -16,  16, 16, -16, 8);
            sleep(250);


        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }


    // encoder drive function, to be used in main body to set speeds and distances
    public void encoderDrive(double speed, double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches,
                             double timeoutS){

        int frontLeftTarget;
        int frontRightTarget;
        int backRightTarget;
        int backLeftTarget;

        if (opModeIsActive()) {
            //define targets to match motor and position
            frontLeftTarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            frontRightTarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            backLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            backRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);

            //motor to target
            frontLeft.setTargetPosition(frontLeftTarget);
            frontRight.setTargetPosition(frontRightTarget);
            backLeft.setTargetPosition(backLeftTarget);
            backRight.setTargetPosition(backRightTarget);

            //motor to position
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            //gives motor mecanum configuration
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // loop that will constantly take the robots current position, update and use it for further movements
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();
            }
            //when path is done with movement all motors power set to 0
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            //run all actions through encoders
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move
        }
    }
    public void encoderLift(double speedLift, double liftNinches, double liftRinches, double timeoutS){

        int liftNTarget;
        int liftRTarget;

        if (opModeIsActive()) {
            //define targets to match motor and position
            liftNTarget = liftN.getCurrentPosition() + (int) (liftNinches * COUNTS_PER_INCH);
            liftRTarget = liftR.getCurrentPosition() + (int) (liftRinches * COUNTS_PER_INCH);

            //motor to target
            liftN.setTargetPosition(liftNTarget);
            liftR.setTargetPosition(liftRTarget);

            //motor to position
            liftN.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            //gives motor speed configuration
            liftN.setPower(speedLift);
            liftR.setPower(speedLift);

            // loop that will constantly take the robots current position, update and use it for further movements
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (liftN.isBusy() && liftR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", liftNTarget, liftRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        liftN.getCurrentPosition(),
                        liftR.getCurrentPosition());
                telemetry.update();
            }
            //when path is done with movement all motors power set to 0
            liftN.setPower(0);
            liftR.setPower(0);

            //run all actions through encoders
            liftN.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }


    //telemetry for camera detection
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}


