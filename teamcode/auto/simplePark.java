package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
@Disabled
public class simplePark extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: neverest Motor Encoder lol
    static final double     WHEEL_DIAMETER_INCHES   = 3.85827 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV ) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;
    //declare motors
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    public Servo intake;
    public Servo intakeSetup;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

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
        intake = hardwareMap.get(Servo.class, "intake");
        //intakeSetup = hardwareMap.get(Servo.class, "intakeSetup");

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            double open = 0.6;
            double close = 0.98;

            intake.setPosition(close); //close
            //intakeSetup.setPosition(up); //up
            sleep(2000);
            // move forward 4 inches
            encoderDrive(DRIVE_SPEED,  -4,  -4, -4, -4, 8);
            // strafe left
            encoderDrive(DRIVE_SPEED,  30,  -30, -30, 30, 8);
            // move forward
            encoderDrive(DRIVE_SPEED, -30, -30, -30, -30, 6);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory
            waitForStart();
            double open = 0.6;
            double close = 0.98;
            intake.setPosition(close); //close intake (grips cone)
            //intakeSetup.setPosition(up); //up - (sets cone in upright position
            sleep(2000);
            // move forward 40 inches
            encoderDrive(DRIVE_SPEED,  -40,  -40, -40, -40, 8);  // S1: Forward 47 Inches with 5 Sec timeout

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }else{
            waitForStart();
            double open = 0.6;
            double close = 0.98;
            intake.setPosition(close); //close intake (grips cone)
            //intakeSetup.setPosition(up); //up - (sets cone in upright position
            sleep(2000);
            // move forward 4 inches
            encoderDrive(DRIVE_SPEED,  -4,  -4, -4, -4, 8);
            // strafe right
            encoderDrive(DRIVE_SPEED,  -25,  25, 25, -25, 8);
            // move forward
            encoderDrive(DRIVE_SPEED, -36, -36, -36, -36, 6);
//
            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);

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