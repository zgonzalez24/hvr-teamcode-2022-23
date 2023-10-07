package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
//@Disabled
public class leftHighScore extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private final ElapsedTime runtime = new ElapsedTime();
    //declare motors

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor liftN = hardwareMap.get(DcMotor.class, "liftN");
        DcMotor liftR = hardwareMap.get(DcMotor.class, "liftR");
        Servo intake = hardwareMap.get(Servo.class, "intake");
        liftN.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        double open = 0.6;
        double close = 0.98;


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
            //ADD CODE HERE
            Pose2d startPose = new Pose2d(26, 61, Math.toRadians(272));
            drive.setPoseEstimate(startPose);

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(26, 61,Math.toRadians(272)))
                    .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                        intake.setPosition(close);
                    })
                    .splineTo(new Vector2d(34, 32), Math.toRadians(270)) //spline to straight position
                    .splineTo(new Vector2d(34, 22), Math.toRadians(270)) //spline to straight position
                    .addTemporalMarker(1.1, () ->{ //lift goes up 1.1 seconds into program, during spline
                        liftN.setPower(0.9);
                        liftR.setPower(0.9);
                    })
                    .addTemporalMarker(2.3, ()->{ //claw opens 2.2 seconds into program, during spline
                        intake.setPosition(open);
                    })
                    .splineTo(new Vector2d(28, 5), Math.toRadians(235)) //spline to scoring position
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(59, 12,25.2))
                    .addTemporalMarker(2.7, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(2.85, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })

                    .addTemporalMarker(5.1, ()->{
                        intake.setPosition(close);
                    })
                    //stopped here
                    .waitSeconds(2)

                    .addTemporalMarker(5.3, ()->{
                        liftN.setPower(0.75);
                        liftR.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(36, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(32, 10,Math.toRadians(232)))

                    .lineToLinearHeading(new Pose2d(27.5, 5.25, Math.toRadians(232))) //line to score (direct to pole)
                    .addTemporalMarker(10, ()->{
                        intake.setPosition(open);})
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(59, 13,25.2))
                    .addTemporalMarker(10.5, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(10.65, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })


                    .addTemporalMarker(13, ()->{
                        intake.setPosition(close);
                    })
                    .waitSeconds(2)
                    .addTemporalMarker(13.2, ()->{
                        liftN.setPower(0.75);
                        liftR.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(36, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(32, 10,Math.toRadians(230)))

                    .lineToLinearHeading(new Pose2d(27.5, 5.25, Math.toRadians(230))) //line to score (direct to pole)
                    .addTemporalMarker(17.9, ()->{
                        intake.setPosition(open);})
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(58, 13,25.2))
                    .addTemporalMarker(18.8, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(19.2, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })
                    .build();

            drive.followTrajectorySequence(traj1);


            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(50);
        } else if (tagOfInterest.id == MIDDLE) {
            //trajectory
            waitForStart();
            Pose2d startPose = new Pose2d(26, 61, Math.toRadians(272));
            drive.setPoseEstimate(startPose);

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(26, 61,Math.toRadians(272)))
                    .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                        intake.setPosition(close);
                    })
                    .splineTo(new Vector2d(34, 32), Math.toRadians(270)) //spline to straight position
                    .splineTo(new Vector2d(34, 22), Math.toRadians(270)) //spline to straight position
                    .addTemporalMarker(1.1, () ->{ //lift goes up 1.1 seconds into program, during spline
                        liftN.setPower(0.9);
                        liftR.setPower(0.9);
                    })
                    .addTemporalMarker(2.3, ()->{ //claw opens 2.2 seconds into program, during spline
                        intake.setPosition(open);
                    })
                    .splineTo(new Vector2d(28, 5), Math.toRadians(235)) //spline to scoring position
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(59, 12,25.2))
                    .addTemporalMarker(2.7, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(2.85, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })

                    .addTemporalMarker(5.1, ()->{
                        intake.setPosition(close);
                    })
                    //stopped here
                    .waitSeconds(2)

                    .addTemporalMarker(5.3, ()->{
                        liftN.setPower(0.75);
                        liftR.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(36, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(32, 10,Math.toRadians(232)))

                    .lineToLinearHeading(new Pose2d(27.5, 5.25, Math.toRadians(232))) //line to score (direct to pole)
                    .addTemporalMarker(10, ()->{
                        intake.setPosition(open);})
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(59, 13,25.2))
                    .addTemporalMarker(10.5, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(10.65, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })


                    .addTemporalMarker(13, ()->{
                        intake.setPosition(close);
                    })
                    .waitSeconds(2)
                    .addTemporalMarker(13.2, ()->{
                        liftN.setPower(0.75);
                        liftR.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(36, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(32, 10,Math.toRadians(230)))

                    .lineToLinearHeading(new Pose2d(27.5, 5.25, Math.toRadians(230))) //line to score (direct to pole)
                    .addTemporalMarker(17.9, ()->{
                        intake.setPosition(open);})
                    .back(9)
                    .lineToLinearHeading(new Pose2d(34, 13,Math.toRadians(90)))
                    .addTemporalMarker(18.8, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(19.2, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })
                    .build();

            drive.followTrajectorySequence(traj1);



            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(50);
        }else{
            waitForStart();
            Pose2d startPose = new Pose2d(26, 61, Math.toRadians(272));
            drive.setPoseEstimate(startPose);

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(26, 61,Math.toRadians(272)))
                    .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                        intake.setPosition(close);
                    })
                    .splineTo(new Vector2d(34, 32), Math.toRadians(270)) //spline to straight position
                    .splineTo(new Vector2d(34, 22), Math.toRadians(270)) //spline to straight position
                    .addTemporalMarker(1.1, () ->{ //lift goes up 1.1 seconds into program, during spline
                        liftN.setPower(0.9);
                        liftR.setPower(0.9);
                    })
                    .addTemporalMarker(2.3, ()->{ //claw opens 2.2 seconds into program, during spline
                        intake.setPosition(open);
                    })
                    .splineTo(new Vector2d(28, 5), Math.toRadians(235)) //spline to scoring position
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(59, 12,25.2))
                    .addTemporalMarker(2.7, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(2.85, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })

                    .addTemporalMarker(5.1, ()->{
                        intake.setPosition(close);
                    })
                    //stopped here
                    .waitSeconds(2)

                    .addTemporalMarker(5.3, ()->{
                        liftN.setPower(0.75);
                        liftR.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(36, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(32, 10,Math.toRadians(232)))

                    .lineToLinearHeading(new Pose2d(27.5, 5.25, Math.toRadians(232))) //line to score (direct to pole)
                    .addTemporalMarker(10, ()->{
                        intake.setPosition(open);})
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(59, 13,25.2))
                    .addTemporalMarker(10.5, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(10.65, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })


                    .addTemporalMarker(13, ()->{
                        intake.setPosition(close);
                    })
                    .waitSeconds(2)
                    .addTemporalMarker(13.2, ()->{
                        liftN.setPower(0.75);
                        liftR.setPower(0.75);
                    })
                    .lineToLinearHeading(new Pose2d(36, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(32, 10,Math.toRadians(230)))

                    .lineToLinearHeading(new Pose2d(27.5, 5.25, Math.toRadians(230))) //line to score (direct to pole)
                    .addTemporalMarker(17.9, ()->{
                        intake.setPosition(open);})
                    .back(9)
                    .lineToLinearHeading(new Pose2d(36, 10,25.27))
                    .lineToLinearHeading(new Pose2d(10, 10,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(11, 11,Math.toRadians(90)))

                    .addTemporalMarker(18.8, ()->{ //lift goes down during cone stack turn
                        liftN.setPower(-.5);
                        liftR.setPower(-.5);
                    })
                    .addTemporalMarker(19.2, ()->{ //lift stops
                        liftN.setPower(0);
                        liftR.setPower(0);
                    })
                    .build();

            drive.followTrajectorySequence(traj1);


        }

        telemetry.addData("Path", "Complete");
        telemetry.update();


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
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


