package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "auto")
@Disabled
public class accuracyTest extends LinearOpMode {
    //public double nineAngle = Math.toRadians(90);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor liftN = hardwareMap.get(DcMotor.class, "liftN");
        DcMotor liftR = hardwareMap.get(DcMotor.class, "liftR");
        Servo intake = hardwareMap.get(Servo.class, "intake");
        liftN.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        double open = 0.6;
        double close = 0.98;
        waitForStart();

        if (isStopRequested()) return;
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
                .addTemporalMarker(2.9, ()->{ //lift stops
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
                .addTemporalMarker(10.85, ()->{ //lift stops
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

    }
}
