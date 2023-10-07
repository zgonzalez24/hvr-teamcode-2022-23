package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
@Disabled
public class autoTesting extends LinearOpMode {
    //public double nineAngle = Math.toRadians(90);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor liftN = hardwareMap.get(DcMotor.class, "liftN");
        DcMotor liftR = hardwareMap.get(DcMotor.class, "liftR");
        Servo intake = hardwareMap.get(Servo.class, "intake");
        liftN.setDirection(DcMotorSimple.Direction.FORWARD);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;
//        Trajectory splineToPosition = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .splineTo(new Vector2d(15,15), toRadians(90))
//                .build();
//        drive.followTrajectory(splineToPosition);
//
        Trajectory driveForward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .forward(50)
                .build();

        Trajectory driveForward2 = drive.trajectoryBuilder(driveForward.end().plus(new Pose2d(0,0,Math.toRadians(-45))))
                .forward(5)
                .build();

        Trajectory coneTurn = drive.trajectoryBuilder(driveForward2.end())
                .lineToLinearHeading(new Pose2d(50,20, Math.toRadians(90)))
                .build();

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0,0))
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    double close = 0.98;
                    intake.setPosition(close);
                })
                .splineToLinearHeading(new Pose2d(49, -2, Math.toRadians(-45)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    liftN.setPower(1);
                    liftR.setPower(1);
                })
                //
                .forward(5)
                .waitSeconds(0.8)
                //change from 3 to 6
                .forward(8)
                //pos 1 to neg
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () ->{
                    double open =0.6;
                    intake.setPosition(0.6);
                })
                .back(14)
                .UNSTABLE_addTemporalMarkerOffset(2, () ->{
                    liftN.setPower(-.5);
                    liftR.setPower(-.5);
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(1, () ->{
                    liftN.setPower(0);
                    liftR.setPower(0);
                })
                .lineToLinearHeading(new Pose2d(53,20, Math.toRadians(90)))
                .build();


//        drive.followTrajectory(driveForward);
//        drive.turn(Math.toRadians(-45));
//        drive.followTrajectory(driveForward2);
//        drive.followTrajectory(coneTurn);
        drive.followTrajectorySequence(traj1);

    }
}
