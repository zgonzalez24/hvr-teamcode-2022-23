package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "drive")
public class blueLeft extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(28, 61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {//close claw during forward action
                    intake.setPosition(close);
                })
                .splineTo(new Vector2d(36, 32), Math.toRadians(270))//spline to straight position
                .splineTo(new Vector2d(36, 22), Math.toRadians(270))//spline to straight position

                .addTemporalMarker(1.1, () ->{//lift goes up 1.1 secs into program
                    liftN.setPower(0.8);
                    liftR.setPower(0.8);
                })
                .addTemporalMarker(2.2, ()->{//claw opens 2.2 secs into program, during spline
                    intake.setPosition(open);
                })
                .splineTo(new Vector2d(26.5, 5.5), Math.toRadians(235))//spline to scoring pos
                .back(8)//back before cone stack turn so no crash
// cone stack line begins
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack

                .lineToLinearHeading( new Pose2d(59, 10, 0))//line to cone stack
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack

                .addTemporalMarker(2.7, ()->{//lift goes up during cone stack turn
                    liftN.setPower(-.5);
                    liftR.setPower(-.5);
                })
                .addTemporalMarker(2.9, ()->{//lift stops
                    liftN.setPower(0);
                    liftR.setPower(0);
                })
//pole score line begins
                 .lineToLinearHeading( new Pose2d(32, 11.5, Math.toRadians(235)))//line to set up to score (not direct to pole)
                .addTemporalMarker(4.9, ()->{
                    intake.setPosition(close);
                })
                .addTemporalMarker(5.2, ()->{
                    liftN.setPower(0.75);
                    liftR.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(26.5, 5.5, Math.toRadians(235)))//line to score (direct to pole)
                .addTemporalMarker(8, ()->{//during line^ that goes to score
                    intake.setPosition(open);
                })
//cone stack line begins
                .back(8) //back before line to cone stack so no crash
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack

                .lineToLinearHeading( new Pose2d(59, 10, 0)) //line up to cone stack
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack

                .addTemporalMarker(9.6, ()->{ //lift goes down during cone stack turn
                    liftN.setPower(-.5);
                    liftR.setPower(-.5);
                })
                .addTemporalMarker(10, ()->{ // lift stops
                    liftN.setPower(0);
                    liftR.setPower(0);
                })
 //pole score line begins
                .lineToLinearHeading(new Pose2d(32, 11.5, Math.toRadians(235))) //line to set up to score (not direct to pole)
                .addTemporalMarker(11, ()->{ //close around cone stack
                    intake.setPosition(close);
                })
                .addTemporalMarker(11.2, ()->{ //lift cone from stack
                    liftN.setPower(0.75);
                    liftR.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(26.5, 5.5, Math.toRadians(235))) //line to score (direct to pole)
                .addTemporalMarker(14, ()->{//during line^ that goes to score
                    intake.setPosition(open);
                })
                .back(8) //back before line to cone stack so no crash
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack
                .lineToLinearHeading(new Pose2d(59, 10, 0)) //line up to cone stack
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack
                .addTemporalMarker(16, ()->{ //lift goes down during cone stack turn
                    liftN.setPower(-.5);
                    liftR.setPower(-.5);
                })
                .addTemporalMarker(16.4, ()->{ // lift stops
                    liftN.setPower(0);
                    liftR.setPower(0);
                })
                //pole score line begins
                .lineToLinearHeading(new Pose2d(32, 11.5, Math.toRadians(235))) //line to set up to score (not direct to pole)
                .addTemporalMarker(17, ()->{ //close around cone stack
                    intake.setPosition(close);
                })
                .addTemporalMarker(17.4, ()->{ //lift cone from stack
                    liftN.setPower(0.75);
                    liftR.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(26.5, 5.5, Math.toRadians(235))) //line to score (direct to pole)
                .addTemporalMarker(19.4, ()->{
                    //during line^ that goes to score
                    intake.setPosition(open);
                })
                //cone stack line begins
                .back(8) //back before line to cone stack so no crash
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack
                .lineToLinearHeading(new Pose2d(59, 10, 0)) //line up to cone stack
                .lineToLinearHeading(new Pose2d(40, 10, 0))//line to cone stack
                .addTemporalMarker(22, ()->{ //lift goes down during cone stack turn
                    liftN.setPower(-.5);
                    liftR.setPower(-.5);
                })
                .addTemporalMarker(22.4, ()->{ // lift stops
                    liftN.setPower(0);
                    liftR.setPower(0);
                })
                //pole score line begins
                .lineToLinearHeading(new Pose2d(32, 11.5, Math.toRadians(235))) //line to set up to score (not direct to pole)
                .addTemporalMarker(23.8, ()->{ //close around cone stack
                    intake.setPosition(close);
                })
                .addTemporalMarker(24.2, ()->{ //lift cone from stack
                    liftN.setPower(0.75);
                    liftR.setPower(0.75);
                })
                .lineToLinearHeading(new Pose2d(26.5, 5.5, Math.toRadians(235))) //line to score (direct to pole)
                .addTemporalMarker(26, ()->{
                    //during line^ that goes to score
                    intake.setPosition(open);
                })
                //end park segment
                .back(8)

                .build();


        drive.followTrajectorySequence(traj1);
    }
}

