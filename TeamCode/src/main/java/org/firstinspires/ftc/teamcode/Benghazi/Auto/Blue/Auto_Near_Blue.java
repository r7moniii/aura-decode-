package org.firstinspires.ftc.teamcode.Benghazi.Auto.Blue;

import static org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants.pinpoint.distance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Intake;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Outtake;

@Autonomous
public class Auto_Near_Blue extends OpMode {
    Intake intake;
    Outtake outtake;

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double shoot_duration=2;

    public static double x_startPose=8.6, y_startPose=134.7, heading_StartPose=-90;
    public static double x_shoot=60, y_shoot=84, heading_shoot=145;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_StartPose));
    private final Pose shoot = new Pose(x_shoot, y_shoot, Math.toRadians(heading_shoot));

    Path Path1;
    PathChain
    Path2,
    Path3,
    Path4,
    Path5,
    Path6,
    Path7,
    Path8,
    Path9,
    Path10;

    public void buildPaths() {
        Path1 = new Path(new BezierLine(new Pose(29, 131.6), new Pose(48, 106)));
        Path1.setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(140));

        Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(48, 106),
                        new Pose(55, 84)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
            .build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(55.000, 84.000),
                    new Pose(17, 84.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(17, 84.000),
                    new Pose(48, 106)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(48, 106),
                        new Pose(55.000, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(55.000, 60.000),
                    new Pose(12, 60.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

        Path7=follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(12.000, 60.000),
                    new Pose(55.000, 55.000),
                    new Pose(48, 106)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(48, 106),
                    new Pose(55.000, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
            .build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(55.000, 36.000),
                    new Pose(12, 36.000)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

        Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(12, 36.000),
                    new Pose(48, 106)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
            .build();
    }



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(Path1, true);
                    setPathState(1);
                }
                break;

            case 1: {
                if (!follower.isBusy()) {
                    outtake.unblock();
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                    intake.stop();
                    outtake.shoot(0.83);
                    while (timer.seconds() < 1.5) {
                        continue;
                    }
                    intake.in(0.6);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    setPathState(2);
                }
                break;
            }

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(Path2);
                    setPathState(3);
                }
                break;

            case 3:
                outtake.block();
                while (outtake.isBusy()) {
                    continue;
                }
                intake.in(1.0);
                setPathState(4);
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {

                    setPathState(6);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake.stop();
                    follower.followPath(Path4);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    outtake.unblock();
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    intake.stop();
                    outtake.shoot(0.83);
                    while (timer.seconds() < 1.5) {
                        continue;
                    }
                    intake.in(0.6);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(Path5);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    outtake.block();
                    while (outtake.isBusy()) {
                        continue;
                    }
                    intake.in(1.0);
                    follower.followPath(Path6, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(11);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    outtake.unblock();
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                    intake.stop();
                    outtake.shoot(0.83);
                    while (timer.seconds() < 1.5) {
                        continue;
                    }
                    intake.in(0.6);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    outtake.update_pid();
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(Path8);
                    setPathState(14);
                }
                break;

            case 14:
                if(!follower.isBusy()) {
                    outtake.block();
                    while (outtake.isBusy()) {
                        continue;
                    }
                    intake.in(1.0);
                    follower.followPath(Path9, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(16);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(Path10);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    outtake.unblock();
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                    intake.stop();
                    outtake.shoot(0.83);
                    while (timer.seconds() < 1.5) {
                        continue;
                    }
                    intake.in(0.6);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    outtake.update_pid();
                    setPathState(18);
                }
                break;
//
        }
    }
    VoltageSensor voltageSensor;
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake.init();
        outtake.init();
        buildPaths();
        follower.setStartingPose(new Pose(29, 131.6, Math.toRadians(144)));

    }

    @Override
    public void loop() {
        outtake.update_pid();
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("pos x: ", follower.getPose().getX());
        telemetry.addData("pos y:", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());
        telemetry.addData("distance", distance);
        telemetry.addData("vel: ", velocity);
        telemetry.addData("voltage: ", voltage);
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
