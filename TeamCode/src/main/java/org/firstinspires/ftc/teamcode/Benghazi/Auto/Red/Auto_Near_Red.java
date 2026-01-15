package org.firstinspires.ftc.teamcode.Benghazi.Auto.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Benghazi.Modules.Intake;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class Auto_Near_Red extends OpMode {
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
    Path9;

    public void buildPaths() {
        Path1 = new Path(new BezierLine(new Pose(8.6, 134.7), new Pose(60, 80)));
        Path1.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(142));

        Path2 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(60, 80),
                        new Pose(13, 84)
                    )
            ).setTangentHeadingInterpolation()
            .build();

        Path3 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(13, 84),
                        new Pose(60, 80)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
            .build();

        Path4 = follower.pathBuilder().addPath(
                    new BezierCurve(
                        new Pose(60, 80),
                        new Pose(64.000, 70.000),
                        new Pose(57, 60)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
            .build();

        Path5 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(49.252, 60),
                        new Pose(13, 60)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(13, 60),
                    new Pose(60, 80)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
            .build();

        Path7 = follower.pathBuilder().addPath(
                new BezierCurve(
                    new Pose(60.000, 80),
                    new Pose(73.861, 53.690),
                    new Pose(50.000, 36)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
            .build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(50.000, 36),
                        new Pose(13.000, 36)
                )
            ).setTangentHeadingInterpolation()
            .build();

        Path9 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(13.000, 36),
                        new Pose(60.000, 80.000)
                    )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
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
                    outtake.shoot(1.0);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                    intake.stop();
                    while (timer.seconds() < 1.5) {
                        if(timer.seconds()>1) {
                            outtake.unblock();
                        }
                        continue;
                    }
                    intake.in(1.0);
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
                outtake.block();
                while (outtake.isBusy()) {
                    continue;
                }
                intake.in(1.0);
                setPathState(3);
                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(Path2);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Path3);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    outtake.shoot(1.0);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    intake.stop();
                    while (timer.seconds() < 1.5) {
                        if(timer.seconds()>1) {
                            outtake.unblock();
                        }
                        continue;
                    }
                    intake.in(1.0);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    setPathState(7);
                }
                break;

            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(Path4);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    outtake.block();
                    while (outtake.isBusy()) {
                        continue;
                    }
                    intake.in(1.0);
                    follower.followPath(Path5);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(10);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(Path6);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    outtake.unblock();
                    outtake.shoot(1.0);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                    intake.stop();
                    while (timer.seconds() < 1.5) {
                        if(timer.seconds()>1) {
                            outtake.unblock();
                        }
                        continue;
                    }
                    intake.in(1.0);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()) {
                    outtake.block();
                    while (outtake.isBusy()) {
                        continue;
                    }
                    intake.in(1.0);
                    follower.followPath(Path8);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intake.stop();
                    setPathState(15);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(Path9);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    outtake.unblock();
                    outtake.shoot(1.0);
                    ElapsedTime timer;
                    timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    timer.reset();
                    intake.stop();
                    while (timer.seconds() < 1.5) {
                        if(timer.seconds()>1) {
                            outtake.unblock();
                        }
                        continue;
                    }
                    intake.in(1.0);
                    timer.reset();
                    while (timer.seconds() < 1.3) {
                        continue;
                    }
                    intake.stop();
                    outtake.stop();
                    setPathState(17);
                }
                break;

        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake.init();
        outtake.init();

        buildPaths();
        follower.setStartingPose(new Pose(8.600, 134.700, Math.toRadians(-90)));
    }

    @Override
    public void loop() {
        outtake.update_pid();
        follower.update();
        autonomousPathUpdate();
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
