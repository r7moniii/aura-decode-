package org.firstinspires.ftc.teamcode.Benghazi.TeleOp;

import static org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants.pinpoint.currentHeading;
import static org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants.pinpoint.distance;
import static org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants.pinpoint.relative_angle;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Benghazi.Modules.Constants;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Intake;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Pinpoint;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;

@TeleOp
public class Blue extends LinearOpMode {
    SampleMecanumDrive drive = null;
    Intake intake = null;
    Outtake outtake = null;
    ElapsedTime timer;
    Pinpoint pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.init();
        outtake.init();
        pinpoint.init();

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        boolean start_intake=false, stop_intake=false, reverse_intake=false;

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            pinpoint.update_blue();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.7
                    )
            );

            drive.update();

            if(gamepad1.right_trigger>0.01 && !start_intake) {
                intake.in(1.0);
            }
            start_intake=gamepad1.right_trigger>0.01;

            if(gamepad1.left_trigger>0.01 && !stop_intake) {
                intake.stop();
            }
            stop_intake=gamepad1.left_trigger>0.01;

            if(gamepad1.left_bumper && !reverse_intake) {
                intake.out(1.0);
            }
            reverse_intake=gamepad1.left_bumper;

            if(gamepad1.a) {
                while(currentHeading>relative_angle) {
                    pinpoint.update_blue();
                    drive.setDrivePower(new Pose2d(
                            0,
                            0,
                            -0.7
                    ));
                    drive.update();
                }
                outtake.unblock();
                if(distance<10) {
                    outtake.shoot(1);
                }
                else if(distance<30) {
                    outtake.shoot(0);
                }
                timer.reset();
                intake.stop();
                drive.setWeightedDrivePower(
                        new Pose2d(
                                0,
                                0,
                                0
                        )
                );
                drive.update();
                while(timer.seconds()<1.5) {
                    if(timer.seconds()>0.9) {
                        outtake.unblock();
                    }
                    pinpoint.update_blue();
                }
                intake.in(1.0);
                timer.reset();
                while(timer.seconds()<1.3) {
                    pinpoint.update_blue();
                    drive.update();
                }
                outtake.block();
                intake.stop();
                outtake.stop();
            }

            telemetry.addData("pos x: ", Constants.pinpoint.currentX);
            telemetry.addData("pos y: ", Constants.pinpoint.currentY);
            telemetry.addData("heading: ", Constants.pinpoint.currentHeading);
            telemetry.addData("distance: ", distance);
            telemetry.addData("angle: ", relative_angle);
            telemetry.update();
        }
    }
}
