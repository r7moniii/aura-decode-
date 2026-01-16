package org.firstinspires.ftc.teamcode.Benghazi.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Benghazi.Modules.Intake;
import org.firstinspires.ftc.teamcode.Benghazi.Modules.Outtake;

@TeleOp
public class MotorTest extends OpMode {
    DcMotorEx shooterMotorRight, shooterMotorLeft;
    Intake intake;

    double x=0;
    @Override
    public void init() {
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "sho1");
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "sho2");
        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = new Intake(hardwareMap);
        intake.init();
    }
    boolean a=false, b=false;
    @Override
    public void loop() {
        intake.in(0.6);
        if(gamepad1.dpad_up && !a) {
            x+=0.01;
        }
        a=gamepad1.dpad_up;
        if(gamepad1.dpad_down && !b) {
            x-=0.01;
        }
        b=gamepad1.dpad_down;
        shooterMotorRight.setPower(x);
        shooterMotorLeft.setPower(x);
        telemetry.addData("power: ", x);
        telemetry.addData("velocity: ", shooterMotorLeft.getVelocity());
        telemetry.addData("velocity: ", shooterMotorRight.getVelocity());
        telemetry.update();
    }
}
