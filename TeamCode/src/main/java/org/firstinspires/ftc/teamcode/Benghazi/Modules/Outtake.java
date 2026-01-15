package org.firstinspires.ftc.teamcode.Benghazi.Modules;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Configurable
public class Outtake extends Constants.outtake {
    HardwareMap hardwareMap;
    public Outtake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotorEx shooterMotorRight, shooterMotorLeft;
    Servo servo;
    ElapsedTime timer;
    PIDFController controller = new PIDFController(kp, 0, 0, 0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

    public void init() {
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "sho1");
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "sho2");
        servo = hardwareMap.get(Servo.class, "servo");
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);


        block();
        near();

        target_velocity = 0;
    }

    public void update_pid() {
        velocity = shooterMotorRight.getVelocity();

        controller.setPIDF(kp, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        double PID_output = controller.calculate(velocity, target_velocity);
        double ff_output = feedforward.calculate(target_velocity);
        double output = PID_output + ff_output;

        if(target_velocity != 0) {
            shooterMotorRight.setPower(output * (nominalvoltage / voltage));
            shooterMotorLeft.setPower(output * (nominalvoltage / voltage));
        }

        else {
            shooterMotorRight.setPower(0);
            shooterMotorLeft.setPower(0);
        }
    }

    public void shoot(double power) {
        shooterMotorRight.setPower(power);
        shooterMotorLeft.setPower(power);
    }

    public void stop() {
        shooterMotorRight.setPower(0.0);
        shooterMotorLeft.setPower(0.0);
    }


    public void near() {
        target_velocity = 1200;
    }

    public void block() {
        servo.setPosition(block);
    }

    public void unblock() {
        servo.setPosition(unblock);
    }

    public boolean isBusy() {
        if (servo.getPosition() < 0.38) {
            return true;
        }
        return false;
    }
}
