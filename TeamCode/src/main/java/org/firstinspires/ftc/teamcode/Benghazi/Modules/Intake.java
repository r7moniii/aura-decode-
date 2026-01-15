package org.firstinspires.ftc.teamcode.Benghazi.Modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake extends Constants.intake {
    HardwareMap hardwareMap;
    public Intake (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotorEx motor_intake;
    DcMotorEx motor_transfer;
    public void init() {
        motor_intake = hardwareMap.get(DcMotorEx.class, "intake1");
        motor_transfer = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void in(double power) {
        motor_intake.setPower(power);
        motor_transfer.setPower(power);
    }

    public void out(double power) {
        motor_intake.setPower(-power);
        motor_transfer.setPower(-power);
    }

    public void stop() {
        motor_intake.setPower(0);
        motor_transfer.setPower(0);
    }

    public void stop_transfer() {
        motor_transfer.setPower(0);
    }

    public void start_transfer() {
        motor_transfer.setPower(0.6);
    }
}