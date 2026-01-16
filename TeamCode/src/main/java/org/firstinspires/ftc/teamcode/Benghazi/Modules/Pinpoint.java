package org.firstinspires.ftc.teamcode.Benghazi.Modules;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Pinpoint extends Constants.pinpoint {
    HardwareMap hardwareMap;
    public Pinpoint (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    GoBildaPinpointDriver pinpoint;

    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(91/25.4, -90/25.4, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    public void update_blue() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);
        velocityX = pinpoint.getVelX(DistanceUnit.INCH);
        velocityY = pinpoint.getVelY(DistanceUnit.INCH);
        deltaY = 144 - currentY;
        deltaX = 0 - currentX;
        relative_angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    public void update_red() {
        pinpoint.update();
        currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        currentX = pinpoint.getPosX(DistanceUnit.INCH);
        currentY = pinpoint.getPosY(DistanceUnit.INCH);
        velocityX = pinpoint.getVelX(DistanceUnit.INCH);
        velocityY = pinpoint.getVelY(DistanceUnit.INCH);
        deltaY = 144 - currentY;
        deltaX = 144 - currentX;
        relative_angle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
}
