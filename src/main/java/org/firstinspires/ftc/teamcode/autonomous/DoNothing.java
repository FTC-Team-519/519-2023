package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="Do nothing", group="Iterative OpMode")
//@Disabled
public class DoNothing extends OpMode {;
    protected static final double MIN_VALUE_FOR_WRIST_SERVO = 0.27;
    protected static final double MAX_VALUE_FOR_WRIST_SERVO = 0.85;
    protected static final double INT_VALUE_FOR_WRIST = 0.79;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;
    @Override
    public void init() {
        wristServoDroneSide = hardwareMap.get(Servo.class, "wristServoDroneSide");
        wristServoControlHubSide = hardwareMap.get(Servo.class, "wristServoControlHubSide");

        wristServoControlHubSide.setDirection(Servo.Direction.REVERSE);

        moveWrist(INT_VALUE_FOR_WRIST);
    }


    @Override
    public void loop() {

    }
    private void moveWrist(double position) {
        wristServoDroneSide.setPosition(position);
        wristServoControlHubSide.setPosition(position);
    }
}
