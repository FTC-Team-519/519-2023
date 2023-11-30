package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Autonomous(name="TestWithTSE Detection", group="Iterative OpMode")

public class BaseAuto extends OpMode {
    protected boolean USE_WEBCAM = true;
    protected TestVisionProcessor teamScoringElementFinder;
    protected VisionPortal portal;
    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected AprilTagProcessor aprilTagProcessor;

    static final double     COUNTS_PER_MOTOR_REV    = 1425.1  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void init() {
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");

        teamScoringElementFinder = new TestVisionProcessor(telemetry, true); // Need to change the looking for red value depending on what color we are
        initAprilTag();

        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), teamScoringElementFinder, aprilTagProcessor);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        if (teamScoringElementFinder.getCenter()) {

        }
    }

    @Override
    public void stop() {

    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
//        if (USE_WEBCAM) {
//            portal = VisionPortal.easyCreateWithDefaults(
//                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
//        } else {
//            portal = VisionPortal.easyCreateWithDefaults(
//                    BuiltinCameraDirection.BACK, aprilTagProcessor);
//        }
    }
}
