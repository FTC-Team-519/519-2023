package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@Autonomous(name="TestWithTSE Detection", group="Iterative OpMode")
//@Disabled
public class BaseAuto extends OpMode {
    protected boolean USE_WEBCAM = true;

    public static final double MAX_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.97;
    public static final double MIN_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.31;
    public static final double CLOSED_VALUE_FOR_PIXEL_DROPPER = 0.35;
    public static final double OPEN_VALUE_FOR_PIXEL_DROPPER = 0.38;
    public static final double MIN_VALUE_FOR_ARM_SERVO = 0.27;
    public static final double MAX_VALUE_FOR_ARM_SERVO = 0.85;

    protected TestVisionProcessor teamScoringElementFinder;
    protected VisionPortal portal;

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected AprilTagProcessor aprilTagProcessor;

    protected BNO055IMU imu = null;
    protected Orientation angles;

    protected boolean goingCenter;
    protected boolean goingLeft;
    protected boolean goingRight;

    protected int positionOfTheTSE = 2; // 1 for left 2 for center 3 for right
    protected int step = 1;

    protected Servo autoPixelServo = null;

    protected ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1425.1  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int leftFrontTarget = 0;
    private int rightFrontTarget = 0;
    private int leftBackTarget = 0;
    private int rightBackTarget = 0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    @Override
    public void init() {
        autoPixelServo = hardwareMap.get(Servo.class, "pixelDropperServo");

        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        teamScoringElementFinder = new TestVisionProcessor(telemetry, true); // Need to change the looking for red value depending on what color we are
        initAprilTag();

        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), teamScoringElementFinder, aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        if (teamScoringElementFinder.isCenter()) {
            goingCenter = true;
            goingLeft = false;
            goingRight = false;
        } else if (teamScoringElementFinder.isRight()) {
            goingRight = true;
            goingLeft = false;
            goingCenter = false;
        } else if (teamScoringElementFinder.isLeft()) {
            goingLeft = true;
            goingRight = false;
            goingCenter = false;
        }

    }

    @Override
    public void start() {
        if (goingLeft) {
            positionOfTheTSE = 1;
        }else if (goingCenter) {
            positionOfTheTSE = 2;
        }else if (goingRight) {
            positionOfTheTSE = 3;
        }
        runtime.reset();
    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        switch (positionOfTheTSE) {
            case 1: // Going to the left
                switch (step) {
                    case 1:
                        driveDistanceInches(driveSpeed, 16);
                        if (runtime.seconds() > 3) {
                            step++;
                        }
                        break;
                    case 2:
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        turnLeft(0.25, 90);
                        break;
                }
        }
//        if (goingCenter) {
//            driveDistanceInches(driveSpeed, 18);
//        } else if (goingLeft) {
//            driveDistanceInches(driveSpeed, 12);
//            turnLeft(0.25, 90);
//        }else if (goingRight) {
//            driveDistanceInches(driveSpeed, 18);
//        }

//        autoPixelServo.setPosition(0.0);
        telemetry.addData("Step:", step);
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.addData("Pitch", angles.secondAngle);
        telemetry.addData("Roll", angles.thirdAngle);
        telemetry.update();
    }

    @Override
    public void stop() {}

    public void turnLeft(double speed, double degree) {
        if (angles.firstAngle < degree) {
            setLeftDrivesPower(-speed);
            setRightDrivesPower(speed);
        }else {
            setAllDrivePower(0.0);
        }
    }

    public void driveDistanceInches(double speed, double distanceInches) {
        int moveCounts = (int)(distanceInches * COUNTS_PER_INCH);
        telemetry.addData("Target Position", leftFrontDrive.getCurrentPosition() + moveCounts);
        telemetry.addData("Current Position", leftFrontDrive.getCurrentPosition());

        leftFrontDrive.setTargetPosition(moveCounts);
        leftBackDrive.setTargetPosition(moveCounts);
        rightBackDrive.setTargetPosition(moveCounts);
        rightFrontDrive.setTargetPosition(moveCounts);

        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllDrivePower(0.5);


    }

    private void setAllDrivePower(double power) {
        setLeftDrivesPower(power);
        setRightDrivesPower(power);
    }

    private void setLeftDrivesPower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
    }

    private void setRightDrivesPower(double power) {
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void setMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
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
