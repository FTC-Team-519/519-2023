package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="Pixel Drop No Move After Blue Side", group="Iterative OpMode")
//@Disabled
public class PixelDropNoMoveAfterBlue extends OpMode {
    protected boolean USE_WEBCAM = true;

    public static final double MAX_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.97;
    public static final double MIN_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.31;
    public static final double CLOSED_VALUE_FOR_PIXEL_DROPPER = 0.34;
    public static final double OPEN_VALUE_FOR_PIXEL_DROPPER = 0.40;
    public static final double MIN_VALUE_FOR_ARM_SERVO = 0.27;
    public static final double MAX_VALUE_FOR_ARM_SERVO = 0.85;

    protected TestVisionProcessor teamScoringElementFinder;
    protected VisionPortal portal;

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected AprilTagProcessor aprilTagProcessor;

    protected ColorSensor pixelDropperColorSensor;

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

    private double angleOffset;

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

        pixelDropperColorSensor = hardwareMap.get(ColorSensor.class, "pixelColorSensor");

        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");

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

        teamScoringElementFinder = new TestVisionProcessor(telemetry, false); // Need to change the looking for red value depending on what color we are
        initAprilTag();

        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), teamScoringElementFinder, aprilTagProcessor);
        autoPixelServo.setPosition(CLOSED_VALUE_FOR_PIXEL_DROPPER);

        angleOffset = 0;
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
        teamScoringElementFinder.stop();
        runtime.reset();
    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        switch (positionOfTheTSE) {
            case 1: // Going to the left
                switch (step) {
                    case 1:
                        driveDistanceInches(driveSpeed, 15);
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
                        if (runtime.seconds() > 3.0) {
                            step++;
                        }
                        break;
                    case 4:
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        setAllDrivePower(0.25);
                        if (seeingRed()) {
                            setAllDrivePower(0.0);
                            step++;
                        }
                        break;
                    case 6:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        step++;
                        break;
                    case 7:
                        runtime.reset();
                        step++;
                        break;
                    case 8:
                        setAllDrivePower(-0.25);
                        if (seeingGrey() && runtime.seconds() > .5) {
                            setAllDrivePower(0.0);
                            step++;
                        }
                        break;
//               THIS CODE BELOW WILL NOT WORK IN THIS CONFIGURATION IT DRIVES OVER THE PIXEL BECAUSE IT DOESN"T DRIVE BACK FAR ENOUGH
//                    case 9:
//                        runtime.reset();
//                        angleOffset = angles.firstAngle;
//                        step++;
//                        break;
//                    case 10:
//                        turnRight(0.25, 90.0);
//                        if (runtime.seconds() > 3.0) {
//                            step++;
//                        }
//                        break;
//                    case 11:
//                        runtime.reset();
//                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        step++;
//                        break;
//                    case 12:
//                        driveDistanceInches(0.5, -18);
//                        if (runtime.seconds() > 3.0) {
//                            step++;
//                        }
//                        break;
//                    case 13:
//                        runtime.reset();
//                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        step++;
//                        break;
//                    case 14:
//                        driveDistanceInches(0.25, 3);
//                        if (runtime.seconds() > 1.0) {
//                            step++;
//                        }
//                        break;
                }
                break;




            case 2: // Going to Center
                switch (step){
                    case 1:
                        driveDistanceInches(driveSpeed, 16);
                        if (runtime.seconds() > 3) {
                            step++;
                        }
                        break;
                    case 2:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        setAllDrivePower(-0.25);
                        if (seeingGrey() && runtime.seconds() > .5) {
                            setAllDrivePower(0.0);
                            step++;
                        }
                        break;
                    case 4:
                        runtime.reset();
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        step++;
                        break;
                    case 5:
                        driveDistanceInches(0.5, -18);
                        if (runtime.seconds() > 3.0) {
                            step++;
                        }
                        break;
                    case 6:
                        runtime.reset();
                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        step++;
                        break;
                    case 7:
                        driveDistanceInches(0.25, 3);
                        if (runtime.seconds() > 1.0) {
                            setAllDrivePower(0.0);
                            step++;
                        }
                        break;

                }
                break;




            case 3:
                switch (step) {
                    case 1:
                        driveDistanceInches(driveSpeed, 15);
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
                        turnRight(0.25, 90);
                        if (runtime.seconds() > 3.0) {
                            step++;
                        }
                        break;
                    case 4:
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        setAllDrivePower(0.25);
                        if (seeingRed()) {
                            setAllDrivePower(0.0);
                            step++;
                        }
                        break;
                    case 6:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        step++;
                        break;
                    case 7:
                        runtime.reset();
                        step++;
                        break;
                    case 8:
                        setAllDrivePower(-0.25);
                        if (seeingGrey() && runtime.seconds() > .60) {
                            setAllDrivePower(0.0);
                            step++;
                        }
                        break;


//            THIS CODE BELOW WILL NOT WORK IN THIS CONFIGURATION IT DRIVES OVER THE PIXEL BECAUSE IT DOESN"T DRIVE BACK FAR ENOUGH
//                    case 9:
//                        runtime.reset();
//                        angleOffset = angles.firstAngle;
//                        step++;
//                        break;
//                    case 10:
//                        turnLeft(0.25, 90.0);
//                        if (runtime.seconds() > 3.0) {
//                            step++;
//                        }
//                        break;
//                    case 11:
//                        runtime.reset();
//                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        step++;
//                        break;
//                    case 12:
//                        driveDistanceInches(0.5, -18);
//                        if (runtime.seconds() > 3.0) {
//                            step++;
//                        }
//                        break;
//                    case 13:
//                        runtime.reset();
//                        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        step++;
//                        break;
//                    case 14:
//                        driveDistanceInches(0.25, 3);
//                        if (runtime.seconds() > 1.0) {
//                            step++;
//                        }
//                        break;
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

    public void turnRight(double speed, double degree) {
        double targetDegree = angleOffset - degree;
        telemetry.addData("target degree", targetDegree);
        telemetry.addData("angle offset", angleOffset);
        if (angles.firstAngle >= targetDegree) {
            setLeftDrivesPower(speed);
            setRightDrivesPower(-speed);
        } else {
            setAllDrivePower(0.0);
        }
    }

    public void turnLeft(double speed, double degree) {
        double targetDegree = angleOffset + degree;
        telemetry.addData("target degree", targetDegree);
        telemetry.addData("angle offset", angleOffset);
        if (angles.firstAngle <= targetDegree) {
            setLeftDrivesPower(-speed);
            setRightDrivesPower(speed);
        } else {
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

    private void driveToRed(double speed) {
        double startingPosition = leftFrontDrive.getCurrentPosition();
        setAllDrivePower(speed);
        if (seeingRed())  {
            setAllDrivePower(0.0);
        }
    }

    private boolean seeingBlue() {
        return pixelDropperColorSensor.blue() > pixelDropperColorSensor.red() && pixelDropperColorSensor.blue() > pixelDropperColorSensor.green();
    }

    private boolean seeingGrey() {
        return pixelDropperColorSensor.green() > pixelDropperColorSensor.red() && pixelDropperColorSensor.green() > pixelDropperColorSensor.blue();
    }

    private boolean seeingRed() {
        return pixelDropperColorSensor.red() > pixelDropperColorSensor.green() && pixelDropperColorSensor.red() > pixelDropperColorSensor.blue();
//        if (pixelDropperColorSensor.green() > pixelDropperColorSensor.red() && pixelDropperColorSensor.green() > pixelDropperColorSensor.blue()) {
//            telemetry.addData("Current Color", "Grey");
//            leftPower = 0.33;
//            rightPower = 0.33;
//        }
//        if (pixelDropperColorSensor.blue() > pixelDropperColorSensor.red() && pixelDropperColorSensor.blue() > pixelDropperColorSensor.green()) {
//            telemetry.addData("Current Color", "Blue");
//            leftPower = 0;
//            rightPower = 0;
//        }
    }

    private void strafeLeft(int distanceInches, double power){
        leftFrontDrive.setTargetPosition(-distanceInches);
        leftBackDrive.setTargetPosition(distanceInches);
        rightFrontDrive.setTargetPosition(distanceInches);
        rightBackDrive.setTargetPosition(-distanceInches);
        setAllDrivePower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void strafeRight(int value, double power){
        leftFrontDrive.setTargetPosition(value);
        leftBackDrive.setTargetPosition(-value);
        rightFrontDrive.setTargetPosition(-value);
        rightBackDrive.setTargetPosition(value);
        setAllDrivePower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
