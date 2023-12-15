package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;


public class BaseAuto extends OpMode {
    protected static final double MAX_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.97;
    protected static final double MIN_VALUE_FOR_SERVO_PIXEL_DROPPER = 0.31;
    protected static final double CLOSED_VALUE_FOR_PIXEL_DROPPER = 0.34;
    protected static final double OPEN_VALUE_FOR_PIXEL_DROPPER = 0.40;
    protected static final double MIN_VALUE_FOR_WRIST_SERVO = 0.27;
    protected static final double MAX_VALUE_FOR_WRIST_SERVO = 0.85;

    protected AprilTagProcessor aprilTagProcessor;
    protected TestVisionProcessor teamScoringElementFinder;
    protected VisionPortal portal;

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;

    protected Servo autoPixelServo = null;
    protected ColorSensor pixelDropperColorSensor;

    protected IMU imu = null;

    boolean orientationIsValid = false;
    //protected Orientation angles;
    double yaw;
    double pitch;
    double roll;

    private boolean goingCenter = false;
    private boolean goingLeft = false;
    private boolean goingRight = false;

    protected boolean onRedTeam = true;

    protected int positionOfTheTSE = 2; // 1 for left 2 for center 3 for right
    protected int step = 1;

    protected ElapsedTime runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 1425.1  ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    private static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    protected double  driveSpeed    = 0.25;

    protected double angleOffset;

    protected Servo wristServoControlHubSide;
    protected Servo wristServoDroneSide;
    @Override
    public void init() {
        autoPixelServo = hardwareMap.get(Servo.class, "pixelDropperServo");
        wristServoDroneSide = hardwareMap.get(Servo.class, "wristServoDroneSide");
        wristServoControlHubSide = hardwareMap.get(Servo.class, "wristServoControlHubSide");

        pixelDropperColorSensor = hardwareMap.get(ColorSensor.class, "pixelColorSensor");

        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        wristServoControlHubSide.setDirection(Servo.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imuExpan");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imu.initialize(parameters);
        orientationIsValid = false;
        yaw = 0.0d;
        pitch = 0.0d;
        roll = 0.0d;

        try {
            RevHubOrientationOnRobot orientationOnRobot =
                    new RevHubOrientationOnRobot(LogoFacingDirection.RIGHT, UsbFacingDirection.BACKWARD);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }

        initAprilTag();

        autoPixelServo.setPosition(CLOSED_VALUE_FOR_PIXEL_DROPPER);

        angleOffset = 0;

        step = 1;

        moveWrist(MIN_VALUE_FOR_WRIST_SERVO);

        initAprilTag();
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

        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        YawPitchRollAngles yawPitchRollAngles = imu.getRobotYawPitchRollAngles();
        yaw = yawPitchRollAngles.getYaw(AngleUnit.DEGREES);
        pitch = yawPitchRollAngles.getPitch(AngleUnit.DEGREES);
        roll = yawPitchRollAngles.getRoll(AngleUnit.DEGREES);
        switch (positionOfTheTSE) {
            case 1: // Going to the left
                switch (step) {
                    case 1:
                        driveDistanceTime(driveSpeed, 5000, runtime);
                        break;
                    case 2:
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        turnLeft(0.25, 45);
                        if (runtime.seconds() > 4.0) {
                            step++;
                        }
                        break;
                    case 4:
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        setAllDrivePower(0.25);
                        if ((onRedTeam && seeingRed()) || (!onRedTeam && seeingBlue())) {
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
                }
                break;


            case 2: // Going to Center
                switch (step) {
                    case 1:
                        driveDistanceTime(driveSpeed, 5000, runtime);
                        break;
                    case 2:
                        autoPixelServo.setPosition(OPEN_VALUE_FOR_PIXEL_DROPPER);
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        setAllDrivePower(-0.25);
                        if (seeingGrey()) {
                            setAllDrivePower(0.0);
                        }
                        step++;
                        break;
                }
                break;


            case 3: // Going to the right
                switch (step) {
                    case 1:
                        driveDistanceTime(driveSpeed, 5000, runtime);
                        break;
                    case 2:
                        runtime.reset();
                        step++;
                        break;
                    case 3:
                        turnRight(0.25, 45);
                        if (runtime.seconds() > 4.0) {
                            step++;
                        }
                        break;
                    case 4:
                        runtime.reset();
                        step++;
                        break;
                    case 5:
                        setAllDrivePower(0.25);
                        if ((onRedTeam && seeingRed()) || (!onRedTeam && seeingBlue())) {
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
                }
        }
        telemetry.addData("Step:", step);

        telemetry.addData("Heading", yaw);
        telemetry.addData("Pitch", pitch);
        telemetry.addData("Roll", roll);
        telemetry.update();
    }

    @Override
    public void stop() {}

    protected void driveDistanceTime(double speed, double desiredTimeMiliseconds, ElapsedTime timeElapsed) {
        if (timeElapsed.milliseconds() >= desiredTimeMiliseconds) {
            step++;
            setAllDrivePower(0.0);
        }else {
            setAllDrivePower(speed);
        }
    }

    protected void driveDistanceInches(double speed, double distanceInches) {
        setTargetPosition((int)(distanceInches * COUNTS_PER_INCH));
    }

    protected void setTargetPosition(int position) {
        leftFrontDrive.setTargetPosition(position);
        leftBackDrive.setTargetPosition(position);
        rightFrontDrive.setTargetPosition(position);
        rightBackDrive.setTargetPosition(position);
    }

    protected void turnRight(double speed, double degree) {
        double targetDegree = angleOffset - degree;
        telemetry.addData("target degree", targetDegree);
        telemetry.addData("angle offset", angleOffset);
        if (yaw >= targetDegree) {
            setLeftDrivesPower(speed);
            setRightDrivesPower(-speed);
        } else {
            setAllDrivePower(0.0);
        }
    }

    protected void turnLeft(double speed, double degree) {
        double targetDegree = angleOffset + degree;
        telemetry.addData("target degree", targetDegree);
        telemetry.addData("angle offset", angleOffset);
        if (yaw <= targetDegree) {
            setLeftDrivesPower(-speed);
            setRightDrivesPower(speed);
        } else {
            setAllDrivePower(0.0);
        }
    }

    protected void setAllDrivePower(double power) {
        setLeftDrivesPower(power);
        setRightDrivesPower(power);
    }

    protected void setLeftDrivesPower(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
    }

    protected void setRightDrivesPower(double power) {
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    private void setMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    protected boolean seeingBlue() {
        return pixelDropperColorSensor.blue() > pixelDropperColorSensor.red() && pixelDropperColorSensor.blue() > pixelDropperColorSensor.green();
    }

    protected boolean seeingGrey() {
        return pixelDropperColorSensor.green() > pixelDropperColorSensor.red() && pixelDropperColorSensor.green() > pixelDropperColorSensor.blue();
    }

    protected boolean seeingRed() {
        return pixelDropperColorSensor.red() > pixelDropperColorSensor.green() && pixelDropperColorSensor.red() > pixelDropperColorSensor.blue();
    }

    protected void strafeLeft(int distanceInches, double power){
        leftFrontDrive.setTargetPosition(-distanceInches);
        leftBackDrive.setTargetPosition(distanceInches);
        rightFrontDrive.setTargetPosition(distanceInches);
        rightBackDrive.setTargetPosition(-distanceInches);
        setAllDrivePower(power
        );
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void strafeRight(int value, double power){
        leftFrontDrive.setTargetPosition(value);
        leftBackDrive.setTargetPosition(-value);
        rightFrontDrive.setTargetPosition(-value);
        rightBackDrive.setTargetPosition(value);
        setAllDrivePower(power);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void moveWrist(double position) {
        wristServoDroneSide.setPosition(position);
        wristServoControlHubSide.setPosition(position);
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
