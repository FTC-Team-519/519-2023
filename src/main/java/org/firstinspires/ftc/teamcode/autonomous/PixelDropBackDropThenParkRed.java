package org.firstinspires.ftc.teamcode.autonomous;

public class PixelDropBackDropThenParkRed extends PixelDropNoMoveAfterRed {
    private DriveToAprilTag tagDetection = new DriveToAprilTag(telemetry);
    private static final int LEFT_RED_TAG_ID = 4;
    private static final int CENTER_RED_TAG_ID = 5;
    private static final int RIGHT_RED_TAG_ID = 6;

    private enum ParkingSteps {
        GO_FORWARD_INCHS,
        STRAFE_RIGHT, // Might need to make this be turn left
        TURN_RIGHT,
        SCORE_ON_BACKDROP,
        FIND_AND_GO_TO_TAG,
        STRAFE_RIGHT_FOR_BACKDROP,
        PARK,
        DONE
    }

    private ParkingSteps parkingSteps = ParkingSteps.GO_FORWARD_INCHS;
    @Override
    public void init() {
        super.init();
        tagDetection.init(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive, aprilTagProcessor);
        parkingSteps = ParkingSteps.GO_FORWARD_INCHS;
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {
        if (!isDone()) {
            super.loop();
        } else {
            switch (parkingSteps) {
                case GO_FORWARD_INCHS:
                    driveDistanceInches(0.1, 2);
                    if (runtime.milliseconds() > 100) {
                        runtime.reset();
//                                parkingSteps = ParkingSteps.STRAFE_RIGHT;
                        parkingSteps = ParkingSteps.STRAFE_RIGHT;
                    }
                    break;
                case STRAFE_RIGHT:
                    strafeRight(48, .5);
                    if (runtime.milliseconds() > 15000) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.FIND_AND_GO_TO_TAG;
                    }
                    break;
                case TURN_RIGHT:
                    turnRight(0.1, 90);
                    if (runtime.milliseconds() > 1000) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.FIND_AND_GO_TO_TAG;
                    }
                    break;
                case FIND_AND_GO_TO_TAG:
                    boolean isFound = false;
                    if (positionOfTheTSE == PositionOfTSE.LEFT) {
                        isFound = tagDetection.moveToTargetId(CENTER_RED_TAG_ID);
                    }else if (positionOfTheTSE == PositionOfTSE.CENTER) {
                        isFound = tagDetection.moveToTargetId(RIGHT_RED_TAG_ID);
                    }else if (positionOfTheTSE == PositionOfTSE.RIGHT) {
                        isFound = tagDetection.moveToTargetId(RIGHT_RED_TAG_ID);
                    }
                    if (isFound && positionOfTheTSE == PositionOfTSE.RIGHT) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.STRAFE_RIGHT_FOR_BACKDROP;
                    }else if (isFound) {
                        parkingSteps = ParkingSteps.STRAFE_RIGHT_FOR_BACKDROP;
                    }
                    break;
                case STRAFE_RIGHT_FOR_BACKDROP:
                    strafeRight((int)(6.5 * COUNTS_PER_INCH), 0.25);
                    if (runtime.milliseconds() > 500) {
                        parkingSteps = ParkingSteps.SCORE_ON_BACKDROP;
                    }
                    break;
                case SCORE_ON_BACKDROP:

                    if (runtime.milliseconds() > 15000) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.DONE;
                    }
                    break;
            }
        }
    }

    protected boolean ParkingDone() {
        return parkingSteps == ParkingSteps.DONE;
    }
}
