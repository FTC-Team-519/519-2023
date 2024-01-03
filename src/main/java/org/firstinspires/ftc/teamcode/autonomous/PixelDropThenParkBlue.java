package org.firstinspires.ftc.teamcode.autonomous;

public class PixelDropThenParkBlue extends PixelDropNoMoveAfterBlue {
    private enum ParkingSteps {
        GO_FORWARD_INCHS,
        STRAFE_LEFT, // Might need to make this be turn left
        TURN_LEFT,
        GO_FORWARD_AND_PARK,
        DONE
    }

    private ParkingSteps parkingSteps = ParkingSteps.GO_FORWARD_INCHS;

    @Override
    public void init() {
        super.init();
        parkingSteps = ParkingSteps.GO_FORWARD_INCHS;
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {super.start();}

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
                        parkingSteps = ParkingSteps.TURN_LEFT;
                    }
                    break;
                case STRAFE_LEFT:
                    strafeRight(48, .5);
                    if (runtime.milliseconds() > 15000) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.DONE;
                    }
                    break;
                case TURN_LEFT:
                    turnRight(0.1, 90);
                    if (runtime.milliseconds() > 1000) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.GO_FORWARD_AND_PARK;
                    }
                    break;
                case GO_FORWARD_AND_PARK:
                    driveDistanceInches(driveSpeed, 48);
                    if (runtime.milliseconds() > 15000) {
                        runtime.reset();
                        parkingSteps = ParkingSteps.DONE;
                    }
                    break;
            }


        }
    }
}
