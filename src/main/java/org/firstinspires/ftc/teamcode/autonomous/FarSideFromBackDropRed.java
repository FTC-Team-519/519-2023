package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FarSideFromBackDropRed extends PixelDropNoMoveAfterRed {

    @Override
    public void init() {
        super.init();
        teamScoringElementFinder.setToFarSide();
    }

    public void init_loop() {
        super.init_loop();
    }

    public void start() {
        super.start();
    }

    public void loop() {
        super.loop();
    }

    public void stop() {
        super.start();
    }
}
