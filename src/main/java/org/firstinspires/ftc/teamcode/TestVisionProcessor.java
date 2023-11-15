package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class TestVisionProcessor implements VisionProcessor {
    private final Telemetry telemetry;

    public TestVisionProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        telemetry.addData("Width: ", width);
        telemetry.addData("Height: ", height);
        telemetry.update();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double[] indexList = frame.get(0, 0);
        telemetry.addData("What the thing is", indexList[0] + " " + indexList[1] + " " + indexList[2]);
        telemetry.addData("Width: ", frame.width());
        telemetry.addData("Height", frame.height());
//        telemetry.update();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint test = new Paint();
        test.setColor(Color.RED);
        canvas.drawLine(100, 100, 100, 200, test);
        canvas.drawLine(100, 200, 200, 200, test);
        canvas.drawLine(200, 200, 200, 100, test);
        canvas.drawLine(200, 100, 100, 100, test); // a
    }
}
