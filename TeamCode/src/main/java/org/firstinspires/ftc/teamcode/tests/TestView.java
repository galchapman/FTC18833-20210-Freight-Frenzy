package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;

@Disabled
@TeleOp
public class TestView extends OpMode {
    private OpenCvCamera camera;
    private final Bitmap bitmap = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);
    private DistanceSensor distanceSensor;

    @Override
    public void init() {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        camera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(camera_width, camera_height, OpenCvCameraRotation.UPRIGHT);
                        camera.setPipeline(new OpenCvPipeline() {
                            @Override
                            public Mat processFrame(Mat input) {
                                Utils.matToBitmap(input, bitmap);
                                FtcDashboard.getInstance().sendImage(bitmap);
                                input.release();
                                return input;
                            }
                        });
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                }
        );

        distanceSensor = hardwareMap.get(DistanceSensor.class, "TestSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.METER));
        telemetry.update();
    }
}
