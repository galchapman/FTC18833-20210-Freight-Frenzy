package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.HueThresholdHigh;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.HueThresholdLow;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.SaturationThresholdHigh;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.SaturationThresholdLow;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.ValueThresholdHigh;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.ValueThresholdLow;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;

//@Disabled
@TeleOp(group = "tests")
public class TestView extends OpMode {
    private OpenCvCamera camera;
    private final Bitmap bitmap = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);

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

                                Mat filtered = new Mat();
                                filterGreen(input, filtered);

                                Mat masked = new Mat();
                                mask(input, filtered, masked);
                                filtered.release();

                                input.release();
                                input = masked;

                                Imgproc.rectangle(input, Constants.VisionConstants.FarRedARect, new Scalar(255, 0, 0), 4);
                                Imgproc.rectangle(input, Constants.VisionConstants.FarRedBRect, new Scalar(255, 255, 0), 4);

                                Utils.matToBitmap(input, bitmap);
                                FtcDashboard.getInstance().sendImage(bitmap);
                                input.release();
                                return input;
                            }

                            private void filterGreen(Mat input,
                                                     Mat out) {
                                Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
                                Core.inRange(out, new Scalar(HueThresholdLow, SaturationThresholdLow, ValueThresholdLow),
                                        new Scalar(HueThresholdHigh, SaturationThresholdHigh, ValueThresholdHigh), out);
                            }

                            private void mask(Mat input, Mat mask, Mat output) {
                                mask.convertTo(mask, CvType.CV_8UC1);
                                Core.bitwise_xor(output, output, output);
                                input.copyTo(output, mask);
                            }
                        });
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                }
        );
    }

    @Override
    public void loop() {
        telemetry.update();
    }
}
