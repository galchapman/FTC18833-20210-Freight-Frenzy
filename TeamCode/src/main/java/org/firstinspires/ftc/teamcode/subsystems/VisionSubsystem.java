package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Bitmap;

import org.commandftc.RobotUniversal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.GameType;
import org.firstinspires.ftc.teamcode.lib.StartingPosition;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;
import static org.commandftc.RobotUniversal.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.FarBlueBRect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.FarBlueCRect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.FarRedARect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.FarRedBRect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.HueThresholdHigh;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.HueThresholdLow;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.NearBlueBRect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.NearBlueCRect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.NearRedARect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.NearRedBRect;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.SaturationThresholdHigh;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.SaturationThresholdLow;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.ValueThresholdHigh;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.ValueThresholdLow;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_height;
import static org.firstinspires.ftc.teamcode.Constants.VisionConstants.camera_width;

public class VisionSubsystem extends SubsystemBase {
    private final OpenCvCamera m_camera;
    private GameType gameType;
    private final Rect AView;
    private final Rect BView;

    private class PipeLine extends OpenCvPipeline {

        private final Bitmap bitmap = Bitmap.createBitmap(camera_width, camera_height, Bitmap.Config.RGB_565);

        @Override
        public Mat processFrame(Mat input) {

            Mat a_sub = input.submat(AView);
            Mat b_sub = input.submat(BView);

            Mat a_filtered = new Mat(), b_filtered = new Mat();
            filterGreen(a_sub, a_filtered);
            filterGreen(b_sub, b_filtered);

            telemetry.addData("A view", Core.countNonZero(a_filtered));
            telemetry.addData("B view", Core.countNonZero(b_filtered));
            telemetry.update();
            if (Core.countNonZero(b_filtered) > 5000) {
                gameType = GameType.B;
            } else if (Core.countNonZero(a_filtered) > 5000 ^ (RobotUniversal.startingPosition == StartingPosition.FarBlue || RobotUniversal.startingPosition == StartingPosition.NearBlue)) {
                gameType = GameType.A;
            } else {
                gameType = GameType.C;
            }

            Imgproc.rectangle(input, AView, new Scalar(255, 255, 0), 3);
            Imgproc.rectangle(input, BView, new Scalar(255, 0, 255), 3);

//            a_filtered.release();
//            b_filtered.release();

            return input;
        }

        private void filterGreen(Mat input,
                                  Mat out) {
            Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
            Core.inRange(out, new Scalar(HueThresholdLow, SaturationThresholdLow, ValueThresholdLow),
                    new Scalar(HueThresholdHigh, SaturationThresholdHigh, ValueThresholdHigh), out);
        }

        /**
         * Filter out an area of an image using a binary mask.
         * @param input The image on which the mask filters.
         * @param mask The binary image that is used to filter.
         * @param output The image in which to store the output.
         */
        private void mask(Mat input, Mat mask, Mat output) {
            mask.convertTo(mask, CvType.CV_8UC1);
            Core.bitwise_xor(output, output, output);
            input.copyTo(output, mask);
        }
    }

    public VisionSubsystem() {
        m_camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));
        m_camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                m_camera.startStreaming(camera_width, camera_height, OpenCvCameraRotation.UPRIGHT);
                m_camera.setPipeline(new PipeLine());
            }
            @Override
            public void onError(int errorCode) {}
        });

        Rect AView, BView;

        switch (RobotUniversal.startingPosition) {
            default:
            case FarRed:
                AView = FarRedARect;
                BView = FarRedBRect;
                break;
            case NearRed:
                AView = NearRedARect;
                BView = NearRedBRect;
                break;
            case FarBlue:
                AView = FarBlueCRect;
                BView = FarBlueBRect;
                break;
            case NearBlue:
                AView = NearBlueCRect;
                BView = NearBlueBRect;
                break;
        }

        this.AView = AView;
        this.BView = BView;
    }

    public GameType getGameType() {
        return gameType;
    }

    public void stop() {
        m_camera.closeCameraDevice();
    }
}
