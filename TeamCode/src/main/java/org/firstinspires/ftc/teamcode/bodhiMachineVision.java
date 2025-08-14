/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

// This version of the internal camera example uses EasyOpenCV's interface to the original
// Android camera API

@TeleOp(name = "Bodhi Machine Vision", group = "Robot")

public class bodhiMachineVision extends LinearOpMode {
    OpenCvCamera webCam;
    SampleAlignmentPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new SampleAlignmentPipeline();

        // Specify the image processing pipeline we wish to invoke upon receipt of a frame from the camera.
        // Note that switching pipelines on-the-fly (while a streaming session is in flight) *IS* supported.
        webCam.setPipeline(pipeline);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the user to press start on the Driver Station
        waitForStart();

        while (opModeIsActive()) {
            // Send some stats to the telemetry
            telemetry.addData("Frame Count", webCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webCam.getFps()));
            telemetry.addData("Total frame time ms", webCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webCam.getCurrentPipelineMaxFps());
            telemetry.addData("Angle", pipeline.getAnalysis());
            telemetry.update();

            if (gamepad1.a) {
                // IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                // from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                // if the reason you wish to stop the stream early is to switch use of the camera
                // over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                // (commented out below), because according to the Android Camera API documentation:
                //	     "Your application should only have one Camera object active at a time for
                //	      a particular hardware camera."
                //
                // NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                // but it doesn't hurt to call it anyway, if for no other reason than clarity.
                //
                // NB2: if you are stopping the camera stream to simply save some processing power
                // (or battery power) for a short while when you do not need your vision pipeline,
                // it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                // it the next time you wish to activate your vision pipeline, which can take a bit of
                // time. Of course, this comment is irrelevant in light of the use case described in
                // the above "important note".
                webCam.stopStreaming();
                //phoneCam.closeCameraDevice();
            }

            sleep(100);
        }
    }

    static class SampleAlignmentPipeline extends OpenCvPipeline {
        // An enum to define the skystone position

        Mat colorR0 = new Mat();
        Mat colorR1 = new Mat();
        Mat colorR = new Mat();
        Mat colorG = new Mat();
        Mat colorB = new Mat();
        Mat dst = new Mat();
        Mat cdst = new Mat();
        Mat hsv = new Mat();
        Mat result = new Mat();

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile double angles;

        @Override
        public void init(Mat firstFrame) {
            // We need to call this in order to make sure the 'Cb' object is initialized, so that the
            // submats we make will still be linked to it on subsequent frames. (If the object were to
            // only be initialized in processFrame, then the submats would become delinked because the
            // backing buffer would be re-allocated the first time a real frame was crunched)
            //inputToCb(firstFrame);

            // Submats are a persistent reference to a region of the parent buffer. Any changes to the
            // child affect the parent, and the reverse also holds true.
            /*
            region1_Cb = colorR.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = colorR.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = colorR.submat(new Rect(region3_pointA, region3_pointB));*/
        }

        public boolean areClose(double i1, double i2, double range) {
            return Math.abs(i1 - i2) <= range;
        }

        public double houghPolar(Mat input, Scalar color) {
            // Edge detection
            Imgproc.Canny(input, dst, 50, 200, 3, false);

            // Copy edges to the images that will display the results in BGR
            Imgproc.cvtColor(dst, cdst, Imgproc.COLOR_GRAY2BGR);
            // Standard Hough Line Transform
            Mat lines = new Mat(); // will hold the results of the detection
            Imgproc.HoughLines(dst, lines, 1, Math.PI / 180, 60); // runs the actual detection
            // Draw the lines

            ArrayList<Double> angles = new ArrayList<Double>();

            for (int x = 0; x < Math.min(lines.rows(), 2); x++) {
                double rho = lines.get(x, 0)[0],
                        theta = lines.get(x, 0)[1];
                double a = Math.cos(theta), b = Math.sin(theta);
                double x0 = a * rho, y0 = b * rho;
                Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
                Point pt2 = new Point(Math.round(x0 - 1000 * (-b)), Math.round(y0 - 1000 * (a)));

                Imgproc.line(result, pt1, pt2, color, 3, Imgproc.LINE_AA, 0);

                double dy = pt1.y - pt2.y;
                double dx = pt1.x - pt2.x;

                double rawAngle = Math.atan2(dy, dx) * (180 / Math.PI);
                double angle = 180 - Math.abs(rawAngle);
                angles.add(angle * (rawAngle < 0 ? 1 : -1));
                //angles.add(angle < 0 ? 180 - Math.abs(angle) : angle);
            }

            double fAngle = 0;
            double mainAngle = angles.isEmpty() ? 0 : angles.get(0);
            double sumAngles = mainAngle;
            double validAngles = angles.isEmpty() ? 0 : 1;

            int[] angleIncrements = {0, -90, 90};

            for (int i = 1; i < angles.size(); i++) {
                double cAngle = angles.get(i);

                for (int j = 0; j < angleIncrements.length; j++) {
                    if (areClose(mainAngle, cAngle + angleIncrements[j], 1)) {
                        sumAngles += cAngle + angleIncrements[j];
                        validAngles += 1;
                        break;
                    }
                }
            }

            fAngle = angles.isEmpty() ? 0 : sumAngles / validAngles;
            return fAngle;
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV, 4);
            
            //red
            Core.inRange(hsv, new Scalar(160, 15, 170), new Scalar(180, 255, 255), colorR0);
            Core.inRange(hsv, new Scalar(0, 15, 170), new Scalar(19, 255, 255), colorR1);
            Core.add(colorR0, colorR1, colorR);

            //yellow
            Core.inRange(hsv, new Scalar(20, 80, 170), new Scalar(40, 255, 255), colorG);

            //blue
            Core.inRange(hsv, new Scalar(90, 25, 100), new Scalar(140, 255, 255), colorB);

            //result = input;

            //double angleR = houghPolar(colorR, new Scalar(255, 0, 0));
            //double angleG = houghPolar(colorG, new Scalar(255, 255, 0));
            //double angleB = houghPolar(colorB, new Scalar(0, 0, 255));

            //angles = angleR;

            List<Mat> listMat = Arrays.asList(colorR, colorG, colorB);
            Core.merge(listMat, result);
            //result = colorR;
            return result;
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public double getAnalysis() {
            return angles;
        }
    }
}
