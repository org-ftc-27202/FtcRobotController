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

package org.firstinspires.ftc.teamcode.machinevision;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import android.content.Context;

import java.util.ArrayList;
import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.StreamFormat;
import org.firstinspires.ftc.vision.VisionProcessor;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

@TeleOp(name = "Vision Calibration", group = "Robot")

public class visionCalibration extends LinearOpMode {
    //HsvBounds redBounds = new HsvBounds(new double[]{160, 15, 179}, new double[]{9, 255, 255});
    //HsvBounds yellowBounds = new HsvBounds(new double[]{20, 80, 170}, new double[]{40, 255, 255});
    //HsvBounds blueBounds = new HsvBounds(new double[]{90, 25, 100}, new double[]{140, 255, 255});

    HsvBounds rBounds = new HsvBounds(new double[]{0, 0, 0}, new double[]{0, 0, 0});
    HsvBounds yBounds = new HsvBounds(new double[]{0, 0, 0}, new double[]{0, 0, 0});
    HsvBounds bBounds = new HsvBounds(new double[]{0, 0, 0}, new double[]{0, 0, 0});
    DateMs calibrationDate = new DateMs(0);

    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        SampleAlignmentPipeline sampleAlignmentPipeline = new SampleAlignmentPipeline();

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(StreamFormat.MJPEG)
                .addProcessor(sampleAlignmentPipeline);

        VisionPortal visionPortal = builder.build();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the user to press start on the Driver Station
        waitForStart();

        CalibrationDataAccessPoint calibrationData = new CalibrationDataAccessPoint();
        //calibrationData.writeToInternalStorage("calibration_data.json", redBounds, yellowBounds, blueBounds, new DateMs(System.currentTimeMillis()));


        boolean receivedCalibrationFromInternalStorage = calibrationData.readAndParseCalibrationData("calibration_data.json", rBounds, yBounds, bBounds, calibrationDate);

        while (opModeIsActive()) {
            // Send some stats to the telemetry
            //ProcessorTelemetry telemetry = visionPortal.getProcessorTelemetry(myTfodProcessor);
            //telemetry.addData("Frame Count", visionPortal.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", visionPortal.getFps()));

            //telemetry.addData("File Path", calibrationData.getInternalDir());
            //telemetry.addData("File Contents", calibrationData.readFromJsonInternalStorage("calibration_data.json"));

            telemetry.addData("Received Calibration From Internal Storage", receivedCalibrationFromInternalStorage);

            double[] r_lower = rBounds.getLowerBoundsArray();
            double[] r_upper = rBounds.getUpperBoundsArray();
            double[] y_lower = yBounds.getLowerBoundsArray();
            double[] y_upper = yBounds.getUpperBoundsArray();
            double[] b_lower = bBounds.getLowerBoundsArray();
            double[] b_upper = bBounds.getUpperBoundsArray();

            telemetry.addLine("Red Lower Bounds: [" + r_lower[0] + ", " + r_lower[1] + ", " + r_lower[2] + "]");
            telemetry.addLine("Red Upper Bounds: [" + r_upper[0] + ", " + r_upper[1] + ", " + r_upper[2] + "]");
            telemetry.addLine("Yellow Lower Bounds: [" + y_lower[0] + ", " + y_lower[1] + ", " + y_lower[2] + "]");
            telemetry.addLine("Yellow Upper Bounds: [" + y_upper[0] + ", " + y_upper[1] + ", " + y_upper[2] + "]");
            telemetry.addLine("Blue Lower Bounds: [" + b_lower[0] + ", " + b_lower[1] + ", " + b_lower[2] + "]");
            telemetry.addLine("Blue Upper Bounds: [" + b_upper[0] + ", " + b_upper[1] + ", " + b_upper[2] + "]");

            telemetry.addData("Minutes Since Calibration", calibrationDate.getMinutesTimeSince());

            //telemetry.addData("Total frame time ms", visionPortal.getTotalFrameTimeMs());
            //telemetry.addData("Pipeline time ms", visionPortal.getPipelineTime());
            //telemetry.addData("Overhead time ms", visionPortal.getOverheadTime());
            //telemetry.addData("Theoretical max FPS", visionPortal.getCurrentPipelineMaxFps());
            //telemetry.addData("Angle", myPipeline.getAnalysis());
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
                visionPortal.close(); //closes communication with USB permanently
                //visionPortal.stopStreaming(); //can be re-enabled
                //phoneCam.closeCameraDevice();
            }

            sleep(100);
        }
    }

    public static class HsvBounds {
        private Scalar lowerBounds;
        private Scalar upperBounds;

        private HsvBounds(double[] initLowerBounds, double[] initUpperBounds) {
            if (initLowerBounds == null || initUpperBounds == null) {
                throw new IllegalArgumentException("lowerBounds and upperBounds cannot be null");
            }

            if (initLowerBounds.length != 3 || initUpperBounds.length != 3) {
                throw new IllegalArgumentException("lowerBounds and upperBounds must have 3 elements");
            }

            this.lowerBounds = new Scalar(initLowerBounds[0], initLowerBounds[1], initLowerBounds[2]);
            this.upperBounds = new Scalar(initUpperBounds[0], initUpperBounds[1], initUpperBounds[2]);
        }

        public Scalar getLowerBounds() {
            return new Scalar(lowerBounds.val[0], lowerBounds.val[1], lowerBounds.val[2]);
        }

        public double[] getLowerBoundsArray() {
            return Arrays.copyOf(new double[] {lowerBounds.val[0], lowerBounds.val[1], lowerBounds.val[2]}, 3);
        }

        public Scalar getUpperBounds() {
            return new Scalar(upperBounds.val[0], upperBounds.val[1], upperBounds.val[2]);
        }

        public double[] getUpperBoundsArray() {
            return Arrays.copyOf(new double[] {upperBounds.val[0], upperBounds.val[1], upperBounds.val[2]}, 3);
        }

        public void setLowerBounds(double[] newLowerBounds) {
            if (newLowerBounds == null) {
                throw new IllegalArgumentException("lowerBounds cannot be null");
            }

            if (newLowerBounds.length != 3) {
                throw new IllegalArgumentException("lowerBounds must have 3 elements");
            }

            lowerBounds = new Scalar(newLowerBounds[0], newLowerBounds[1], newLowerBounds[2]);
        }

        public void setUpperBounds(double[] newUpperBounds) {
            if (newUpperBounds == null) {
                throw new IllegalArgumentException("upperBounds cannot be null");
            }

            if (newUpperBounds.length != 3) {
                throw new IllegalArgumentException("upperBounds must have 3 elements");
            }

            upperBounds = new Scalar(newUpperBounds[0], newUpperBounds[1], newUpperBounds[2]);
        }

        public void setBounds(double[] newLowerBounds, double[] newUpperBounds) {
            setLowerBounds(newLowerBounds);
            setUpperBounds(newUpperBounds);
        }

        public void hsvMatInRange(Mat src, Mat output) {
            double lowerBoundsH = lowerBounds.val[0];
            double upperBoundsH = upperBounds.val[0];

            if (lowerBoundsH > upperBoundsH) {
                // Handling HSV wrap-around for hues (ex. red)
                Mat withinBounds0 = new Mat();
                Mat withinBounds1 = new Mat();

                // lowerBoundsH to Max Hue
                Core.inRange(src,
                        new Scalar(lowerBoundsH, lowerBounds.val[1], lowerBounds.val[2]),
                        new Scalar(179, upperBounds.val[1], upperBounds.val[2]),
                        withinBounds0);

                // Min Hue to upperBoundsH
                Core.inRange(src,
                        new Scalar(0, lowerBounds.val[1], lowerBounds.val[2]),
                        new Scalar(upperBoundsH, upperBounds.val[1], upperBounds.val[2]),
                        withinBounds1);

                // Combine the 2 masks
                Core.add(withinBounds0, withinBounds1, output);

                //Releasing temporary Mat objects from memory
                withinBounds0.release();
                withinBounds1.release();
            } else {
                // Standard HSV range
                Core.inRange(src, lowerBounds, upperBounds, output);
            }
        }
    }

    public static class DateMs {
        private long date_ms;

        private DateMs(long init_ms) {
            date_ms = init_ms;
        }

        public long getDateMs() {
            return date_ms;
        }

        public void setDateMs(long new_ms) {
            date_ms = new_ms;
        }

        public double getMinutesTimeSince() {
            long differenceMillis = System.currentTimeMillis() - date_ms;
            return (double) differenceMillis / 1000 / 60;
        }
    }

    public class CalibrationDataAccessPoint {
        private final Context appContext;

        private final File internalFilesDir;
        private CalibrationDataAccessPoint() {
            appContext = hardwareMap.appContext;
            internalFilesDir = appContext.getFilesDir();
        }

        public String getInternalDir() {
            return internalFilesDir.getAbsolutePath();
        }

        private String getCalibrationJson(HsvBounds rBounds, HsvBounds yBounds, HsvBounds bBounds, DateMs date_ms) {
            double[] r_lower = rBounds.getLowerBoundsArray();
            double[] r_upper = rBounds.getUpperBoundsArray();
            double[] y_lower = yBounds.getLowerBoundsArray();
            double[] y_upper = yBounds.getUpperBoundsArray();
            double[] b_lower = bBounds.getLowerBoundsArray();
            double[] b_upper = bBounds.getUpperBoundsArray();

            return
                "{\n" +
                "   \"calibration_values\": {\n" +
                "       \"red\": {\n" +
                "           \"lower_bounds\": [" + r_lower[0] + ", " + r_lower[1] + ", " + r_lower[2] + "],\n" +
                "           \"upper_bounds\": [" + r_upper[0] + ", " + r_upper[1] + ", " + r_upper[2] + "]\n" +
                "       },\n" +
                "       \"yellow\": {\n" +
                "           \"lower_bounds\": [" + y_lower[0] + ", " + y_lower[1] + ", " + y_lower[2] + "],\n" +
                "           \"upper_bounds\": [" + y_upper[0] + ", " + y_upper[1] + ", " + y_upper[2] + "]\n" +
                "       },\n" +
                "       \"blue\": {\n" +
                "           \"lower_bounds\": [" + b_lower[0] + ", " + b_lower[1] + ", " + b_lower[2] + "],\n" +
                "           \"upper_bounds\": [" + b_upper[0] + ", " + b_upper[1] + ", " + b_upper[2] + "]\n" +
                "       }\n" +
                "   },\n" +
                "   \"calibration_date_ms\": " + date_ms.getDateMs() + "\n" +
                "}";
        }

        public void writeToInternalStorage(String filename, HsvBounds rBounds, HsvBounds yBounds, HsvBounds bBounds, DateMs date_ms) {
            String jsonStringContent = getCalibrationJson(rBounds, yBounds, bBounds, date_ms);
            FileOutputStream fos = null;

            try {
                fos = appContext.openFileOutput(filename, Context.MODE_PRIVATE);
                // Context.MODE_PRIVATE: Default mode. If the file already exists, it will be overwritten.
                // Context.MODE_APPEND: If the file already exists, data will be appended to the end.

                fos.write(jsonStringContent.getBytes(StandardCharsets.UTF_8));
                // Telemetry or Log.d to confirm save
                // telemetry.addData("Internal Storage", "Saved to " + FILENAME);
            } catch (IOException e) {
                e.printStackTrace();
                // telemetry.addData("Internal Storage", "Error saving: " + e.getMessage());
            } finally {
                if (fos != null) {
                    try {
                        fos.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
        }

        public String readFromJsonInternalStorage(String filename) {
            StringBuilder stringBuilder = new StringBuilder();
            FileInputStream fis = null;
            String fileContent;

            try {
                fis = appContext.openFileInput(filename);
                InputStreamReader inputStreamReader = new InputStreamReader(fis, StandardCharsets.UTF_8);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

                String line;
                while ((line = bufferedReader.readLine()) != null) {
                    stringBuilder.append(line);
                }
                fileContent = stringBuilder.toString();
                // telemetry.addData("Internal Storage", "Content: " + fileContent);

            } catch (FileNotFoundException e) {
                e.printStackTrace();
                return "File not found";
            } catch (IOException e) {
                e.printStackTrace();
                return "Error reading file";
            } finally {
                if (fis != null) {
                    try {
                        fis.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
            return fileContent;
            //todo: convert json to values and add backup values in case of failure
        }

        public double[] jsonArrayToDoubleArray(JSONArray array) {
            double[] doubleArray = new double[array.length()];
            try {
                for (int i = 0; i < array.length(); i++) {
                    doubleArray[i] = array.getDouble(i);
                }
                return doubleArray;
            } catch (JSONException e) {
                e.printStackTrace();
            }
            // Returning an empty array doesn't matter because it will catch error in other function anyways
            return doubleArray;
        }

        public boolean readAndParseCalibrationData(String filename, HsvBounds rBounds, HsvBounds yBounds, HsvBounds bBounds, DateMs date_ms) {
            try {
                String jsonStringContent = readFromJsonInternalStorage(filename);
                if (!jsonStringContent.isEmpty()) {
                    try {
                        JSONObject jsonObject = new JSONObject(jsonStringContent);

                        JSONObject calibrationValues = jsonObject.getJSONObject("calibration_values");
                        JSONObject red_bounds = calibrationValues.getJSONObject("red");
                        JSONObject yellow_bounds = calibrationValues.getJSONObject("yellow");
                        JSONObject blue_bounds = calibrationValues.getJSONObject("blue");

                        rBounds.setBounds(
                                jsonArrayToDoubleArray(red_bounds.getJSONArray("lower_bounds")),
                                jsonArrayToDoubleArray(red_bounds.getJSONArray("upper_bounds"))
                        );
                        yBounds.setBounds(
                                jsonArrayToDoubleArray(yellow_bounds.getJSONArray("lower_bounds")),
                                jsonArrayToDoubleArray(yellow_bounds.getJSONArray("upper_bounds"))
                        );
                        bBounds.setBounds(
                                jsonArrayToDoubleArray(blue_bounds.getJSONArray("lower_bounds")),
                                jsonArrayToDoubleArray(blue_bounds.getJSONArray("upper_bounds"))
                        );

                        date_ms.setDateMs(jsonObject.getLong("calibration_date_ms"));
                        return true;
                    } catch (JSONException e) {
                        e.printStackTrace();
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }

            // SOMETHING WENT WRONG, USE DEFAULT VALUES
            /*
            rBounds.setBounds(
                    new double[]{160, 15, 179},
                    new double[]{9, 255, 255}
            );
            yBounds.setBounds(
                    new double[]{20, 80, 170},
                    new double[]{40, 255, 255}
            );
            bBounds.setBounds(
                    new double[]{90, 25, 100},
                    new double[]{140, 255, 255}
            );
            date_ms.setDateMs(System.currentTimeMillis());
            // todo: change the date_ms to an accurate date
            */
            //USING 0 FOR DEBUGGING
            rBounds.setBounds(
                    new double[]{1, 1, 1},
                    new double[]{1, 1, 1}
            );
            yBounds.setBounds(
                    new double[]{1, 1, 1},
                    new double[]{1, 1, 1}
            );
            bBounds.setBounds(
                    new double[]{1, 1, 1},
                    new double[]{1, 1, 1}
            );
            return false;
        }
    }

    public class SampleAlignmentPipeline implements VisionProcessor {
        //HsvBounds redBounds = new HsvBounds(new double[]{160, 15, 179}, new double[]{9, 255, 255});
        //HsvBounds yellowBounds = new HsvBounds(new double[]{20, 80, 170}, new double[]{40, 255, 255});
        //HsvBounds blueBounds = new HsvBounds(new double[]{90, 25, 100}, new double[]{140, 255, 255});

        Mat dst = new Mat();
        Mat cdst = new Mat();
        Mat hsv = new Mat();
        Mat processedFrame = new Mat();

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile double angles;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
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
                double rho = lines.get(x, 0)[0], theta = lines.get(x, 0)[1];
                double a = Math.cos(theta), b = Math.sin(theta);
                double x0 = a * rho, y0 = b * rho;
                Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
                Point pt2 = new Point(Math.round(x0 - 1000 * (-b)), Math.round(y0 - 1000 * (a)));

                Imgproc.line(processedFrame, pt1, pt2, color, 3, Imgproc.LINE_AA, 0);

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
        public Mat processFrame(Mat frame, long captureTimeNanos) {
            Mat flippedFrame = new Mat();

            // Flip camera feed right side-up
            Core.flip(frame, flippedFrame, -1);

            // Convert to HSV
            Imgproc.cvtColor(flippedFrame, hsv, Imgproc.COLOR_RGB2HSV, 4);

            Mat redMask = new Mat();
            Mat yellowMask = new Mat();
            Mat blueMask = new Mat();

            // Apply masks to frame
            rBounds.hsvMatInRange(hsv, redMask);
            yBounds.hsvMatInRange(hsv, yellowMask);
            bBounds.hsvMatInRange(hsv, blueMask);

            // Combine masks
            Core.merge(Arrays.asList(redMask, yellowMask, blueMask), processedFrame);

            // Release temporary Mat objects from memory
            redMask.release();
            yellowMask.release();
            blueMask.release();

            return processedFrame;
        }

        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            Mat processedMat = (Mat) userContext;

            if (processedMat != null && !processedMat.empty()) {

                // Step 1: Handle Grayscale Images
                // onDrawFrame() requires a 3-channel (RGB) Mat to convert to a Bitmap.
                // If your processed Mat is grayscale (CV_8UC1), convert it to 3-channel BGR.
                //if (processedMat.channels() == 1) {
                //    Imgproc.cvtColor(processedMat, processedMat, Imgproc.COLOR_GRAY2BGR);
                //}

                // Step 2: Convert the Mat to a Bitmap.
                // Use the ARGB_8888 format for maximum compatibility.
                Bitmap processedBitmap = Bitmap.createBitmap(processedMat.cols(), processedMat.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(processedMat, processedBitmap);

                // Step 3: Draw the Bitmap to the canvas.
                // The canvas expects to be a 3-channel Mat (RGB)
                canvas.drawBitmap(processedBitmap, 0, 0, null);
            }
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public double getAnalysis() {
            return angles;
        }
    }
}
