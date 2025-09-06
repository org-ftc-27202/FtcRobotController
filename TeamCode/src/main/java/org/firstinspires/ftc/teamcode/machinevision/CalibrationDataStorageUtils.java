package org.firstinspires.ftc.teamcode.machinevision;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import android.content.Context;

import java.util.Arrays;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import androidx.annotation.NonNull;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;

public class CalibrationDataStorageUtils {
	private CalibrationDataStorageUtils() {

	}

	public static class HsvChannelBounds {
		private Scalar lowerBounds;
		private Scalar upperBounds;

		public HsvChannelBounds(double[] initLowerBounds, double[] initUpperBounds) {
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

		public DateMs(long init_ms) {
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

	public static class CalibrationDataStorage {
		private final Context appContext;

		private final File internalFilesDir;
		public CalibrationDataStorage(Context context) {
			if (context == null) {
				throw new IllegalArgumentException("Context cannot be null");
			}
			appContext = context.getApplicationContext();
			internalFilesDir = appContext.getFilesDir();
		}

		public String getInternalDir() {
			return internalFilesDir.getAbsolutePath();
		}

		@NonNull
		private String getCalibrationJson(HsvChannelBounds rBounds, HsvChannelBounds yBounds, HsvChannelBounds bBounds, DateMs date_ms) {
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

		public void writeToInternalStorage(String filename, HsvChannelBounds rBounds, HsvChannelBounds yBounds, HsvChannelBounds bBounds, DateMs date_ms) {
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

		public boolean readAndParseCalibrationData(String filename, HsvChannelBounds rBounds, HsvChannelBounds yBounds, HsvChannelBounds bBounds, DateMs date_ms) {
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
						// Return Did Fetch Calibration Data
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
			//USING 1s FOR DEBUGGING
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
			// Return Didn't Fetch Calibration Data
			return false;
		}
	}
}
