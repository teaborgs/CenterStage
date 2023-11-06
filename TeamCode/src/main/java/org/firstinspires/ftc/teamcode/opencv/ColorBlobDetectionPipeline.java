package org.firstinspires.ftc.teamcode.opencv;

import static org.firstinspires.ftc.teamcode.Constants.Detection.*;

import android.annotation.SuppressLint;

import androidx.appcompat.widget.AppCompatImageHelper;

import org.firstinspires.ftc.teamcode.testing.ColorBlobDetectionTest;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorBlobDetectionPipeline extends OpenCvPipeline
{
	private boolean debug = false;

	private final Mat hsv = new Mat();
	private final Mat mask = new Mat();
	private final Mat hierarchy = new Mat();
	private final List<MatOfPoint> contours = new ArrayList<>();
	private final MatOfPoint2f contour2f = new MatOfPoint2f();
	private final MatOfPoint2f approx = new MatOfPoint2f();

	private final List<MatOfPoint> detections = new ArrayList<>(); // TODO: Implement actual detection class with functionality

	@Override
	public Mat processFrame(Mat input)
	{
		detections.clear();

		Imgproc.resize(input, input, new Size(160 * 5, 90 * 5));


		Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
		for (int i = 0; i < LOWER_BOUNDS.size(); i++)
			detectAndDrawBlobs(hsv, input, LOWER_BOUNDS.get(i), UPPER_BOUNDS.get(i), i);
		return input;
	}

	private void detectionTest(Mat hsv, Mat output, Scalar lower, Scalar upper, int index)
	{
		contours.clear();
		Core.inRange(hsv, lower, upper, mask);
		Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		Imgproc.cvtColor(mask, output, 0);
	}

	@SuppressLint("DefaultLocale")
	private void detectAndDrawBlobs(Mat hsv, Mat output, Scalar lower, Scalar upper, int index)
	{
		contours.clear();
		Core.inRange(hsv, lower, upper, mask);
		Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		for (MatOfPoint contour : contours) {
			double contourSize = Imgproc.contourArea(contour);
			if (contourSize > MIN_AREA) { // Filter by area
				// Approximate contour shape and check for hexagon-like shape
				contour2f.fromArray(contour.toArray());
				Imgproc.approxPolyDP(contour2f, approx, 0.04 * contour2f.rows(), true);
				long vertices = approx.total();
				if (true) {  // Check for hexagon shape
					detections.add(contour);

					Rect rect = Imgproc.boundingRect(contour);
					Scalar meanHSV = Core.mean(hsv.submat(rect), mask.submat(rect));
					Scalar color = hsvToRgb(meanHSV.val[0], meanHSV.val[1], meanHSV.val[2]);

					// Draw rounded bounding box
					Imgproc.rectangle(output, rect.tl(), rect.br(), color, 1);

					if (debug) {
						// Draw label
						Imgproc.putText(output, "Pixel", new Point(rect.x + rect.width + 5, rect.y + rect.height), Imgproc.FONT_HERSHEY_SIMPLEX, 0.2, color, 1);

						// Draw HSV values
						String hsvText = String.format("H:%.0f, S:%.0f, V:%.0f", meanHSV.val[0], meanHSV.val[1], meanHSV.val[2]);
						Imgproc.putText(output, hsvText, new Point(rect.x + rect.width + 5, rect.y + rect.height - 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.2, color, 1);

						// Draw the number of vertices
						String verticesText = String.format("Vertices: %d", approx.total());
						Imgproc.putText(output, verticesText, new Point(rect.x + rect.width + 5, rect.y + rect.height - 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.2, color, 1);

						// Draw detection color
						String colorIndex = "NULL";
						if (index == 0) colorIndex = "White";
						if (index == 1) colorIndex = "Green";
						if (index == 2) colorIndex = "Purple";
						if (index == 3) colorIndex = "Yellow";
						Imgproc.putText(output, colorIndex, new Point(rect.x + rect.width + 5, rect.y + rect.height - 90), Imgproc.FONT_HERSHEY_SIMPLEX, 0.2, color, 1);
					}
				}
			}
		}
	}

	private Scalar hsvToRgb(double h, double s, double v)
	{
		// Normalize to 0..1
		double hPrime = h / 179.0 * 360.0;
		double sPrime = s / 255.0;
		double vPrime = v / 255.0;

		double c = vPrime * sPrime;
		double hPrimeDivided = hPrime / 60.0;
		double x = c * (1 - Math.abs(hPrimeDivided % 2 - 1));

		double rPrime = 0;
		double gPrime = 0;
		double bPrime = 0;

		if (0 <= hPrimeDivided && hPrimeDivided < 1) {
			rPrime = c;
			gPrime = x;
		} else if (1 <= hPrimeDivided && hPrimeDivided < 2) {
			rPrime = x;
			gPrime = c;
		} else if (2 <= hPrimeDivided && hPrimeDivided < 3) {
			gPrime = c;
			bPrime = x;
		} else if (3 <= hPrimeDivided && hPrimeDivided < 4) {
			gPrime = x;
			bPrime = c;
		} else if (4 <= hPrimeDivided && hPrimeDivided < 5) {
			rPrime = x;
			bPrime = c;
		} else if (5 <= hPrimeDivided && hPrimeDivided < 6) {
			rPrime = c;
			bPrime = x;
		}

		double m = vPrime - c;

		int r = (int) ((rPrime + m) * 255.0);
		int g = (int) ((gPrime + m) * 255.0);
		int b = (int) ((bPrime + m) * 255.0);

		return new Scalar(r, g, b);
	}

	public void setDebug(boolean debug)
	{
		this.debug = debug;
	}

	public List<MatOfPoint> getLatestDetections()
	{
		return detections;
	}
}