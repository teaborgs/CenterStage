package org.firstinspires.ftc.teamcode.opencv;

import static org.firstinspires.ftc.teamcode.Constants.Detection.MIN_AREA;
import static org.firstinspires.ftc.teamcode.Utilities.hsvToRgb;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.Constants;
import org.opencv.core.Core;
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

public class TeamPropDetectionPipeline extends OpenCvPipeline
{
	private boolean debug = false;

	Mat hsvImage = new Mat();
	Mat redMask = new Mat();
	Mat blueMask = new Mat();
	Mat sphereMask = new Mat();
	List<Rect> boundingBoxes = new ArrayList<>();

	@Override
	public Mat processFrame(Mat input)
	{
		hsvImage.release();
		redMask.release();
		blueMask.release();
		sphereMask.release();
		Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_BGR2HSV);

		// Define the range of red color in HSV
		Scalar lowerRed = new Scalar(0, 100, 100);
		Scalar upperRed = new Scalar(10, 255, 255);
		Core.inRange(hsvImage, lowerRed, upperRed, redMask);

		// Define the range of blue color in HSV
		Scalar lowerBlue = new Scalar(100, 100, 100);
		Scalar upperBlue = new Scalar(140, 255, 255);
		Core.inRange(hsvImage, lowerBlue, upperBlue, blueMask);

		// Combine the red and blue masks
		Core.addWeighted(redMask, 1.0, blueMask, 1.0, 0.0, sphereMask);

		// Optional: Apply morphological operations to clean up the mask
		Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
		Imgproc.morphologyEx(sphereMask, sphereMask, Imgproc.MORPH_OPEN, kernel);
		Imgproc.morphologyEx(sphereMask, sphereMask, Imgproc.MORPH_CLOSE, kernel);


		// Find contours
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(sphereMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		// Filter contours based on area (adjust as needed)
		double minContourArea = 100.0;
		boundingBoxes.clear();

		for (MatOfPoint contour : contours) {
			double contourArea = Imgproc.contourArea(contour);

			if (contourArea > minContourArea) {
				// Get bounding box
				Rect boundingBox = Imgproc.boundingRect(contour);
				Imgproc.rectangle(input, boundingBox.tl(), boundingBox.br(), new Scalar(0, 255, 0), 2);
				boundingBoxes.add(boundingBox);
			}
		}

		return input;
	}

	public void setDebug(boolean debug)
	{
		this.debug = debug;
	}
}