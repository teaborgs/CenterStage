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
		hsv.release();
		hierarchy.release();
		mask.release();
		detections.clear();
		Imgproc.resize(input, input, new Size(160 * 5, 90 * 5));
//		Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//		Scalar lower = new Scalar(300, 40, 40);
//		Scalar upper = new Scalar(360, 255, 255);
//		Core.inRange(hsv, lower, upper, mask);
//		Core.bitwise_and(input, input, hierarchy, mask);

		Scalar lower = new Scalar(50, 0, 0);
		Scalar upper = new Scalar(255, 100, 100);

		Core.inRange(input, lower, upper, mask);
		Core.bitwise_and(input, input, hierarchy, mask);

		return mask;
	}

	private void filterMatrix(Mat input)
	{

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