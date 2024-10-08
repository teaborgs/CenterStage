package org.firstinspires.ftc.teamcode.opencv;

import static org.firstinspires.ftc.teamcode.Constants.Detection.TeamProp.BLUE_LOWER;
import static org.firstinspires.ftc.teamcode.Constants.Detection.TeamProp.BLUE_UPPER;
import static org.firstinspires.ftc.teamcode.Constants.Detection.TeamProp.RED_LOWER;
import static org.firstinspires.ftc.teamcode.Constants.Detection.TeamProp.RED_UPPER;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TeamPropDetectionPipeline extends OpenCvPipeline
{
	private boolean debug;

	private final Mat hsvImage = new Mat();;
	private final Mat sphereMask = new Mat();
	private final Mat hierarchy = new Mat();
	private final List<Rect> boundingBoxes = new ArrayList<>();

	private Utilities.DetectionCase detectionCase = Utilities.DetectionCase.LEFT;
	private Utilities.Alliance currentAlliance;

	private boolean stopped;

	public TeamPropDetectionPipeline(Utilities.Alliance alliance, boolean debug) {
		this.currentAlliance = alliance;
		this.debug = debug;
	}

	public TeamPropDetectionPipeline(Utilities.Alliance alliance) { this(alliance, false); }

	public TeamPropDetectionPipeline(boolean debug) { this(null, debug); }

	public TeamPropDetectionPipeline() { this(null, false); }

	public void setAlliance(Utilities.Alliance alliance) { currentAlliance = alliance; }

	public void stopProcessing() { stopped = true; }

	@SuppressLint("DefaultLocale")
	@Override
	public Mat processFrame(Mat input)
	{
		if (stopped) return input;

		Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

		if(currentAlliance == Utilities.Alliance.RED) Core.inRange(hsvImage, Constants.Detection.TeamProp.RED_LOWER, Constants.Detection.TeamProp.RED_UPPER, sphereMask);
		else if(currentAlliance == Utilities.Alliance.BLUE) Core.inRange(hsvImage, Constants.Detection.TeamProp.BLUE_LOWER, Constants.Detection.TeamProp.BLUE_UPPER, sphereMask);
		else throw new IllegalStateException("Alliance not set");

		// Optional: Apply morphological operations to clean up the mask
		Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
		Imgproc.morphologyEx(sphereMask, sphereMask, Imgproc.MORPH_OPEN, kernel);
		Imgproc.morphologyEx(sphereMask, sphereMask, Imgproc.MORPH_CLOSE, kernel);

		// Find contours
		List<MatOfPoint> contours = new ArrayList<>();
		Imgproc.findContours(sphereMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		// Filter contours based on area (adjust as needed)
		boundingBoxes.clear();

		for (MatOfPoint contour : contours)
		{
			double contourArea = Imgproc.contourArea(contour);

			if (contourArea < Constants.Detection.TeamProp.PROP_SIZE) continue;

			// Get bounding box
			Rect boundingBox = Imgproc.boundingRect(contour);
			Imgproc.rectangle(input, boundingBox.tl(), boundingBox.br(), new Scalar(0, 255, 0), 1);
			Imgproc.putText(input, String.format("%.2f", contourArea), boundingBox.tl(), 0, 1, new Scalar(0, 255, 0));
			boundingBoxes.add(boundingBox);

			if(debug) Imgproc.putText(input, String.format("Size: %.2f", contourArea), boundingBox.tl(), 0, 1, new Scalar(0, 255, 0));
		}

		if (boundingBoxes.isEmpty())
			detectionCase = Utilities.DetectionCase.LEFT;
		else if (boundingBoxes.get(0).x + boundingBoxes.get(0).width / 2 < 300)
			detectionCase = Utilities.DetectionCase.CENTER;
		else
			detectionCase = Utilities.DetectionCase.RIGHT;

		hsvImage.release();
		sphereMask.release();
		hierarchy.release();

		return input;
	}

	public void setDebug(boolean debug) { this.debug = debug; }

	public Utilities.DetectionCase getDetectionCase() { return detectionCase; }
}