package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoWriter;
import org.openftc.easyopencv.OpenCvPipeline;

public class VideoWriterPipeline extends OpenCvPipeline
{
	private final VideoWriter videoWriter;

	private static final int CAMERA_WIDTH = 640;
	private static final int CAMERA_HEIGHT = 480;
	private static final int CAMERA_FPS = 30;
	private static final int CAMERA_FOURCC = VideoWriter.fourcc('H', '2', '6', '4');

	private final String filename;

	public VideoWriterPipeline(String filename)
	{
		this.filename = filename;
		videoWriter = new VideoWriter();
	}

	private final Mat converted = new Mat();

	@Override
	public Mat processFrame(Mat input)
	{
		if (!videoWriter.isOpened())
			return input;

		converted.release();
		Imgproc.cvtColor(input, converted, Imgproc.COLOR_RGBA2RGB);
		videoWriter.write(converted);
		return input;
	}

	public void startRecording()
	{
		if (!videoWriter.open(filename, CAMERA_FOURCC, CAMERA_FPS, new Size(CAMERA_WIDTH, CAMERA_HEIGHT)))
			throw new RuntimeException("Failed to open video writer");
	}

	public void stopRecording()
	{
		videoWriter.release();
	}

	public boolean isRecording()
	{
		return videoWriter.isOpened();
	}
}