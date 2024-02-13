package org.firstinspires.ftc.teamcode.testing;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.InputSystem;
import org.firstinspires.ftc.teamcode.opencv.VideoWriterPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.util.Date;

@TeleOp(name = "Media Recorder Test", group = "Testing")
public class MediaRecorderTest extends BaseOpMode
{
	private OpenCvCamera camera;
	private VideoWriterPipeline videoWriterPipeline;

	@Override
	protected void OnInitialize()
	{
		File directory = new File(Environment.getExternalStorageDirectory().getAbsolutePath() + "/recordings");
		if (!directory.exists()) directory.mkdir();
		videoWriterPipeline = new VideoWriterPipeline(directory.getAbsolutePath() + "/recording_" + new Date().getTime() + ".mp4");
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		camera.setPipeline(videoWriterPipeline);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				camera.startStreaming(Constants.Camera.CAMERA_WIDTH, Constants.Camera.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode)
			{
				telemetry.addData("[ERROR] Camera failed to open with error code", errorCode);
				telemetry.update();
			}
		});
	}

	@Override
	protected void OnRun()
	{
		telemetry.addData("Recording", videoWriterPipeline.isRecording());
		telemetry.update();
	}

	@Override
	protected void OnStart()
	{
		videoWriterPipeline.startRecording();
	}

	@Override
	protected void OnStop()
	{
		videoWriterPipeline.stopRecording();
	}
}