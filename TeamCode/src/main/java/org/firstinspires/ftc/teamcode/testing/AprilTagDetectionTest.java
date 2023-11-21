package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
 * Test the camera by displaying the AprilTag detections on the screen
 */
@TeleOp(name = "Camera Test", group = "Testing")
public final class AprilTagDetectionTest extends BaseOpMode
{
	private OpenCvCamera camera;
	private AprilTagDetectionPipeline detectionPipeline;

	@Override
	protected void OnInitialize()
	{
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		detectionPipeline = new AprilTagDetectionPipeline(
				Constants.Camera.CAMERA_FX,
				Constants.Camera.CAMERA_FY,
				Constants.Camera.CAMERA_CX,
				Constants.Camera.CAMERA_CY);
		camera.setPipeline(detectionPipeline);
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
				telemetry.addData("Camera Error", errorCode);
			}
		});

		telemetry.setMsTransmissionInterval(50);
	}

	@Override
	protected void OnRun()
	{
		ArrayList<AprilTagDetection> currentDetections = detectionPipeline.getLatestDetections();

		telemetry.addData("No. Objects", currentDetections.size());
		for (int i = 0; i < currentDetections.size(); i++) {
			AprilTagDetection detection = currentDetections.get(i);

			telemetry.addData("Object " + i, detection.id);
			telemetry.addData("Margin", detection.decisionMargin);
			telemetry.addData("Center", detection.center);
		}
		telemetry.update();
	}
}