package me.xbl4z3r.meepmeeptesting;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Internal;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.QuinticSpline1d;
import com.acmerobotics.roadrunner.QuinticSpline2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting
{
	public static void main(String[] args)
	{
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity robot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
				.build();

		robot.runAction(
				robot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
						.setTangent(Math.PI)
						.splineToConstantHeading(new Vector2d(-10, 15), Math.PI)
						.build()
		);

		meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
				.addEntity(robot)
				.start();
	}
}