package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.BallisticTrajectoryUtil;
import org.firstinspires.ftc.teamcode.util.PolynomialCalculator.Vector3;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@TeleOp(group = "drive")
public class AimTest extends LinearOpMode {
    public static double TARGETX = 0; // in
    public static double TARGETY = 0; // in
    public static double TARGETHEIGHT = 0; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector3 launcherPos = new Vector3(DriveConstants.relativeLauncherPos.rotated(poseEstimate.getHeading()).plus(poseEstimate.vec()), DriveConstants.relativeLauncherHeight);
            Vector3 targetPos = new Vector3(new Vector2d(TARGETX, TARGETY), TARGETHEIGHT);

            Vector3 solution1 = Vector3.zero;
            Vector3 solution2 = Vector3.zero;
            int solutions = BallisticTrajectoryUtil.solve_ballistic_arc(
                    BallisticTrajectoryUtil.robotToFieldVelocityVector(DriveConstants.relativeLauncherPos, poseEstimate, drive.getPoseVelocity()),
                    launcherPos,
                    DriveConstants.launchSpeed,
                    targetPos,
                    Vector3.zero,
                    DriveConstants.gravity,
                    solution1,
                    solution2);

            double targetHeading = solution1.getGroundVector2d().angle();
            double targetAim = solution1.getSkyVector2d().angle();
            drive.turnAsync(targetHeading - poseEstimate.getHeading());
            drive.aimLauncher(targetAim);

            while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
                drive.update();
                telemetry.addData("Projectile Targeting", launcherPos.toPoseString() + " -> " + launcherPos.toPoseString());
                telemetry.addData("Robot Heading", drive.getPoseEstimate().getHeading());
                telemetry.addData("Solution 1", String.format("Heading: %.2f, Aim: %.2f", targetHeading, targetAim));
                telemetry.addLine();
                telemetry.addData("Solution 2", String.format("Heading: %.2f, Aim: %.2f", solution2.getGroundVector2d().angle(), solution2.getSkyVector2d().angle()));
                telemetry.addData("Total Solutions", solutions);
            }
        }
    }
}
