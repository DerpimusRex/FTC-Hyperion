package org.firstinspires.ftc.teamcode.FTC_Into_The_Deep.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class autoneamt extends LinearOpMode {

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("StangaSus");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("StangaJos");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("DreaptaSus");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("DreaptaJos");
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController xControl = new PIDController(0,0,0);
        PIDController yControl = new PIDController(0,0,0);
        PIDController thetaControl = new PIDController(0,0,0);

        double xTarget = 1000;
        double yTarget = 0;
        double thetaTarget = 0;
        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive())
        {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double x = xControl.update(xTarget, frontLeftMotor.getCurrentPosition() + backLeftMotor.getCurrentPosition());
            double y = yControl.update(yTarget, 0);
            double t = thetaControl.update(thetaTarget, 0);

            double x_rotated = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double y_rotated = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(t), 1);
            double frontLeftPower = -(y + x + t) / denominator;
            frontLeftPower = Range.clip(frontLeftPower, -0.8, 0.8);
            double backLeftPower = -(y - x + t) / denominator;
            backLeftPower = Range.clip(backLeftPower, -0.8, 0.8);
            double frontRightPower = -(y - x - t) / denominator;
            frontRightPower = Range.clip(frontRightPower, -0.8, 0.8);
            double backRightPower = -(y + x - t) / denominator;
            backRightPower = Range.clip(backRightPower, -0.8, 0.8);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
