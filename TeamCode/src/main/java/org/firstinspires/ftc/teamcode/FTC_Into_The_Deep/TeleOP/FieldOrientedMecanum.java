package org.firstinspires.ftc.teamcode.FTC_Into_The_Deep.TeleOP;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldOrientedMecanum extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
//        imu.resetYaw();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("StangaSus");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("StangaJos");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("DreaptaSus");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("DreaptaJos");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors. This may be wrong for your setup.

        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            if(gamepad1.options)
            {
                imu.resetYaw();//poti sa resetezi unghiul si manual daca vrei
            }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double rotX = x * Math.cos(botHeading) - y * Math.sin(-botHeading);
            rotX*=1.1;
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(rotY + rotX + rx) / denominator;
            frontLeftPower = Range.clip(frontLeftPower, -1, 1);
            double backLeftPower = -(rotY - rotX + rx) / denominator;
            backLeftPower = Range.clip(backLeftPower, -1, 1);
            double frontRightPower = -(rotY - rotX - rx) / denominator;
            frontRightPower = Range.clip(frontRightPower, -1, 1);
            double backRightPower = -(rotY + rotX - rx) / denominator;
            backRightPower = Range.clip(backRightPower, -1, 1);
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}