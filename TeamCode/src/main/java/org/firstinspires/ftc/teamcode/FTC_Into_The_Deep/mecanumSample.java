package org.firstinspires.ftc.teamcode.FTC_Into_The_Deep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class mecanumSample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("StangaSus");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("StangaJos");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("DreaptaSus");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("DreaptaJos");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = -(y + x + rx) / denominator;
            frontLeftPower = Range.clip(frontLeftPower, -0.8, 0.6);
            double backLeftPower = -(y - x + rx) / denominator;
            backLeftPower = Range.clip(backLeftPower, -0.6, 0.6);
            double frontRightPower = -(y - x - rx) / denominator;
            frontRightPower = Range.clip(frontRightPower, -0.6, 0.6);
            double backRightPower = -(y + x - rx) / denominator;
            backRightPower = Range.clip(backRightPower, -0.6, 0.6);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}