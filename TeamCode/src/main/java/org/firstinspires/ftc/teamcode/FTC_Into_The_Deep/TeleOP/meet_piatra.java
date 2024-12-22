package org.firstinspires.ftc.teamcode.FTC_Into_The_Deep.TeleOP;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class meet_piatra extends LinearOpMode {
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
        DcMotor axMotor = hardwareMap.dcMotor.get("Ax");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        axMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reverse the right side motors. This may be wrong for your setup.
        CRServo intake = hardwareMap.crservo.get("intake");
        Servo intakeRot = hardwareMap.servo.get("intakeRot");
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeRot.setPosition(0);

        double lastAxPower = 0;

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

            if(gamepad2.dpad_down)
            {
                intake.setPower(1);
                intakeRot.setPosition(1);
            } else
            {
                intakeRot.setPosition(0);
                if(gamepad2.a)
                {
                    intake.setPower(-1);
                } else intake.setPower(0);
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
            frontLeftPower = Range.clip(frontLeftPower, -0.9, 0.9);
            double backLeftPower = -(rotY - rotX + rx) / denominator;
            backLeftPower = Range.clip(backLeftPower, -0.9, 0.9);
            double frontRightPower = -(rotY - rotX - rx) / denominator;
            frontRightPower = Range.clip(frontRightPower, -0.9, 0.9);
            double backRightPower = -(rotY + rotX - rx) / denominator;
            backRightPower = Range.clip(backRightPower, -0.9, 0.9);
            double axPower= (gamepad2.right_trigger - gamepad2.left_trigger)*8.0/10.0;
//            axPower = Range.clip(axPower, -0.4, 0.4);
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            axMotor.setPower(axPower!=0?axPower:Math.signum(lastAxPower)*(-0.25));
            if(axPower!=0) lastAxPower = axPower;
        }
    }
}
