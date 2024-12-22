package org.firstinspires.ftc.teamcode.FTC_Into_The_Deep.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class autovechidaenouptneamt extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.203;
    static final double WHEEL_CIRCUMFERENCE_MM = 96 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.dcMotor.get("StangaSus");
        backLeftMotor = hardwareMap.dcMotor.get("StangaJos");
        frontRightMotor = hardwareMap.dcMotor.get("DreaptaSus");
        backRightMotor = hardwareMap.dcMotor.get("DreaptaJos");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("timp","ceva"+ runtime.toString());
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // segment 1
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() <= 1)
            {
                telemetry.addData("timp","ceva"+ runtime.toString());
                telemetry.update();
                /*while(runtime.seconds() >2 && runtime.seconds()<=4)
                {*/
                //drive(0.5, 40, 40);
                telemetry.addData("timp",runtime.seconds());
                telemetry.update();
                /*}
                while(runtime.seconds() >6 && runtime.seconds()<=8)
                {*/
                drive(0.5, 20/*12.5*/, 20);
                telemetry.addData("timp",runtime.seconds());
                telemetry.update();
                /*}
                while(runtime.seconds() >10 && runtime.seconds()<=12)
                {*/
                //drive(0.5, 40, 40);
                telemetry.addData("timp",runtime.seconds());
                telemetry.update();
                //}

            }
        }
    }
    private void drive (double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions
            rightTarget = frontRightMotor.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = frontLeftMotor.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_IN);
            telemetry.addData("rightT",rightTarget);
            telemetry.addData("LeftT",leftTarget);
            telemetry.update();
            // set target position
            frontLeftMotor.setTargetPosition(leftTarget);
            backLeftMotor.setTargetPosition(leftTarget);
            frontRightMotor.setTargetPosition(rightTarget);
            backRightMotor.setTargetPosition(rightTarget);

            //switch to run to position mode
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the desiginated power
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (frontLeftMotor.isBusy() || frontRightMotor.isBusy())) {
                telemetry.addData("busyL",frontLeftMotor.isBusy());
                telemetry.addData("busyR",frontRightMotor.isBusy());

                telemetry.update();
            }
            /*while(runtime.seconds()>2 && runtime.seconds()<=4) {
            }*/

            // set motor power back to 0
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}