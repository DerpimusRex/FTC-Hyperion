package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class autotestgo extends LinearOpMode {
    private DcMotor RightDrive;
    private DcMotor LeftDrive;
    private DcMotor armLeft;
    private DcMotor armRight;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        RightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        LeftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armRight = hardwareMap.get(DcMotor.class, "armRight");

        // reverse left drive motor direcito
        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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
                //drive(0.5, 50, 40);
                telemetry.addData("timp",runtime.seconds());
                telemetry.update();
                /*}
                while(runtime.seconds() >6 && runtime.seconds()<=8)
                {*/
                drive(0.5, +12.5, 0);
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
            rightTarget = RightDrive.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_IN);
            leftTarget = LeftDrive.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_IN);
            telemetry.addData("rightT",rightTarget);
            telemetry.addData("LeftT",leftTarget);
            telemetry.update();
            // set target position
            LeftDrive.setTargetPosition(leftTarget);
            RightDrive.setTargetPosition(rightTarget);

            //switch to run to position mode
            LeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //run to position at the desiginated power
            LeftDrive.setPower(power+0.15);
            RightDrive.setPower(power);

            // wait until both motors are no longer busy running to position
            while (opModeIsActive() && (LeftDrive.isBusy() || RightDrive.isBusy())) {
                telemetry.addData("busyL",LeftDrive.isBusy());
                telemetry.addData("busyR",RightDrive.isBusy());

                telemetry.update();
            }
            /*while(runtime.seconds()>2 && runtime.seconds()<=4) {
            }*/

            // set motor power back to 0
            LeftDrive.setPower(0);
            RightDrive.setPower(0);
        }
    }

}
