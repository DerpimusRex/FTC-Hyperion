/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.FTC_Centerstage.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "regionala", group = "Iterative OpMode")

public class Regionala extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor axBrat = null;
    private DcMotor brat = null;
    private Servo avion = null;
    private Servo preload = null;
    private Servo carlig = null;
    private Servo axCarlig = null;
    private boolean stay;
    private boolean xpress;
    private boolean secrethihi;
    private int posc = 0;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        //motorul din dreapta(privind dinspre baterie spre avion), portul 0 control hub
        leftDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //motorul din stanga, portul 1 contrul hub
        rightDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorul gobilda, portul 2 control hub
        //controale: pe LT se ridica bratul, pe RB+LB+ bratul se coboara constant in jos pana cand se opreste OpModeul din driver station, gamepad 2
        brat = hardwareMap.get(DcMotor.class, "hang");
        //motorul din fata, portul 3 control hub
        //controale:joystickul din dreapta, gamepad 2
        axBrat = hardwareMap.get(DcMotor.class, "axBrat");
        axBrat.setDirection(DcMotorSimple.Direction.FORWARD);
        axBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        //servo avion, portul 5 pe expansion hub
        //controlat pe Y
        avion = hardwareMap.get(Servo.class,"avion");
        avion.setPosition(1);
        // servoul de la cleste, portul 5 de la control hub
        // controlat pe X
        carlig = hardwareMap.get(Servo.class, "carlig");
        carlig.setPosition(1);
        //servoul de la axul de carlig, portul 4 de la control hub
        //3 controale, dpad down e pozitia initiala, dpad left e pozitia dreapta pentru a lua pixeli, dpad right pentru a pune pixel pe backboard(nu o vom fplosi niciodata)
        axCarlig = hardwareMap.get(Servo.class, "axCarlig");
        axCarlig.setPosition(0);
//        //brat = hardwareMap.get(DcMotor.class,"brat");
//        // brat.setPower(0);
        preload = hardwareMap.get(Servo.class, "servo1exp");
        stay = false;
        xpress = false;
        telemetry.addData("Status", "Initialized");

        waitForStart();

        if(opModeIsActive())
        {
            while(opModeIsActive())
            {
                double leftPower;
                double rightPower;
                double liftPower;
                double rotateSpeed;
                double[] carligPos = {0, 0.9, 1};
                //DRIVE
                double drive = -gamepad1.left_stick_y;
                double turn  =  -gamepad1.right_stick_x;
                double lift = -gamepad2.left_trigger;
                double rotate = -gamepad2.right_stick_y;
                leftPower    = Range.clip(drive + turn, -0.6, 0.6) ;
                rightPower   = Range.clip(drive - turn, -0.6, 0.6) ;
                rotateSpeed = Range.clip( rotate, -0.3, 0.3);
                if(stay == false) liftPower = Range.clip(lift, -0.7, 0.7); else liftPower = 0.7;
                if(gamepad2.right_bumper && gamepad2.left_bumper)stay = true;
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
                brat.setPower(liftPower);
                axBrat.setPower(rotateSpeed);

                //avion
                boolean a_touched=false;
                if(gamepad2.a) a_touched = true;
                if(a_touched)
                {
                    avion.setPosition(0);
                    sleep(200);
                    avion.setPosition(1);
                }

                //carlig
                if(!xpress)carlig.setPosition(1);
                if(gamepad2.x)
                {
                    carlig.setPosition(0);
                    xpress = true;
                } else xpress = false;

                //axCarlig
                if(gamepad2.dpad_down)
                {
                    posc = 0;
                } else if(gamepad2.dpad_left)
                {
                    posc = 1;
                } else if(gamepad2.dpad_right)
                {
                    posc = 2;
                }
                axCarlig.setPosition(carligPos[posc]);
                if(!secrethihi)preload.setPosition(1);
                if(gamepad1.x)
                {
                    preload.setPosition(0);
                    secrethihi = true;
                } else secrethihi = false;
        /*if(gamepad2.b)
        {
            brat.setPowe
            +r(0.5);
        }
        else brat.setPower(0);
        if(gamepad2.x)
        {
            brat.setPower(-0.5);
        }
        else brat.setPower(0);*/


                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Gamepad", "drive (%.2f), turn (%.2f)", drive, turn);
                telemetry.addData("Motors", "left (%.2f), right (%.2f), lift (%.2f), rotate(%.2f)", leftPower, rightPower, liftPower, rotateSpeed);
            }
        }
    }
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//
//        leftDrive  = hardwareMap.get(DcMotor.class, "rightDrive");
//        rightDrive = hardwareMap.get(DcMotor.class, "leftDrive");
//        brat = hardwareMap.get(DcMotor.class, "hang");
//        axBrat = hardwareMap.get(DcMotor.class, "axBrat");
//
//        avion = hardwareMap.get(Servo.class,"avion");
//        carlig = hardwareMap.get(Servo.class, "carlig");
//        axCarlig = hardwareMap.get(Servo.class, "axCarlig");
//        //brat = hardwareMap.get(DcMotor.class,"brat");
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        axBrat.setDirection(DcMotorSimple.Direction.FORWARD);
//        axBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
//        avion.setPosition(1);
//        axCarlig.setPosition(0);
//        carlig.setPosition(1);
//       // brat.setPower(0);
//        stay = false;
//        telemetry.addData("Status", "Initialized");
//    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
//    @Override
//    public void init_loop() {
//    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
//    @Override
//    public void start() {
//        runtime.reset();
//    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
//    @Override
//    public void loop() {
//        double leftPower;
//        double rightPower;
//        double liftPower;
//
//        //DRIVE
//        double drive = -gamepad1.left_stick_y;
//        double turn  =  -gamepad1.right_stick_x;
//        double lift = -gamepad2.left_trigger;
//        leftPower    = Range.clip(drive + turn, -0.6, 0.6) ;
//        rightPower   = Range.clip(drive - turn, -0.6, 0.6) ;
//        if(stay == false) liftPower = Range.clip(lift, -0.7, 0.7); else liftPower = 0.7;
//        if(gamepad2.right_bumper)stay = true;
//        leftDrive.setPower(leftPower);
//        rightDrive.setPower(rightPower);
//        brat.setPower(liftPower);
//
//        //avion
//        boolean a_touched=false;
//        if(gamepad2.a) a_touched = true;
//        if(a_touched)
//        {
//            avion.setPosition(0);
//            sleep(200);
//            avion.setPosition(1);
//        }
//        /*if(gamepad2.b)
//        {
//            brat.setPower(0.5);
//        }
//        else brat.setPower(0);
//        if(gamepad2.x)
//        {
//            brat.setPower(-0.5);
//        }
//        else brat.setPower(0);*/
//
//
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Gamepad", "drive (%.2f), turn (%.2f)", drive, turn);
//        telemetry.addData("Motors", "left (%.2f), right (%.2f), lift (%.2f)", leftPower, rightPower, liftPower);
//    }
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//    }

//    public final void sleep(long milliseconds) {
//        try {
//            Thread.sleep(milliseconds);
//        } catch (InterruptedException e) {
//            Thread.currentThread().interrupt();
//        }
//    }

    private void drive (double power, double mm) {
        int target;

        if (opModeIsActive())
        {
            target = axBrat.getCurrentPosition() + (int)(mm * DRIVE_COUNTS_PER_MM);
            telemetry.addData("right",target);
            telemetry.update();

            axBrat.setTargetPosition(target);

            axBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);;

            axBrat.setPower(power);

            while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
                telemetry.addData("busyL",leftDrive.isBusy());
                telemetry.addData("busyR",rightDrive.isBusy());

                telemetry.update();
            }

            axBrat.setPower(0);
        }
    }
}
