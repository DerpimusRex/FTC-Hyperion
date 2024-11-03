/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.FTC_Centerstage.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "AutoRosuSus", group = "Concept")

public class AutoRosuSus extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "PiramidaRosie.tflite";
    private static final String TFOD_MODEL_FILE = "PiramidaRosie.tflite";
    private static final String[] LABELS = {
            "piramida",
    };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    public boolean objectFound = false;
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
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 18.9;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;
    static int cadran;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        preload = hardwareMap.get(Servo.class, "servo1exp");
//        //brat = hardwareMap.get(DcMotor.class,"brat");
        preload.setPosition(1);

        initTfod();

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() <= 5) {

                sleep(5000);
                cadran = cadrane();

                telemetryTfod();
                telemetry.update();
                drive(0.5, 569.5, 569.5);
                if (cadran == 1) {
                    drive(0.5, 300, -300);
                    drive(0.5, 250, 250);
                    preload.setPosition(0.5);
//                    drive(0.5, -100, -100);
//                    drive(0.5, 300, -300);
//                    drive(0.5, 540, 540);
//                    drive(0.5, 300, -300);
//                    drive(0.5, 800, 800);
//                    //mergi putin in spate sa nu dai de truss
//                    drive(0.5, -100, -100);
//                    //roteste a.i. sa poti sa mergi direct in colt mergand in fata
//                    drive(0.5, 370, -370);
//                    //mergi in fata

//                    drive(0.5, 300, -300);
//                    drive(0.5, 90.35, 90.35);
                    telemetry.addData("Stanga", 1);
                    telemetry.update();
                } else if (cadran == 2) {
                    drive(0.5, 171.2, 171.2);
                    drive(0.5, 100, -100);

                    preload.setPosition(0.5);
//                    drive(0.5, -720.7, -720.7);
//                    drive(0.5, -340, 340);
//                    drive(0.5, 900, 900);
////                    //mergi un tile in fata
//                    drive(0.5, 350, 350);
//                    //roteste
//                    drive(0.5, -180, 180);
//                    //parcheaza pe mijloc
//                    drive(0.5, 200, 200);
                    telemetry.addData("Mijloc", 1);7
                    telemetry.update();


                } else if (cadran == 3) {
                    drive(0.5, 30, 30);
                    drive(0.5, -100, 100);
                    drive(0.5, 130, 130);
                    preload.setPosition(0.5);
//                    drive(0.5, 300, 300);
//                    drive(0.5, -300, 300);
//                    drive(0.5, 60.35, 60.35);
//                    preload.setPosition(0.5);





                    telemetry.addData("Dreapta", 1);
                    telemetry.update();
                    preload.setPosition(0.5);
                }
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                sleep(20);
            }
        }

    }

    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        if (!currentRecognitions.isEmpty()) {
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                objectFound = true;
                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }
        }
    }

    private void drive(double power, double leftMm, double rightMm) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            rightTarget = rightDrive.getCurrentPosition() + (int) (rightMm * DRIVE_COUNTS_PER_MM);
            leftTarget = leftDrive.getCurrentPosition() + (int) (leftMm * DRIVE_COUNTS_PER_MM);
            telemetry.addData("rightT", rightTarget);
            telemetry.addData("LeftT", leftTarget);
            telemetry.update();

            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftDrive.setPower(power);
            rightDrive.setPower(power);

            while (opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
                telemetry.addData("busyL", leftDrive.isBusy());
                telemetry.addData("busyR", rightDrive.isBusy());

                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }
    }

    private void park(int distance) {
        drive(0.5, 11.5, -11.5);
        drive(0.5, distance, distance);
    }

    private int cadrane() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        /*if(currentRecognitions == null || currentRecognitions.isEmpty())
        {
            telemetry.addData("Stanga", 1);
            return 3;
        }*/
        double maxConf = 0.0, xmax = -1.0;
        if (!currentRecognitions.isEmpty()) {
            for (Recognition recognition : currentRecognitions) {
                //int im_width = recognition.getImageWidth(); 640
                //int im_width = 640;
                //float left = recognition.getLeft();
                //float right = recognition.getRight();
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                if (recognition.getConfidence() > maxConf) {
                    xmax = x;
                    maxConf = recognition.getConfidence();
                }
            }
        }
        if (xmax < 320 && xmax >= 0) //0 stanga
        {
            telemetry.addData("Mijloc", 1);
            return 2;
        } else if (xmax >= 320) //480 dreapta
        {
            telemetry.addData("Dreapta", 1);
            return 3;
        } else return 1;
    }

}
    // sunt smecher!