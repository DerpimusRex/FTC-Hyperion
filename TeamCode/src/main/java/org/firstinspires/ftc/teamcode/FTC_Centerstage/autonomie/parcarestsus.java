package org.firstinspires.ftc.teamcode.FTC_Centerstage.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

//import java.util.List;
@Autonomous
public class parcarestsus extends LinearOpMode {
    private DcMotor RightDrive;
    private DcMotor LeftDrive;
    private DcMotor armLeft;
    private DcMotor armRight;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM = DRIVE_COUNTS_PER_MM * 10;
    private Servo pixel =null;
    public int scenario=0;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "model_20240123_202310.tflite";
    // private static final String TFOD_MODEL_FILE = "model_20240123_202310.tflite";
    private static final String TFOD_MODEL_FILE = "model_20240123_202310.tflite";
    private static final String[] LABELS = {
            "piramida",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor senzorCl;
    public int[] rgbValues = new int[3];
    public float[] NormrgbValues = new float[3];

    @Override
    public void runOpMode() {

        RightDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        LeftDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        //armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        //armRight = hardwareMap.get(DcMotor.class, "armRight");
        pixel = hardwareMap.get(Servo.class,"pixel");
        senzorCl = hardwareMap.get(ColorSensor.class, "senzorcl");
        // reverse left drive motor direciton
        LeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("timp","ceva"+ runtime.toString());
        telemetry.update();
        initTfod();
        waitForStart();
        if (opModeIsActive()) {
            initTfod();
            // segment 1
            pixel.setPosition(0);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()<3)
            {
                //drive(0.5,10,10);
                /*try{
                    cameraCalibrare.cadrane();
                }
                catch(Exception e)
                {
                    telemetry.addData("eroare","eroare");
                    telemetry.update();
                }*/

                calibrare();
                drive(0.5,55,55);
                telemetry.addData("scenario",scenario);
                telemetry.update();

                if(scenario == 1)
                {
                    drive(0.5, -30.0, 30.0);

                    while(!(rgbValues[2] >=150 && NormrgbValues[2] >=0.41))
                    {
                        drive(0.2, 1.0, 1.0);
                        senzorCuloare();
                    }
                    pixel.setPosition(1);
                    drive(0.5, 65, 65);

                } else if (scenario == 3)
                {
                    drive(0.5, 30.0, -30.0);
                    while(!(rgbValues[2] >=150 && NormrgbValues[2] >=0.41))
                    {
                        drive(0.2, 1.0, 1.0);
                        senzorCuloare();
                    }
                    pixel.setPosition(1);
                } else
                {
                    while(!(rgbValues[2] >=150 && NormrgbValues[2] >=0.41))
                    {
                        drive(0.2, 1.0, 1.0);
                        senzorCuloare();
                    }
                    pixel.setPosition(1);
                }
            }
        }
    }
    private void initTfod()
    {
        tfod = new TfodProcessor.Builder()
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to

                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if(USE_WEBCAM)
        {
            builder.setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"));
        }
        else
        {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }
    private void calibrare()
    {
        List<Recognition> currentRecognitions;
        currentRecognitions = tfod.getRecognitions();
        if(currentRecognitions.isEmpty() == true)
        {
            scenario = 1;
            telemetry.addData("Stanga",1);
            //visionPortal.stopStreaming();
            return;
        }
        for(Recognition recognition : currentRecognitions)
        {
            //int im_width = recognition.getImageWidth(); 640
            //int im_width = 640;
            //float left = recognition.getLeft();
            //float right = recognition.getRight();
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            if(x <= 400) //310 stanga
            {
                telemetry.addData("Mijloc", 1);
                scenario = 2;
            }
            else
            {
                telemetry.addData("Dreapta", 1);
                scenario = 3;
            }
        }
        //visionPortal.stopStreaming();

    }
    private void drive (double power, double leftInches, double rightInches) {
        int rightTarget;
        int leftTarget;

        if (opModeIsActive()) {
            // Create target positions
            rightTarget = RightDrive.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_CM);
            leftTarget = LeftDrive.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_CM);
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
            LeftDrive.setPower(power);
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

    private void senzorCuloare()
    {
        rgbValues = rgb();
        NormrgbValues = Normalizedrgb();
    }

    private int[] rgb()
    {
        int[] rgbV = new int[3];
        rgbV[0] = senzorCl.red();
        rgbV[1] = senzorCl.green();
        rgbV[2] = senzorCl.blue();

        return rgbV;
    }

    private float[] Normalizedrgb()
    {
        float[] Nrgb = new float[3];
        int[] orgNrgb = rgbValues;

        int total = 0;
        for(int i : orgNrgb)
        {
            total += i;
        }

        for(int i = 0; i < 3; ++i)
        {
            if(total>0)
            {
                Nrgb[i] = (float)orgNrgb[i] / total;
            }
            else
            {
                Nrgb[i]=0;
            }
        }

        return Nrgb;
    }
}
