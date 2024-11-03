package org.firstinspires.ftc.teamcode.autonomie;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "cameraCalibrare", group = "Concept")

public class cameraCalibrare extends OpMode
{
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "model_20240123_202310.tflite";
    private static final String TFOD_MODEL_FILE = "model_20240123_202310.tflite";

    private static final String[] LABELS = {
            "piramida",
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;
    public boolean objectFound = false;

    @Override
    public void init()
    {
        initTfod();
        telemetry.addData("Status", "Initialized");
    }

    public void loop(){

        telemetryTfod();
        cadrane();

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

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            objectFound = true;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("ImageSize1", recognition.getImageHeight());
            telemetry.addData("ImageSize2", recognition.getImageWidth());
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Right", "%.0f", recognition.getRight());
            telemetry.addData("- Left", "%.0f ", recognition.getLeft());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void cadrane()
    {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        for(Recognition recognition : currentRecognitions)
        {
            //int im_width = recognition.getImageWidth(); 640
            //int im_width = 640;
            //float left = recognition.getLeft();
            //float right = recognition.getRight();
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            if(x < 170) //160 stanga
            {
                telemetry.addData("Stanga", 1);
            }
            else if(x > 470) //480 dreapta
            {
                telemetry.addData("Dreapta", 1);
            }
            else
            {
                telemetry.addData("Mijloc", 1);
            }
        }
    }
}
