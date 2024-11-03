package org.firstinspires.ftc.teamcode.FTC_Centerstage.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "senzorculoare", group = "Sensor")
public class SenzorCuloare extends LinearOpMode{
    ColorSensor senzorCl;
    public int[] rgbValues = new int[3];
    public float[] NormrgbValues = new float[3];
    @Override
    public void runOpMode() {
        senzorCl = hardwareMap.get(ColorSensor.class, "senzorcl");
        telemetry.addData("Status", "Initialized");
        waitForStart();
        while (opModeIsActive()) {
            senzor();
        }
    }

    public void senzor()
    {
        int a = 2;
        if(a % 2 == 0)
        {
            rgbValues = rgb();
            NormrgbValues = Normalizedrgb();
            /*boolean xButtonCurrentlyPressed;
            boolean xButtonPreviouslyPressed = false;
            xButtonCurrentlyPressed = gamepad1.x;
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                if (xButtonCurrentlyPressed) {
                    senzorCl.enableLed(xButtonCurrentlyPressed);
                }
            }
            senzorCl.enableLed(xButtonCurrentlyPressed);
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;*/
            senzorCl.enableLed(false);


            telemetry.addData("Red", rgbValues[0]); // a % 2
            telemetry.addData("Green", rgbValues[1]);
            telemetry.addData("Blue", rgbValues[2]);
            telemetry.addData("Red", NormrgbValues[0]); // a % 2
            telemetry.addData("Green", NormrgbValues[1]);
            telemetry.addData("Blue", NormrgbValues[2]);
            telemetry.update();
        }
    }

    public int[] rgb()
    {
        int[] rgbV = new int[3];
        rgbV[0] = senzorCl.red();
        rgbV[1] = senzorCl.green();
        rgbV[2] = senzorCl.blue();

        return rgbV;
    }

    public float[] Normalizedrgb()
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
