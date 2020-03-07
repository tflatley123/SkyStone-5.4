package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// this class makes the code in the autonomous less clunky

public class TFOD
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AVC8YYn/////AAABmTHcycHSw0jDoQ+NF6xCbB0tUGyWIV0iiSkEbyZIziI90kxeCYRqCG7+1p7yDhFHRxqP7ShVVBH8UxcYqEaPEX2TBixRKadnQhqJeGfqEsBcZLren0DnbUywTotQPe4qsssno9xfq50fMYU3AxTRURF66n5ATUisE21fF7U372VPIBnPgR27/HlkKMKXkzVko3LDiwX4NMXBMxR9Egez2RVAK8WvGAJ+n9E1OVKGVMnAwo555OUh56yMA/6BxfNsMVUts7QMEFMev68Am+CZ12ilxov/mV+xS+TOO/ZbWCxCQZUd+HAuTxcjDw3s5Qsy7aspYQRszP46cLdDyvzNnwL11fkeRn1ZK7jlK2fD3FRZ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    public List<Recognition> recognizedObjects;


    public boolean initialize(HardwareMap hardwareMap)
    {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.useExtendedTracking = false;
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.55;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

            if (tfod != null)
            {
                tfod.activate();

            }
            com.vuforia.CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
            com.vuforia.CameraDevice.getInstance().setField("zoom", "20");
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
            //tfod.setClippingMargins(40, 130,100, 240);
        }
        else
        {
            return false;
        }

        recognizedObjects = tfod.getUpdatedRecognitions();
        return true;
    }

    public void updateDetectedObjects()
    {
        recognizedObjects = tfod.getUpdatedRecognitions();
    }
    public void shutDown()
    {
        tfod.shutdown();
    }

    // this puts the objects in order from least angle to greatest angle
    public void updateLineUp()
    {

        updateDetectedObjects();
        List<Recognition> tempList = recognizedObjects;
        if(recognizedObjects != null) {
            for (int i = 0; i < tempList.size() - 1; i++) {
                for (int j = 0; j < tempList.size() - 1; j++) {
                    if (tempList.get(i).estimateAngleToObject(AngleUnit.DEGREES) > tempList.get(j).estimateAngleToObject(AngleUnit.DEGREES)) {
                        Recognition temporary = tempList.get(j);
                        tempList.set(j, tempList.get(i));
                        tempList.set(i, temporary);
                    }
                }
            }
            recognizedObjects = tempList;
        }
    }



}