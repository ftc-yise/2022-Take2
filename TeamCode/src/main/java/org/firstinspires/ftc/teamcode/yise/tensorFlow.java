package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AutonomousRedLeft;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import java.util.List;

public class tensorFlow {

    private static final String TFOD_MODEL_ASSET = "model2.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "Blue Y",
            "Green S",
            "Red E"
    };

    private static final String VUFORIA_KEY = "AYvN/Qn/////AAABmTaYCa1SUkR7v5u6+3uR8CkLP9DKgYYzF/zz/slKnKktetT5kD2UyolBbzcRUiOv5ve/Bo+wJ5V01NzDGkWOqbwCwdrrwIau2aTGiVuF+nclQtuaZrp5qnMiWm4G2FUpgmQgoIbKZF2jrtyKfA957eIT6TxfN6CmPuFCjq/hd9FUMg5NjBZKOB3MmsxkgclcDW1U7YfPdVSTZURhXrg705CVtRFtwwy5TPfWCn9fYPLCFihlQpmQrg1D/hvTmlWtqA2edw0u7LaEhzZtFjmgB/O98qlZPqFixSX0yOIpmY04434VJO7uF9+9NAjGnfgtX207yf2TwNm9S4mfe4hCuka1WdNUzdJUkxfciaBtn2BJ";

    public VuforiaLocalizer vuforia;

    public TFObjectDetector tfod;

    public HardwareMap hardwareMap;

    public AutonomousRedLeft auto = new AutonomousRedLeft();

    //public List<Recognition> updatedRecognitions;

    public tensorFlow(HardwareMap hw) {
        hardwareMap = hw;
    }

    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
    // first.

    public int readCone() {
        //int rec = 0;

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel() == "Green S") {
                        return 2;
                    }
                    if (recognition.getLabel() == "Red E") {
                        return 1;
                    }
                }
            }
        }
        return 3;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.0, 16.0 / 9.0);
        }
    }
}
