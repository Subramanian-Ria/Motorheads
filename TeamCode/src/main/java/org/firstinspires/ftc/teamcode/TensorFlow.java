package org.firstinspires.ftc.teamcode;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareMap;

//TensorFlow Object class
//Credit to Viridian Robotics(Daniel Awesome)
public class TensorFlow {
    private boolean error;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    private static final String VUFORIA_KEY = "AVtZQ6n/////AAABmdcWQU7kykWgmYCE0DI/QOReCtZljUv/ks9BxJvDlzzFaMhm4I4BBhWA8BDMwM6wDclf7C3Uejvm+pnib7YV+D/n8iAQdR7MAGlBGRqXbUPG1HwHfKCK27WTAuNxHilwwMcEIyPjgJY+9ozAZtnbaWzCDZjrNC1WlClxqnGMT5qO93K2ARRy+3FKtNV93opS7YAVfhRSNxWh/bBRa05OWKjB41PdctoT1IWWsabSad2Fvj7qzasRnG+cmO2ePVMxIwEPC2w1K6gQSFBsk97Sku2EmqgsnRYNzqnXpPn/tXhsQhmiTpWkZUnZTfTsnx3jxB6vdfoZA+JKBqrlfqqqxiBWLx72h0f31ek6TuwdwTTd";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private HardwareMap hwMap;

    private Device device;

    private Telemetry telemetry;

    public enum RobotOrientation{
        Left, Right, Center
    }
    public enum MineralLocation{
        Left,Center,Right
    }
    public enum Device{
        Phone,Webcam
    }
    public TensorFlow(HardwareMap hwMap,Device device, Telemetry telemetry){
        this.telemetry = telemetry;
        this.hwMap = hwMap;
        this.device = device;
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        //error until tfd is started
        error = true;

    }
    public void start(){
        if (tfod != null) {
            tfod.activate();
            error = false;
        }
    }
    public void shutdown(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public MineralLocation nMineralLocations(RobotOrientation orientation, int n){
        int center = 0;
        int left = 0;
        int right = 0;
        for(int i = 0; i < n; i++){
            MineralLocation o = getMineralLocation(orientation);
            if(o == MineralLocation.Center){
                center++;
            }else if(o == MineralLocation.Right){
                right++;
            }else{
                left++;
            }
        }
        if(center > left && center > right){
            return MineralLocation.Center;
        }else if(right > left){
            return MineralLocation.Right;
        }else{
            return MineralLocation.Left;
        }
    }

    public MineralLocation getMineralLocation(RobotOrientation orientation){
        //the location of the mineral, defaults to center if TFOD fails or objects or not found
        MineralLocation absoluteLocation = MineralLocation.Center;
        //if tfod failed to init, just return center
        //otherwise, continue with detection
        if(!error) {
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            List<Recognition> filteredList = new ArrayList<Recognition>();

            /*for(Recognition recognition : updatedRecognitions){
                if(recognition.getHeight() < recognition.getImageHeight() * 3 / 11){
                    filteredList.add(recognition);
                }
            }*/
            //Variabes to store two mins
            Recognition min1 = null;
            Recognition min2 = null;
            //Iterate through all minerals
            for(Recognition recognition : updatedRecognitions){
                double height = recognition.getHeight();
                if (min1 == null){
                    min1 = recognition;
                }
                else if(min2 == null){
                    min2 = recognition;
                }
                else if(height < min1.getHeight()){
                    min1 = recognition;
                }
                else if(height < min2.getHeight()){
                    min2 = recognition;
                }
                if (min1 != null && min2 != null){
                    if(min1.getHeight() > min2.getHeight()){
                        Recognition temp = min1;
                        min1 = min2;
                        min2 = temp;
                    }
                }
            }
            filteredList.add(min1);
            filteredList.add(min2);
            int goldMineralX = -1;
            int silverMineral1X = -1;
            int silverMineral2X = -1;
            //Three Mineral Algorithm
            if(orientation == RobotOrientation.Center){
                for (Recognition recognition : filteredList) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        return MineralLocation.Left;
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        return MineralLocation.Right;
                    } else {
                        return MineralLocation.Center;
                    }
                }
            }
            else{//Two Mineral Algorithm
                //looks at each detected object, obtains "the most" gold and silver mineral
                float goldMineralConfidence = 0;
                float silverMineralConfidence = 0;
                for (Recognition recognition : updatedRecognitions) {
                    String label = recognition.getLabel();
                    float confidence = recognition.getConfidence();
                    int location = (int) recognition.getLeft();
                    if (label.equals(LABEL_GOLD_MINERAL)
                            && confidence > goldMineralConfidence) {
                        goldMineralX = location;
                        goldMineralConfidence = confidence;
                    } else if (label.equals(LABEL_SILVER_MINERAL)
                            && confidence > silverMineralConfidence) {
                        silverMineral1X = location;
                        silverMineralConfidence = confidence;
                    }
                }
                //using the two gold and silver object x locations,
                //obtains whether the gold mineral is on the relative left or the right
                boolean goldRelativeLeft;
                if (goldMineralX != -1 && silverMineral1X != -1) {
                    if (goldMineralX < silverMineral1X) {
                        goldRelativeLeft = true;
                    } else {
                        goldRelativeLeft = false;
                    }
                    // telemetry.addData("Relative", goldRelativeLeft);
                    //translates the relative location to an absolute location based off the orientation
                    if (orientation == RobotOrientation.Left) {
                        if (goldRelativeLeft) {
                            absoluteLocation = MineralLocation.Left;
                        } else {
                            absoluteLocation = MineralLocation.Center;
                        }
                        //telemetry.addData("orientation",absoluteLocation);
                    } else {
                        if (goldRelativeLeft) {
                            absoluteLocation = MineralLocation.Center;
                        } else {
                            absoluteLocation = MineralLocation.Right;
                        }
                        // telemetry.addData("orientation","fail");
                    }
                } //sees at least one silver (so not a reading from the wrong position, but no gold seen)
                else if(silverMineral1X != -1 && goldMineralX == -1){
                    if(orientation == RobotOrientation.Left){
                        absoluteLocation = MineralLocation.Right;
                    }else if(orientation == RobotOrientation.Right){
                        absoluteLocation = MineralLocation.Left;
                    }
                }
            }

        }
        return absoluteLocation;
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //initialize either phone or webcam depending on field set during object initialization
        if(device == Device.Phone){
            parameters.cameraDirection = CameraDirection.BACK;
        }else{
            parameters.cameraName = hwMap.get(WebcamName.class, "Webcam");

        }
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}