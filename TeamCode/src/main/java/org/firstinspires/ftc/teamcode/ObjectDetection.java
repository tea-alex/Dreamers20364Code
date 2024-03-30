package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "TensorFlow RED")

public class ObjectDetection extends LinearOpMode {
    Config_robot robot = new Config_robot();
    public double route = 0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RED.tflite"; //private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "RED",// "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 10.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.04;     // Larger is more responsive, but also less stable

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    private int BleftTarget = 0;
    private int BrightTarget = 0;
    public static double MULTIPLIER = 0.9;

    @Override
    public void runOpMode() {

        initTfod();

        //detect();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        robot.init_Auto(hardwareMap);
        waitForStart();

        if (opModeIsActive()) {
         //   while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();
/*
                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

 */

                // Share the CPU.
                sleep(20);

                if (route == 1){
                    AUTONOM_LEFT();
                }
                else if (route == 2){
                    AUTONOM_CENTER();
                }

                else if (route == 3){
                    AUTONOM_RIGHT();
                }
                else{
                    AUTONOM_CENTER();
                }
            }
//        visionPortal.close();
        }

        // Save more CPU resources when camera is no longer needed.
 //       visionPortal.close();

 //   }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
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

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x > 180 && x < 540) {
                route = 2;
            }

            else if (x < 180){
                route = 1;
            }
            else if (x > 540){
                route = 3;
            }
            else{
                route = 2;
            }
        }   // end for() loop

    }   // end method telemetryTfod()
/*
    private void detect() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            if (x > 427 && x < 854) {
                route = 2;
            }
            else if (x < 427){
                route = 1;
            }
            else if (x > 854){
                route = 3;
            }
            else{
                route = 2;
            }
        }

    }

 */

    void AUTONOM_CENTER(){

        driveStraight(0.5, -12, 0);
        sleep(500);
        servoOpen();
        sleep(500);
        driveStraight(0.5,2,0);
        sleep(500);
        servoClose();
        sleep(500);
        turnToHeadingMecanum(0.3, -75);
        sleep(500);
        resetHeading();
        sleep(500);
        driveStraight(0.3,-20, 0);
        sleep(500);

        /*
        turnToHeadingMecanum(0.3, -90);
        sleep(500);
        driveStraight(0.5, -18, 0);
        sleep(500);

        lift_up();
        sleep(500);
        lift_down();

         */
        sleep(500);
        off_motor();
    }
    void AUTONOM_LEFT(){
    /*
        driveStraight(0.5, -12, 0);
        sleep(500);
        turnToHeadingMecanum(0.5,75);
        sleep(500);
        driveStraight(0.3,-1.5,0);
        sleep(500);
        servoOpen();
        sleep(500);
        driveStraight(0.3,3,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 0);
        sleep(500);
        resetHeading();
        sleep(500);
        driveStraight(0.5, 12, 0);
        sleep(500);
        resetHeading();
        sleep(500);
        turnToHeadingMecanum(0.3, -75);
        sleep(500);
        resetHeading();
        sleep(500);
        driveStraight(0.3,-20, 0);
        sleep(500);
         */
        sleep(500);
        off_motor();
    }

    void AUTONOM_RIGHT(){

        /*
                driveStraight(0.5, -12, 0);
        sleep(500);
        turnToHeadingMecanum(0.5,-75);
        sleep(500);
        driveStraight(0.5, -1.5, 0);
        servoOpen();
        sleep(500);
        driveStraight(0.3,3,0);
        sleep(500);
        turnToHeadingMecanum(0.3, 0);
        sleep(500);
        resetHeading();
        sleep(500);
        driveStraight(0.5, 12, 0);
        sleep(500);
        resetHeading();
        sleep(500);
        turnToHeadingMecanum(0.3, -75);
        sleep(500);
        resetHeading();
        sleep(500);
        driveStraight(0.3,-20, 0);
        sleep(500);
         */
        sleep(500);
        off_motor();
    }
    public void driveStraight(double maxDriveSpeed, double distance, double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftTarget = robot.FrontLeftDriverMotor.getCurrentPosition() + moveCounts;
            rightTarget = robot.FrontRightDriverMotor.getCurrentPosition() + moveCounts;
            BleftTarget = robot.BackLeftDriverMotor.getCurrentPosition() + moveCounts;
            BrightTarget = robot.BackRightDriverMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.FrontLeftDriverMotor.setTargetPosition(leftTarget);
            robot.FrontRightDriverMotor.setTargetPosition(rightTarget);
            robot.BackLeftDriverMotor.setTargetPosition(BleftTarget);
            robot.BackRightDriverMotor.setTargetPosition(BrightTarget);

            robot.FrontLeftDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeftDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRightDriverMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.FrontLeftDriverMotor.isBusy() && robot.FrontRightDriverMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            robot.FrontLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.FrontLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
        robot.FrontRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
        robot.BackRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
        robot.BackLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
    }

    public double getRawHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    void off_motor(){
        robot.BackLeftDriverMotor.setPower(0);
        robot.BackRightDriverMotor.setPower(0);
        robot.FrontRightDriverMotor.setPower(0);
        robot.FrontLeftDriverMotor.setPower(0);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
            telemetry.addData("Actual Pos L:R", "%7d:%7d", robot.FrontLeftDriverMotor.getCurrentPosition(), robot.FrontRightDriverMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    public void turnMecanum(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.FrontLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
        robot.FrontRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
        robot.BackLeftDriverMotor.setPower(leftSpeed * MULTIPLIER);
        robot.BackRightDriverMotor.setPower(rightSpeed * MULTIPLIER);
    }

    public void turnToHeadingMecanum(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            turnMecanum(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        turnMecanum(0, 0.0);
    }

    public void servoOpen(){
        robot.Box.setPosition(0.8);
    }
    public void servoClose(){
        robot.Box.setPosition(0.5);
    }
    public void lift_up(){
        robot.MotorLift.setVelocity(900);
        robot.MotorLift.setTargetPosition(-1200);
    }
    public void lift_down(){
        robot.MotorLift.setVelocity(900);
        robot.MotorLift.setTargetPosition(4);}
}// end class

