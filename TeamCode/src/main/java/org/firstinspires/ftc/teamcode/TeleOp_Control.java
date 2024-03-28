package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "TeleOp")
//@Config
public class TeleOp_Control extends OpMode {
//    FtcDashboard dashboard = FtcDashboard.getInstance();
    Config_robot robot = new Config_robot();
//    public OnServo launcher = new OnServo();
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

    public static double MULTIPLIER = 0.9;

    public void init(){
        robot.init_Tele(hardwareMap);
    }

    public void start(){
    }

    public void loop(){

        double max;
        double PowerFirst = gamepad2.left_trigger;
        double PowerSecond = -gamepad2.right_trigger;


        double axial   =  - gamepad1.left_stick_y;
        double lateral =  gamepad1.right_stick_x;
        double yaw     =  gamepad1.left_stick_x;


        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }



        robot.FrontLeftDriverMotor.setPower(leftFrontPower);
        robot.FrontRightDriverMotor.setPower(rightFrontPower);
        robot.BackRightDriverMotor.setPower(leftBackPower);
        robot.BackLeftDriverMotor.setPower(rightBackPower);

        robot.MotorConveyer.setPower(PowerFirst + PowerSecond);

/*
        if (robot.digitalTouch.getState() == false) {
            telemetry.addData("Button", "PRESSED");

            robot.MotorLift.setVelocity(900);
            robot.MotorLift.setTargetPosition(1200);


        }
*/
        telemetry.update();

        if(gamepad1.dpad_up){
            slow();
            //podves_up();
            /*
            robot.MotorSelfDestruction.setVelocity(700);
            robot.MotorSelfDestruction.setTargetPosition(-10);//(robot.MotorSelfDestruction.getTargetPosition() + 50);
             */
        }

        if(gamepad1.a){
            turnToHeadingMecanum(0.9, -90);
        }
        if(gamepad1.b){
            turnToHeadingMecanum(0.9, 180);
        }
        if(gamepad1.x){
            turnToHeadingMecanum(0.9, 0);
        }
        if(gamepad1.y){
            turnToHeadingMecanum(0.9, 90);
        }


        if(gamepad2.dpad_up){
            robot.MotorLift.setVelocity(900);
            robot.MotorLift.setTargetPosition(robot.MotorLift.getTargetPosition() - 70);
        }
        if(gamepad2.dpad_down){
            robot.MotorLift.setVelocity(900);
            robot.MotorLift.setTargetPosition(robot.MotorLift.getTargetPosition() + 70);
        }
        if(gamepad2.a){
            high_position();
        }
        if(gamepad2.y){
            zero_position();
        }

        if (gamepad2.dpad_left) {
            robot.Box.setPosition(0.8); //open
        }
        if (gamepad2.dpad_right) {
            robot.Box.setPosition(0.5); //close
        }
        if (gamepad2.left_bumper) {
            robot.Airplane.setPosition(1);
        }
        telemetry.addData(":Robot is ","Running");
        telemetry.addData("","  ");
        telemetry.addData(" ","  ");
        telemetry.addData(":MotorLift Position:",robot.MotorLift.getTargetPosition());
        telemetry.addData(":Motor Self Destration Position:",robot.MotorSelfDestruction.getTargetPosition());


    }



/*
        if (gamepad1.left_stick_button) {
            robot.MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.MotorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

 */
/*
        telemetry.addData("Position rightL: ", robot.rightLift.getCurrentPosition());
        telemetry.addData("Target rightL:",robot.rightLift.getTargetPosition());
        telemetry.addData("Position leftL: ", robot.leftLift.getCurrentPosition());
        telemetry.addData("Target leftL:",robot.leftLift.getTargetPosition());
//        telemetry.addData("Position up: ", robot.leftLift.getCurrentPosition());
        telemetry.update();

 */


    public void stop(){
        robot.MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void stop_lift(){}

    void high_position(){
        robot.MotorLift.setVelocity(900);
        robot.MotorLift.setTargetPosition(1200);
    }

    void zero_position(){
        robot.MotorLift.setVelocity(900);
        robot.MotorLift.setTargetPosition(5);

    }


    void slow(){
        robot.BackLeftDriverMotor.setPower(0.3);
        robot.BackRightDriverMotor.setPower(0.3);
        robot.FrontLeftDriverMotor.setPower(0.3);
        robot.FrontRightDriverMotor.setPower(0.3);
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
        while ((Math.abs(headingError) > HEADING_THRESHOLD)) {//(opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

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


//    class OnServo extends  Thread{
//        boolean launch = false;
//        public void run(){
//            while (!isInterrupted()){
//                if(launch){
//                    if (gamepad1.right_bumper) {
//                        robot.servoP.setPosition(0.5);
//                        telemetry.addData(">", "close");
//                        telemetry.update();
//                    }
//                    if (gamepad1.left_bumper){
//                        robot.servoP.setPosition(1);
//                        telemetry.addData(">", "open");
//                        telemetry.update();
//                    }
//                    if(gamepad1.dpad_right){
//                        robot.servoS.setPosition(0.5);
//                    }
//                }
//            }
//        }
//}
}
