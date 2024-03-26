package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="OpenCV_Contour_RED", group="Auto")


public class OpenCV_Contour_3954_Test extends LinearOpMode {

    Config_robot robot = new Config_robot();
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 720;

    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    public static double borderLeftX    = 0.0;   //доля пикселей с левой стороны камеры, которую нужно пропустить
    public static double borderRightX   = 0.0;   //доля пикселей справа от камеры, которую нужно пропустить
    public static double borderTopY     = 0.0;   //долю пикселей от верхней части камеры, которую нужно пропустить
    public static double borderBottomY  = 0.0;   //долю пикселей от нижней части камеры, которую нужно пропустить

    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 10.0;     // Для определения окружности
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

//    private BNO055IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * Это будет вызвано, если камеру невозможно открыть
                 */
            }
        });

        telemetry.update();
        robot.init_Auto(hardwareMap);
        waitForStart();

  //      while (opModeIsActive()) {
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }
        // Используйте эту строку кода только тогда, когда вы хотите найти нижнее и верхнее значения
//        testing(myPipeline);

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();

            if (myPipeline.getRectArea() > 2000) {
                if (myPipeline.getRectMidpointX() > 1066) {
                    AUTONOMOUS_RIGHT(); // right
                } else if (myPipeline.getRectMidpointX() < 213) {
                    AUTONOMOUS_LEFT(); // left
                } else {
                    AUTONOMOUS_CENTER();
                }
            }
 //       }
    }

    public void testing(ContourPipeline myPipeline) {
        if (lowerruntime + 0.05 < getRuntime()) {
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        if (upperruntime + 0.05 < getRuntime()) {
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }

        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);

        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);

        telemetry.addData("lowerCr ", (int) CrLowerUpdate);
        telemetry.addData("lowerCb ", (int) CbLowerUpdate);
        telemetry.addData("UpperCr ", (int) CrUpperUpdate);
        telemetry.addData("UpperCb ", (int) CbUpperUpdate);
    }

    public Double inValues(double value, double min, double max) {
        if (value < min) {
            value = min;
        }
        if (value > max) {
            value = max;
        }
        return value;
    }

    public void AUTONOMOUS_CENTER() {
        telemetry.addLine("Center");

        encoderDrive(0.3, -29, -29, 5);
        sleep(500);
        servoOpen();
        sleep(500);
        encoderDrive(0.3, 10, 10, 5);
        sleep(500);

        off_motor();
    }

    public void AUTONOMOUS_LEFT() {
        telemetry.addLine("Auto Left");
        encoderDrive(0.3, -26, -26, 5);
        sleep(500);

        encoderDrive(0.3, -20, 20, 5);
        sleep(500);
        encoderDrive(0.3, -3.5, -3.5, 5);
        sleep(500);
        servoOpen();
        sleep(500);
        encoderDrive(0.3, 5, 5, 5);
        sleep(500);
        off_motor();
    }

    public void AUTONOMOUS_RIGHT() {
        telemetry.addLine("Auto Right");
        encoderDrive(0.3, -26, -26, 5);
        sleep(500);
        encoderDrive(0.3, 20, -20, 5);
        sleep(500);
        encoderDrive(0.3, -3, -3, 5);
        sleep(500);
        servoOpen();
        sleep(500);
        encoderDrive(0.3, 5, 5, 5);
        sleep(500);
        off_motor();
    }

    public void servoOpen(){
        robot.Box.setPosition(0.8);
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (opModeIsActive()) {

            newLeftFrontTarget = robot.FrontLeftDriverMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.FrontRightDriverMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            newLeftBackTarget = robot.BackLeftDriverMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.BackRightDriverMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.FrontLeftDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.FrontRightDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.BackLeftDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.BackRightDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            robot.FrontLeftDriverMotor.setTargetPosition(newLeftFrontTarget);
            robot.FrontRightDriverMotor.setTargetPosition(newRightFrontTarget);
            robot.BackLeftDriverMotor.setTargetPosition(newLeftBackTarget);
            robot.BackRightDriverMotor.setTargetPosition(newRightBackTarget);
/*
            robot.FrontLeftDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.FrontRightDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.BackLeftDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.BackRightDriverMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

 */

            runtime.reset();
            robot.FrontLeftDriverMotor.setPower(Math.abs(speed));
            robot.FrontRightDriverMotor.setPower(Math.abs(speed));

            robot.BackLeftDriverMotor.setPower(Math.abs(speed));
            robot.BackRightDriverMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FrontLeftDriverMotor.isBusy() && robot.FrontRightDriverMotor.isBusy()))
                 {

                telemetry.addData("Running to", " %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        robot.FrontLeftDriverMotor.getCurrentPosition(), robot.FrontRightDriverMotor.getCurrentPosition());
                telemetry.update();
            }

            robot.FrontLeftDriverMotor.setPower(0);
            robot.FrontRightDriverMotor.setPower(0);

            robot.BackLeftDriverMotor.setPower(0);
            robot.BackRightDriverMotor.setPower(0);

            robot.FrontLeftDriverMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.FrontRightDriverMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.BackLeftDriverMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.BackRightDriverMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
    void off_motor(){
        robot.BackLeftDriverMotor.setPower(0);
        robot.BackRightDriverMotor.setPower(0);
        robot.FrontRightDriverMotor.setPower(0);
        robot.FrontLeftDriverMotor.setPower(0);
    }

}

