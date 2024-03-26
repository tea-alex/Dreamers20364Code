package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Config_robot {
    /*
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftBackDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightBackDrive = null;
    public DcMotorEx rightLift = null;
    public DcMotorEx leftLift = null;
    public DcMotorEx zahvat = null;
    public DcMotorEx conv = null;
    public Servo servoP = null;
    public Servo servoS = null;
*/
    HardwareMap hardM = null;
    BNO055IMU imu = null;

    public DcMotorEx FrontRightDriverMotor = null;
    public DcMotorEx BackRightDriverMotor = null;
    public DcMotorEx FrontLeftDriverMotor = null;
    public DcMotorEx BackLeftDriverMotor = null;

    public DcMotorEx MotorConveyer ;
    public DcMotorEx MotorLift = null;
    public Servo Box;
    public Servo Airplane;
    public DcMotorEx MotorSelfDestruction = null;

    public Config_robot(){

    }

    public void init_Tele(HardwareMap hMap){

        hardM = hMap;

        imu = hMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        Airplane = hardM.get(Servo.class,"S1");
        Box = hardM.get(Servo.class,"S2");
        FrontRightDriverMotor = hardM.get(DcMotorEx.class,"MotorFR");
        BackRightDriverMotor = hardM.get(DcMotorEx.class,"MotorBR");
        FrontLeftDriverMotor = hardM.get(DcMotorEx.class,"MotorFL");
        BackLeftDriverMotor = hardM.get(DcMotorEx.class,"MotorBL");
        MotorConveyer = hardM.get(DcMotorEx.class,"MotorC");
        MotorLift = hardM.get(DcMotorEx.class,"MotorL");
        MotorSelfDestruction = hardM.get(DcMotorEx.class,"MotorSD");

        FrontLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLift.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSelfDestruction.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSelfDestruction.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRightDriverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftDriverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftDriverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightDriverMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConveyer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setTargetPosition(0);
        MotorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorSelfDestruction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSelfDestruction.setTargetPosition(0);
        MotorSelfDestruction.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Box.setPosition(0.5 );
        Airplane.setPosition(0.);

    }

    public void init_drivetrain(HardwareMap hMap){
        hardM = hMap;


        FrontRightDriverMotor = hardM.get(DcMotorEx.class,"MotorFR");
        BackRightDriverMotor = hardM.get(DcMotorEx.class,"MotorBR");
        FrontLeftDriverMotor = hardM.get(DcMotorEx.class,"MotorFL");
        BackLeftDriverMotor = hardM.get(DcMotorEx.class,"MotorBL");

        FrontLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void init_Auto(HardwareMap hMap){

        hardM = hMap;
        imu = hMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        /*
        Box = hMap.get(Servo.class,"servo");

        FrontLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);

         */
/*
        rightLift = hardM.get(DcMotorEx.class, "rightL");
        leftLift = hardM.get(DcMotorEx.class, "leftL");
        zahvat = hardM.get(DcMotorEx.class, "zahvat");

 */
        Box = hardM.get(Servo.class,"S2");
        FrontRightDriverMotor = hardM.get(DcMotorEx.class,"MotorFR");
        BackRightDriverMotor = hardM.get(DcMotorEx.class,"MotorBR");
        FrontLeftDriverMotor = hardM.get(DcMotorEx.class,"MotorFL");
        BackLeftDriverMotor = hardM.get(DcMotorEx.class,"MotorBL");

        MotorConveyer = hardM.get(DcMotorEx.class,"MotorC");
        MotorLift = hardM.get(DcMotorEx.class,"MotorL");
        MotorSelfDestruction = hardM.get(DcMotorEx.class,"MotorSD");


        FrontLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftDriverMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightDriverMotor.setDirection(DcMotorSimple.Direction.FORWARD);
/*
        MotorLift.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSelfDestruction.setDirection(DcMotorSimple.Direction.REVERSE);
 */

        FrontRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightDriverMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeftDriverMotor.setTargetPosition(0);
        FrontRightDriverMotor.setTargetPosition(0);
        BackLeftDriverMotor.setTargetPosition(0);
        BackRightDriverMotor.setTargetPosition(0);

/*
        MotorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLift.setTargetPosition(0);
        MotorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorSelfDestruction.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSelfDestruction.setTargetPosition(0);
        MotorSelfDestruction.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 */

        Box.setPosition(0.5 );
//        Airplane.setPosition(0.);
    }
}
