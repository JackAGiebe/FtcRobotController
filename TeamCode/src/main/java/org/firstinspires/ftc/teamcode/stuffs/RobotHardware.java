package org.firstinspires.ftc.teamcode.stuffs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.stuffs.Constants;

public class RobotHardware {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor leftEncoder, rightEncoder, normalEncoder;
    public DcMotorEx shooter1, shooter2;
    //    public DcMotor shooter2;
    //    public DcMotor shooter1PreCast, shooter2;
//    public DcMotorEx shooter1;
    public DcMotor intake1, intake2;

    public Servo wobbleSecure, wobble1, wobble2;
    public Servo hopper1, hopper2;
    public Servo pusher;

    public BNO055IMU imu;

    LinearOpMode op;

    Constants constants = new Constants();

    HardwareMap hardwareMap = null;

    public String flName = "fl", frName = "fr", blName = "bl", brName = "br";
    public String shooter1Name = "shooter1", shooter2Name = "shooter2";
    public String intake1Name = "intake1", intake2Name = "intake2";
    public String wobbleSecureName = "wobbleSecure", wobble1Name = "wobble1", wobble2Name = "wobble2";
    public String hopper1Name = "hopper1", hopper2Name = "hopper2";
    public String pusherName = "pusher";

    //Constructor
    public RobotHardware(LinearOpMode op){
        this.op = op;
    }


    //init
    public void init(HardwareMap hardwareMap, boolean initServos){
        this.hardwareMap = hardwareMap;

        frontLeft = hardwareMap.get(DcMotor.class, flName);
        frontRight = hardwareMap.get(DcMotor.class, frName);
        backLeft = hardwareMap.get(DcMotor.class, blName);
        backRight = hardwareMap.get(DcMotor.class, brName);

        leftEncoder = hardwareMap.get(DcMotor.class, intake1Name);
        rightEncoder = hardwareMap.get(DcMotor.class, frName);
        normalEncoder = hardwareMap.get(DcMotor.class, flName);

//        shooter1PreCast = hardwareMap.get(DcMotor.class, shooter1Name);
        shooter1 = hardwareMap.get(DcMotorEx.class, shooter1Name);
        shooter2 = hardwareMap.get(DcMotorEx.class, shooter2Name);

        intake1 = hardwareMap.get(DcMotor.class, intake1Name);
        intake2 = hardwareMap.get(DcMotor.class, intake2Name);

        wobbleSecure = hardwareMap.get(Servo.class, wobbleSecureName);
        wobble1 = hardwareMap.get(Servo.class, wobble1Name);
        wobble2 = hardwareMap.get(Servo.class, wobble2Name);

        hopper1 = hardwareMap.get(Servo.class, hopper1Name);
        hopper2 = hardwareMap.get(Servo.class, hopper2Name);

        pusher = hardwareMap.get(Servo.class, pusherName);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        normalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        shooter1.setPower(0);
        shooter2.setPower(0);

        intake1.setPower(0);
        intake2.setPower(0);

//        shooter1 = (DcMotorEx) shooter1PreCast;
//
//        if(initIMU)
//            initIMU(hardwareMap);

        if(initServos)
            initServos();
    }

    //init imu except we don't
//    public void initIMU(HardwareMap hardwareMap){
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNo055IMUCalibration.json";
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }

    //init servos if we do that
    public void initServos(){
        wobbleSecure.setPosition(constants.wobbleSecureClosed);
        wobble1.setPosition(constants.wobble1Back);
        wobble2.setPosition(constants.wobble2Back);
        hopper1.setPosition(constants.hopper1Down);
        hopper2.setPosition(constants.hopper2Down);
        pusher.setPosition(constants.pusherOut);
    }

    public void pause(double seconds){
        ElapsedTime waitTime = new ElapsedTime();
        waitTime.reset();
        while(waitTime.seconds() < seconds && op.opModeIsActive() && !op.isStopRequested()){
            //do nothing
            //literally just wait
            //don't do shit until the time ends
            //yeet m8 xD lmao
        }
    }

//    public double currentVelocity, error;
//    public double cumulativeError, lastError = 0;
//    public double proportional, integral, derivative;
//    boolean killDerivative, killIntegral;
//    public double shooterPower;
//    public double p;
//    public void
//    setShooterVelocity(double velocity, double p, double i, double d){
//        this.p = p;
//        currentVelocity = shooter1.getVelocity();
//        error = velocity - currentVelocity;
//
//        if((lastError < velocity && error > velocity) || (lastError > velocity && error < velocity) || error == 0){
//            killIntegral = true;
//            killDerivative = true;
//        }
//        else{
//            killIntegral = false;
//            killDerivative = false;
//        }
//
//        if(Math.abs(error) > constants.integralActiveZone){
//            killIntegral = true;
//        }
//
//        proportional = error * p;
//
//        if(killIntegral)
//            cumulativeError = 0;
//        else
//            integral = cumulativeError * i;
//
//        if(!killDerivative)
//            derivative = (error - lastError) * d;
//
//        lastError = error;
//
//        shooterPower = proportional + integral + derivative;
//
//        shooter1.setPower(shooterPower);
//        shooter2.setPower(shooterPower);
//    }*/
}