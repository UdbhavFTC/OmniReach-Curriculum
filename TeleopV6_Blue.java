package org.firstinspires.ftc.teamcode.TeleopCodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Backends.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Backends.ColorSensorBackend;
import org.firstinspires.ftc.teamcode.RobotTeleop.Mecanum;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp (name = "Teleop V6 Blue")

public class TeleopV6_Blue extends OpMode {

    //For Shooting
    AprilTagWebcam aprilTagwebcam = new AprilTagWebcam();
    // Gains
    final double TURN_GAIN = 0.01;
    final double MAX_AUTO_TURN = 0.3;

    // Turret
    private DcMotor TurretBoy = null;

    // AprilTag / Camera
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // Runtime
    private boolean targetFound = false;
    private double turn = 0;

    // Only run exposure once
    private boolean exposureSet = false;

    Mecanum drive = new Mecanum();
    double forward, strafe, rotate;

    private ColorSensorBackend colorSensorBackend;

    //FSM CONSTANTS
    final double timerforshoot = 0.75;

    final double flickerdown = 0.5;

    final double small = 0.1;

    private enum Color_Sensor_GPP {
        TRIGGER_GPP,
        SHOOTGREEN_GPP,
        FLICKER1DOWN_GPP,
        BREAK1_GPP,
        SHOOTPURPLE1_GPP,
        FLICKER2DOWN_GPP,
        BREAK2_GPP,
        SHOOTPURPLE2_GPP,
        FLICKER3DOWN_GPP,
    }

    private enum Color_Sensor_PGP {
        TRIGGER_PGP,
        SHOOTPURPLE1_PGP,
        FLICKER1DOWN_PGP,
        BREAK1_PGP,
        SHOOTGREEN_PGP,
        FLICKER2DOWN_PGP,
        BREAK2_PGP,
        SHOOTPURPLE2_PGP,
        FLICKER3DOWN_PGP,
    }

    private enum Color_Sensor_PPG {
        TRIGGER_PPG,
        SHOOTPURPLE1_PPG,
        FLICKER1DOWN_PPG,
        BREAK1_PPG,
        SHOOTPURPLE2_PPG,
        FLICKER2DOWN_PPG,
        BREAK2_PPG,
        SHOOTGREEN_PPG,
        FLICKER3DOWN_PPG,
    }

    private enum DistanceSensorFSM {
        TRIGGER_DIST,
        FLICKER1_DIST,
        FLICKER1DOWN_DIST,
        BREAK1_DIST,
        FLICKER2_DIST,
        FLICKER2DOWN_DIST,
        BREAK2_DIST,
        FLICKER3_DIST,
        FLICKER3DOWN_DIST,
    }

    Servo Flicker1;
    Servo Flicker2;
    Servo Flicker3;
    Servo angleServo;

    DcMotor Intake;
    DcMotor leftShooter;


    private IMU imu;

    private int usedSensorGreen = 0;
    private int usedSensorPurple1 = 0;
    private int usedSensorPurple2 = 0;


    DistanceSensor distancesensor1;
    DistanceSensor distancesensor2;
    DistanceSensor distanceSensor3;
    DistanceSensor mounted_distance;

    // COLOR GPP

    Color_Sensor_GPP color_sensor_gpp = Color_Sensor_GPP.TRIGGER_GPP;
    ElapsedTime color_sensor_gpp_timer = new ElapsedTime();

    // COLOR PGP
    Color_Sensor_PGP color_sensor_pgp = Color_Sensor_PGP.TRIGGER_PGP;
    ElapsedTime color_sensor_pgp_timer = new ElapsedTime();

    // COLOR PPG
    Color_Sensor_PPG color_sensor_ppg = Color_Sensor_PPG.TRIGGER_PPG;
    ElapsedTime color_sensor_ppg_timer = new ElapsedTime();

    // DISTANCE FSM
    DistanceSensorFSM distanceSensorFSM = DistanceSensorFSM.TRIGGER_DIST;
    ElapsedTime distanceSensorFSM_timer = new ElapsedTime();


    double power;
    double shooterpower = 0.7;
    double step = 0.1;
    double servoinitialpos = 0;

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        if (gamepad1.a) {
            aprilTag.setDecimation(1);
        } else {
            aprilTag.setDecimation(3);
        }

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void flickSensor(int sensorId) {

        //COLOR SENSOR LOGIC
        switch (sensorId) {
            case 1:
                Flicker1.setPosition(0.695);
                break;
            case 2:
                Flicker2.setPosition(0.695);
                break;
            case 3:
                Flicker3.setPosition(0.695);
                break;
        }
    }


    @Override
    public void init() {
        initAprilTag();


        //Webcam Init
        aprilTagwebcam.init(hardwareMap, telemetry);

        // Turret motor init
        TurretBoy = hardwareMap.get(DcMotor.class, "TurretBoy");
        TurretBoy.setDirection(DcMotorSimple.Direction.FORWARD);
        TurretBoy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TurretBoy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Press PLAY to start");
        telemetry.update();


        angleServo = hardwareMap.get(Servo.class, "angleServo");

        //MECANUM
        drive.init(hardwareMap);

        Flicker1 = hardwareMap.get(Servo.class, "Flicker1");
        Flicker2 = hardwareMap.get(Servo.class, "Flicker2");
        Flicker3 = hardwareMap.get(Servo.class, "Flicker3");

        distancesensor1 = hardwareMap.get(DistanceSensor.class, "colorSensor1");
        distancesensor2 = hardwareMap.get(DistanceSensor.class, "colorSensor2");
        distanceSensor3 = hardwareMap.get(DistanceSensor.class, "colorSensor3");

        mounted_distance = hardwareMap.get(DistanceSensor.class,"mounted_distance");

        imu = hardwareMap.get(IMU.class, "imu");

        Flicker1.setDirection(Servo.Direction.REVERSE);
        Flicker2.setDirection(Servo.Direction.FORWARD);
        Flicker3.setDirection(Servo.Direction.REVERSE);

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);



        color_sensor_gpp_timer.reset();
        distanceSensorFSM_timer.reset();

        Flicker1.setPosition(0);
        Flicker2.setPosition(0);
        Flicker3.setPosition(0);

        colorSensorBackend = new ColorSensorBackend(hardwareMap);


    }


    @Override
    public void loop() {

        //Turret Power Change
        aprilTagwebcam.update();
        AprilTagDetection id20 = aprilTagwebcam.getTagBySpecificId(20);

            if (id20 != null && id20.ftcPose != null && gamepad2.dpad_down) {
                aprilTagwebcam.displayDetectionTelemetry(id20);
                double range = id20.ftcPose.range;
                telemetry.addData("range", range);
                telemetry.addLine("Shooting with camera input");
                leftShooter.setPower(Range.clip(-(5.63374 * Math.pow(10, -10) * Math.pow(range, 4)) + (3.7559 * Math.pow(10, -7) * Math.pow(range, 3)) - 0.0000869059 * Math.pow(range, 2) + 0.0096509 * range - 0.109979, 0, 1));
                angleServo.setPosition(Range.clip(-(4.02771 * Math.pow(10, -9) * Math.pow(range, 4)) + 0.00000311623 * Math.pow(range, 3) - 0.000837394 * Math.pow(range, 2) + 0.0940219 * range - 3.59764, 0, 0.55));
            }
            if (gamepad2.dpad_down) {
                double d_distance = mounted_distance.getDistance(DistanceUnit.CM);
                telemetry.addData("d_distance", d_distance);
                telemetry.addLine("Shooting with D-sensor input");
                leftShooter.setPower(Range.clip((-(5.63374 * Math.pow(10, -10) * Math.pow(d_distance, 4)) + (3.7559 * Math.pow(10, -7) * Math.pow(d_distance, 3)) - 0.0000869059 * Math.pow(d_distance, 2) + 0.0096509 * d_distance - 0.109979) * 1.2, 0, 1));
                angleServo.setPosition(Range.clip((-(4.02771 * Math.pow(10, -9) * Math.pow(d_distance, 4)) + 0.00000311623 * Math.pow(d_distance, 3) - 0.000837394 * Math.pow(d_distance, 2) + 0.0940219 * d_distance - 3.59764) * 1.2, 0, 0.55));
            }
            if (!gamepad2.dpad_down) {
                telemetry.addLine("Not shooting");
                leftShooter.setPower(0);
                angleServo.setPosition(0);
            }


        telemetry.addData("Motorpower", shooterpower);

        //MECANUM
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.fieldCentric(forward, strafe, rotate);


        if (gamepad1.right_trigger > 0.1) {
            power = gamepad1.right_trigger; // Forward
        } else if (gamepad1.left_trigger > 0.1) {
            power = -gamepad1.left_trigger; // Reverse
        } else {
            power = 0.0;
        }

        // Apply power to the
        Intake.setPower(power);

        //COLOR SENSOR LOGIC
        if (gamepad2.a) {
            runcolorgpp();
        }

        if (gamepad2.b) {
            runcolorppg();
        }

        if (gamepad2.x){
            runcolorpgp();
        }

        if (gamepad2.y){
            rundistancefsm();
        }

        if (gamepad2.dpad_right){
            Flicker1.setPosition(0);
            Flicker2.setPosition(0);
            Flicker3.setPosition(0);
            color_sensor_gpp_timer.reset();
            color_sensor_gpp = Color_Sensor_GPP.TRIGGER_GPP;
            color_sensor_pgp_timer.reset();
            color_sensor_pgp = Color_Sensor_PGP.TRIGGER_PGP;
            color_sensor_ppg_timer.reset();
            color_sensor_ppg = Color_Sensor_PPG.TRIGGER_PPG;
            distanceSensorFSM_timer.reset();
            distanceSensorFSM = DistanceSensorFSM.TRIGGER_DIST;
        }

        if (gamepad1.dpad_down) {
            imu.resetYaw();
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
        telemetry.update();

        // Run manual exposure one time after stream starts
        if (USE_WEBCAM && !exposureSet) {
            setManualExposure(6, 250);
            exposureSet = true;
        }

        targetFound = false;
        desiredTag = null;

        // Look through detections
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                if ((id20 != null && id20.ftcPose != null )) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Telemetry + compute turret turn power
        if (targetFound) {
            telemetry.addData("\n>", "Turret Bayya is aiming\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

            double headingError = desiredTag.ftcPose.bearing;
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            telemetry.addData("Status", "No tag detected");
            turn = 0; // Hold
        }

        telemetry.addData("Turret Power", "%.2f", turn);
        telemetry.update();

        // Apply power
        MoveTurret(turn);
    }

    public void MoveTurret(double turn) {
        TurretBoy.setPower(turn);
    }
    private void setManualExposure(int exposureMS, int gain) {

        if (visionPortal == null) return;

        // Only try if streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Not streaming yet");
            telemetry.update();
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null) {
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        }

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(gain);


        }
    }

    public void runcolorgpp(){

        ColorSensorBackend.SensorResult[] sensors = colorSensorBackend.getDetectedColor(telemetry);

        switch (color_sensor_gpp) {
            case TRIGGER_GPP:
                    color_sensor_gpp_timer.reset();
                    color_sensor_gpp = Color_Sensor_GPP.SHOOTGREEN_GPP;
                break;
            case SHOOTGREEN_GPP:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.GREEN_DIRECT || sensor.color == ColorSensorBackend.detectedColor.GREEN_INDIRECT) {
                        int sensorId = sensor.sensorId;
                        flickSensor(sensorId);
                        usedSensorGreen = sensor.sensorId;
                        color_sensor_gpp_timer.reset();
                        color_sensor_gpp = Color_Sensor_GPP.FLICKER1DOWN_GPP;
                        break;
                    }
                }
                break;
            case FLICKER1DOWN_GPP:
                if (color_sensor_gpp_timer.seconds() >= flickerdown) {
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_gpp_timer.reset();
                    color_sensor_gpp = Color_Sensor_GPP.BREAK1_GPP;
                }
                break;
            case BREAK1_GPP:
                if (color_sensor_gpp_timer.seconds() >= timerforshoot) {
                    Flicker1.setPosition(0);
                    color_sensor_gpp_timer.reset();
                    color_sensor_gpp = Color_Sensor_GPP.SHOOTPURPLE1_GPP;
                }
                break;
            case SHOOTPURPLE1_GPP:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.PURPLE_DIRECT || sensor.color == ColorSensorBackend.detectedColor.PURPLE_INDIRECT &&
                            sensor.sensorId != usedSensorGreen) {
                        int sensorId = sensor.sensorId;
                        usedSensorPurple1 = sensorId;
                        flickSensor(sensorId);
                        color_sensor_gpp_timer.reset();
                        color_sensor_gpp = Color_Sensor_GPP.FLICKER2DOWN_GPP;
                        break;
                    }
                }
                break;
            case FLICKER2DOWN_GPP:
                if (color_sensor_gpp_timer.seconds() >= flickerdown) {
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_gpp_timer.reset();
                    color_sensor_gpp = Color_Sensor_GPP.BREAK2_GPP;
                }
                break;
            case BREAK2_GPP:
                if (color_sensor_gpp_timer.seconds() >= timerforshoot) {
                    color_sensor_gpp_timer.reset();
                    color_sensor_gpp = Color_Sensor_GPP.SHOOTPURPLE2_GPP;
                }
                break;
            case SHOOTPURPLE2_GPP:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.PURPLE_DIRECT || sensor.color == ColorSensorBackend.detectedColor.PURPLE_INDIRECT &&
                            sensor.sensorId != usedSensorGreen &&
                            sensor.sensorId != usedSensorPurple1) {
                        int sensorId = sensor.sensorId;
                        flickSensor(sensorId);
                        color_sensor_gpp_timer.reset();
                        color_sensor_gpp = Color_Sensor_GPP.FLICKER3DOWN_GPP;
                        break;
                    }
                }
                break;
            case FLICKER3DOWN_GPP:
                if (color_sensor_gpp_timer.seconds() >= flickerdown) {
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_gpp_timer.reset();
                    color_sensor_gpp = Color_Sensor_GPP.TRIGGER_GPP;
                }
                break;

        }

    }

    public void runcolorpgp(){
        ColorSensorBackend.SensorResult[] sensors = colorSensorBackend.getDetectedColor(telemetry);

        switch (color_sensor_pgp) {
            case TRIGGER_PGP:
                    color_sensor_pgp_timer.reset();
                    color_sensor_pgp = Color_Sensor_PGP.SHOOTPURPLE1_PGP;
                break;
            case SHOOTPURPLE1_PGP:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.PURPLE_DIRECT || sensor.color == ColorSensorBackend.detectedColor.PURPLE_INDIRECT) {
                        int sensorId = sensor.sensorId;
                        flickSensor(sensorId);
                        usedSensorPurple1 = sensor.sensorId;
                        color_sensor_pgp_timer.reset();
                        color_sensor_pgp = Color_Sensor_PGP.FLICKER1DOWN_PGP;
                        break;
                    }
                } break;
            case FLICKER1DOWN_PGP:
                if (color_sensor_pgp_timer.seconds() >= flickerdown){
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_pgp_timer.reset();
                    color_sensor_pgp = Color_Sensor_PGP.BREAK1_PGP;
                } break;
            case BREAK1_PGP:
                if (color_sensor_pgp_timer.seconds() >= timerforshoot) {
                    color_sensor_pgp_timer.reset();
                    color_sensor_pgp = Color_Sensor_PGP.SHOOTGREEN_PGP;
                    break;
                }
            case SHOOTGREEN_PGP:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.GREEN_DIRECT || sensor.color == ColorSensorBackend.detectedColor.GREEN_INDIRECT &&
                            sensor.sensorId != usedSensorPurple1) {
                        int sensorId = sensor.sensorId;
                        flickSensor(sensorId);
                        usedSensorGreen = sensor.sensorId;
                        color_sensor_pgp_timer.reset();
                        color_sensor_pgp = Color_Sensor_PGP.FLICKER2DOWN_PGP;
                        break;
                    }
                } break;
            case FLICKER2DOWN_PGP:
                if (color_sensor_pgp_timer.seconds() >= flickerdown) {
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_pgp_timer.reset();
                    color_sensor_pgp = Color_Sensor_PGP.BREAK2_PGP;
                } break;
            case BREAK2_PGP:
                if (color_sensor_pgp_timer.seconds() >= timerforshoot){
                    color_sensor_pgp_timer.reset();
                    color_sensor_pgp = Color_Sensor_PGP.SHOOTPURPLE2_PGP;
                    break;
                }
            case SHOOTPURPLE2_PGP:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.PURPLE_DIRECT || sensor.color == ColorSensorBackend.detectedColor.PURPLE_INDIRECT &&
                            sensor.sensorId != usedSensorGreen &&
                            sensor.sensorId != usedSensorPurple1) {
                        int sensorId = sensor.sensorId;
                        flickSensor(sensorId);
                        color_sensor_pgp_timer.reset();
                        color_sensor_pgp = Color_Sensor_PGP.FLICKER3DOWN_PGP;
                        break;
                    }
                } break;
            case FLICKER3DOWN_PGP:
                if (color_sensor_pgp_timer.seconds() >= flickerdown) {
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_pgp_timer.reset();
                    color_sensor_pgp = Color_Sensor_PGP.TRIGGER_PGP;
                } break;

        }


    }

    public void runcolorppg(){
        ColorSensorBackend.SensorResult[] sensors = colorSensorBackend.getDetectedColor(telemetry);

        switch (color_sensor_ppg) {
            case TRIGGER_PPG:
                    color_sensor_ppg_timer.reset();
                    color_sensor_ppg = Color_Sensor_PPG.SHOOTPURPLE1_PPG;
                 break;
            case SHOOTPURPLE1_PPG:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.PURPLE_DIRECT || sensor.color == ColorSensorBackend.detectedColor.PURPLE_INDIRECT) {
                        int sensorId = sensor.sensorId;
                        usedSensorPurple1 = sensorId;
                        flickSensor(sensorId);
                        color_sensor_ppg_timer.reset();
                        color_sensor_ppg = Color_Sensor_PPG.FLICKER1DOWN_PPG;
                        break;
                    }
                } break;
            case FLICKER1DOWN_PPG:
                if (color_sensor_ppg_timer.seconds() >= flickerdown){
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_ppg_timer.reset();
                    color_sensor_ppg = Color_Sensor_PPG.BREAK1_PPG;
                } break;
            case BREAK1_PPG:
                if (color_sensor_ppg_timer.seconds() >= timerforshoot) {
                    color_sensor_ppg_timer.reset();
                    color_sensor_ppg = Color_Sensor_PPG.SHOOTPURPLE2_PPG;
                } break;
            case SHOOTPURPLE2_PPG:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.PURPLE_DIRECT || sensor.color == ColorSensorBackend.detectedColor.PURPLE_INDIRECT &&
                            sensor.sensorId != usedSensorPurple1) {
                        int sensorId = sensor.sensorId;
                        usedSensorPurple2 = sensorId;
                        flickSensor(sensorId);
                        color_sensor_ppg_timer.reset();
                        color_sensor_ppg = Color_Sensor_PPG.FLICKER2DOWN_PPG;
                        break;
                    }
                } break;
            case FLICKER2DOWN_PPG:
                if (color_sensor_ppg_timer.seconds() >= flickerdown){
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_ppg_timer.reset();
                    color_sensor_ppg = Color_Sensor_PPG.BREAK2_PPG;
                } break;
            case BREAK2_PPG:
                if (color_sensor_ppg_timer.seconds() >= timerforshoot) {
                    color_sensor_ppg_timer.reset();
                    color_sensor_ppg = Color_Sensor_PPG.SHOOTGREEN_PPG;
                }
                break;
            case SHOOTGREEN_PPG:
                for (ColorSensorBackend.SensorResult sensor : sensors) {
                    if (sensor.color == ColorSensorBackend.detectedColor.GREEN_DIRECT || sensor.color == ColorSensorBackend.detectedColor.GREEN_INDIRECT &&
                            sensor.sensorId != usedSensorPurple1 &&
                            sensor.sensorId != usedSensorPurple2) {
                        int sensorId = sensor.sensorId;
                        flickSensor(sensorId);
                        color_sensor_ppg_timer.reset();
                        color_sensor_ppg = Color_Sensor_PPG.FLICKER3DOWN_PPG;
                        break;
                    }
                }
                break;
            case FLICKER3DOWN_PPG:
                if (color_sensor_ppg_timer.seconds() >= flickerdown){
                    Flicker1.setPosition(0);
                    Flicker2.setPosition(0);
                    Flicker3.setPosition(0);
                    color_sensor_ppg_timer.reset();
                    color_sensor_ppg = Color_Sensor_PPG.TRIGGER_PPG;
                } break;

        }
    }
    public void rundistancefsm(){
        double distance1 = distancesensor1.getDistance(DistanceUnit.CM);
        double distance2 = distancesensor2.getDistance(DistanceUnit.CM);
        double distance3 = distanceSensor3.getDistance(DistanceUnit.CM);


        switch (distanceSensorFSM) {
            case TRIGGER_DIST:
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.FLICKER1_DIST;
                break;
            case FLICKER1_DIST:
                if (distance1 < 6 || distance2 < 6 || distance3 < 6) {
                    Flicker1.setPosition(0.695);
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.FLICKER1DOWN_DIST;
                }
                break;
            case FLICKER1DOWN_DIST:
                if (distanceSensorFSM_timer.seconds() > flickerdown) {
                    Flicker1.setPosition(0);
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.BREAK1_DIST;
                }
                break;
            case BREAK1_DIST:
                if (distanceSensorFSM_timer.seconds() > timerforshoot) {
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.FLICKER2_DIST;
                }
                break;
            case FLICKER2_DIST:
                if (distance1 < 6 || distance2 < 6 || distance3 < 6) {
                    Flicker2.setPosition(0.695);
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.FLICKER2DOWN_DIST;
                }
                break;
            case FLICKER2DOWN_DIST:
                if (distanceSensorFSM_timer.seconds() > flickerdown) {
                    Flicker2.setPosition(0);
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.BREAK2_DIST;
                }
                break;
            case BREAK2_DIST:
                if (distanceSensorFSM_timer.seconds() > timerforshoot) {
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.FLICKER3_DIST;
                }
                break;
            case FLICKER3_DIST:
                if (distance1 < 6 || distance2 < 6 || distance3 < 6) {
                    Flicker3.setPosition(0.695);
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.FLICKER3DOWN_DIST;
                }
                break;
            case FLICKER3DOWN_DIST:
                if (distanceSensorFSM_timer.seconds() > flickerdown) {
                    Flicker3.setPosition(0);
                    distanceSensorFSM_timer.reset();
                    distanceSensorFSM = DistanceSensorFSM.TRIGGER_DIST;
                }
                break;
        }
    }
}







