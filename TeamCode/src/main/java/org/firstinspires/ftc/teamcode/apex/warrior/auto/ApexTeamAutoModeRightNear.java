package org.firstinspires.ftc.teamcode.apex.warrior.auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigurator;

import java.util.Locale;

// Apex Team main Autonomous Mode
@Autonomous(name = "Auto Drive - Red Team - Near to goal", group = "Auto Opmode")
public class ApexTeamAutoModeRightNear extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    RobotHardwareConfigurator myRobotHW = new RobotHardwareConfigurator();
    private static double shooterDCMotorPowerScaleFactor = 0.395;
    private static final double transferDCMotorPowerScale = 0.75;
    private static final double intakeDCMotorPowerScale = 0.9;

    // Servo positions (0.0 to 1.0) Rotator Assembly
    private static final double SHOOTER_ROTATOR_SERVO_START_POS = 0.35;  //0.37 to 0.43 starting position

    private static final double SHOOTER_ROTATOR_SERVO_END_POS = 0.43;  // 150° away from start

    private static final double STEP = 0.01;  // // Step rate controls how fast the servo moves (smaller = slower)
    private final double FEEDER_SERVO_POS_0_DEG = 0.0;   // represents 0 degrees
    private final double FEEDER_SERVO_POS_120_DEG = 0.07; // approx 120 degrees on a 5-turn servo
    private final double STOPPER_SERVO_POS_INITIAL_GATE = 0.55;
    private final double STOPPER_SERVO_POS_OPEN_GATE = 0.70;
    private boolean stopperButtonStateForFirstTime = false;

    private DcMotor frontLeftChassisDC;
    private DcMotor frontRightChassisDC;
    private DcMotor backLeftChassisDC;
    private DcMotor backRightChassisDC;
    private DcMotor leftShooterDC;
    private DcMotor rightShooterDC;
    private DcMotor transferMechDC;
    private DcMotor intakeMechDC;
    private Servo shooterMechRotatorServo;
    private Servo feederEnablerServo;
    private Servo specStopperServo;

    private boolean lastLeftTriggerState = false;
    private boolean lastRightTriggerState = false;
    private double rotorServoCurrentPosition;

    ElapsedTime timer = new ElapsedTime();

    PIDController xPID = new PIDController(0.01, 0.01, 0.0);
    PIDController yPID = new PIDController(0.01, 0.01, 0.0);
    PIDController thetaPID = new PIDController(0.015, 0.01, 0.0);

    @Override
    public void runOpMode() throws InterruptedException {
        myRobotHW.configureRobotHardware(hardwareMap);
        initMotorAndServo();
        //odo = myRobotHW.getOdo();
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(19.8945588, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

//        telemetry.addData("Status", "HW Configuration Mapping Initialized");
//        telemetry.update();
//
//        telemetry.addData("Status", "HW Configuration Initialized");
//        telemetry.update();
//
//        telemetry.addData("Status", "Actuators Initialized");
//        telemetry.update();
//
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
        double battaryVoltage = getBatteryVoltage();

        telemetry.addData("Battery Voltage", "%.2f volts", battaryVoltage);
//        telemetry.update();
        shooterDCMotorPowerScaleFactor = calculateShooterFactor(battaryVoltage);
        telemetry.addData("Shooting factor", "%.2f volts", shooterDCMotorPowerScaleFactor);
        telemetry.update();

        waitForStart();
        telemetry.log().clear();

        frontRightChassisDC.setDirection(REVERSE);
        backRightChassisDC.setDirection(REVERSE);

        shootingAnglePositioner();

        /// //////////////////////////////////////////////
//        Thread controlThread = new Thread(() -> controlLoop(targetXMeters, targetHeadingDeg), "ControlLoop");
        Thread controlThread = new Thread(this::controlLoop, "ControlLoop");
        Thread shooterThread = new Thread(this::shooterLoop, "ShooterLoop");

        controlThread.start();
        shooterThread.start();

        // Main thread just watches for stop and keeps telemetry alive
        while (opModeIsActive() && controlThread.isAlive()) {
            telemetry.addLine("Autonomous running...");
            telemetry.update();
            sleep(50);
        }

        // Signal threads to stop (if not already)
//        shouldStop.set(true);
        controlThread.interrupt();
        shooterThread.interrupt();

        try {
            controlThread.join(300);
            shooterThread.join(300);
        } catch (InterruptedException ignored) {
            stopAllDCMotors();
        }
        telemetry.addLine("Autonomous complete.");
        telemetry.update();
    }

    public void controlLoop() {
        long lastTime = System.currentTimeMillis();

        long time_phase2 = System.currentTimeMillis();;

        while (opModeIsActive()) {
            specStopperServo.setPosition(STOPPER_SERVO_POS_INITIAL_GATE);
            odo.update();
            Pose2D curPos = odo.getPosition();

            double x_in_botAxis = curPos.getX(DistanceUnit.MM);
            double y_in_botAxis = curPos.getY(DistanceUnit.MM);
            double heading_deg = curPos.getHeading(AngleUnit.DEGREES);

            double theta = Math.toRadians(heading_deg); // convert to radians

            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", x_in_botAxis,
                    y_in_botAxis, heading_deg);
            telemetry.addData("Position", data);
            Pose2d target;
            target = new Pose2d(1320, 0.0, 2.0);

            long curTime = System.currentTimeMillis();
            if (curTime - time_phase2 > 9000 ) {
                target = new Pose2d(1350, 0.0, 132);
            }
            if (curTime - time_phase2 > 12000 ) {
                target = new Pose2d(710, 850, 132);
            }
            if (curTime - time_phase2 > 16000 ) {
                target = new Pose2d(1350, 0.0, 2.0);
            }
            if (curTime - time_phase2 > 22000 ) {
                target = new Pose2d(1320, 0.0, 80);
            }
            if (curTime - time_phase2 > 25000 ) {
                target = new Pose2d(1350, 1000, 135);
            }
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastTime) / 1e3;
            lastTime = currentTime;

            data = String.format(Locale.US, "{dt: %.3f}", dt);
            telemetry.addData("dt: ", data);

            double errorX = target.x - curPos.getX(DistanceUnit.MM);
            double errorY = target.y - curPos.getY(DistanceUnit.MM);
            double errorTheta = target.heading - curPos.getHeading(AngleUnit.DEGREES);

            double vx = xPID.update(errorX, dt);
            double vy = yPID.update(errorY, dt);
            double omega = thetaPID.update(errorTheta, dt);

            if(vx > 0.2)
                vx = 0.2;
            if(vx < -0.2)
                vx = -0.2;

            if(vy > 0.2)
                vy = 0.2;
            if(vy < -0.2)
                vy = -0.2;

            if(omega > 0.2)
                omega = 0.2;
            if(omega < -0.2)
                omega = -0.2;

            data = String.format(Locale.US, "{Vx: %.3f, Vy: %.3f, Heading Cmd: %.3f}", vx,
                    vy, omega);
            telemetry.addData("Control command: ", data);
            telemetry.update();

            double compensatedVx = vx * Math.cos(theta) + vy * Math.sin(theta);
            double compensatedVy = -vx * Math.sin(theta) + vy * Math.cos(theta);

            vx = compensatedVx;
            vy = compensatedVy;

//            vx = 0.0; // 0.
//            vy = 0.0; // 0.5
//            omega = 0.0; // 0.35

            double fl = -vx + vy + omega;
            double fr = -vx - vy - omega;
            double bl = -vx - vy + omega;
            double br = -vx + vy - omega;
            frontLeftChassisDC.setPower(fl);
            frontRightChassisDC.setPower(fr);
            backLeftChassisDC.setPower(bl);
            backRightChassisDC.setPower(br);
        }
        telemetry.addData("Status", "Stopping Motors");
        stopAllDCMotors();
    }
    public void shooterLoop() {
        shootingAnglePositioner();
        runShooterDCMotors();

        long start_time = System.currentTimeMillis();
        while(opModeIsActive()) {

            long curTime = System.currentTimeMillis();

            if ((curTime - start_time > 5000 && curTime - start_time < 9000) ||
                    (curTime - start_time > 19000 && curTime - start_time < 22000)) {
                startShooterDCMotors();
                runIntakeMechDCMotor();
                runTransferMechDCMotor();
                operateFeederServo();
            }
            else {
                specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
                feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);
                sleep(200);
            }
        }
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        return result;
    }
    private double calculateShooterFactor(double batteryVoltage) {
        double slope = -0.042857142857;     // derived from calibration
        double intercept = 0.96;  // derived from calibration

        double factor = slope * batteryVoltage + intercept;

        // Optional: clamp to safe range
        return Math.max(0.0, Math.min(1.0, factor));
    }
    private void initMotorAndServo() {
        initiateChassisDCMotors();
        initiateOtherDCMotors();
        initiateServoMotors();
    }

    private void initiateChassisDCMotors() {
        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Started");

        backLeftChassisDC = myRobotHW.getBackLeftChassisDC();
        backRightChassisDC = myRobotHW.getBackRightChassisDC();
        frontLeftChassisDC = myRobotHW.getFrontLeftChassisDC();
        frontRightChassisDC = myRobotHW.getFrontRightChassisDC();

        backLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBL", backLeftChassisDC.getDirection());
        backLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBL:", backLeftChassisDC.getCurrentPosition());
        backLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionBR", backRightChassisDC.getDirection());
        backRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeBR:", backRightChassisDC.getCurrentPosition());
        backRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFL", frontLeftChassisDC.getDirection());
        frontLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFL:", frontLeftChassisDC.getCurrentPosition());
        frontLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionFR", frontRightChassisDC.getDirection());
        frontRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("DcMotor ModeFR:", frontRightChassisDC.getCurrentPosition());
        frontRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Completed");
        telemetry.update();
    }

    private void initiateOtherDCMotors() {
        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Started");

        leftShooterDC = myRobotHW.getLeftShooterDC();
        rightShooterDC = myRobotHW.getRightShooterDC();
        transferMechDC = myRobotHW.getTransferMechDC();
        intakeMechDC = myRobotHW.getIntakeMechDC();

        leftShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
        leftShooterDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeLeftShooter:", leftShooterDC.getCurrentPosition());


        rightShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
        rightShooterDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeRightShooter:", rightShooterDC.getCurrentPosition());

        transferMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionTransferMech", transferMechDC.getDirection());
        transferMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeTransferMech:", transferMechDC.getCurrentPosition());

        intakeMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionIntakeMech", intakeMechDC.getDirection());
        intakeMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("DcMotor ModeIntakeMech:", intakeMechDC.getCurrentPosition());

        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Completed");
        telemetry.update();
    }

    private void initiateServoMotors() {
        telemetry.addData("Servo initiateServoMotors:", "Configuration Started");
        shooterMechRotatorServo = myRobotHW.getShooterMechRotatorServo();
        feederEnablerServo = myRobotHW.getFeederEnablerServo();
        specStopperServo = myRobotHW.getSpecStopperServo();
        // Initialize shooterMechRotatorServo & feederEnablerServo to the starting position
        specStopperServo.setDirection(Servo.Direction.FORWARD);
        shooterMechRotatorServo.setPosition(SHOOTER_ROTATOR_SERVO_START_POS);
        feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);
        specStopperServo.setPosition(STOPPER_SERVO_POS_INITIAL_GATE);
        telemetry.addData("Servo initiateServoMotors:", "Configuration Completed");
        telemetry.addLine("Use triggers to move shooterMechRotatorServo");
        telemetry.addLine("Right Trigger → upward movement");
        telemetry.addLine("Left Trigger → downward movement");
        telemetry.addData("shooterMechRotatorServo init position", specStopperServo.getPosition());
        telemetry.addData("shooterMechRotatorServo init direction", specStopperServo.getDirection());
        telemetry.update();
    }

    private void runShooterDCMotors() {
        // Reverse one motor so they spin opposite directions
        leftShooterDC.setDirection(DcMotor.Direction.FORWARD);
        rightShooterDC.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
        telemetry.update();
        startShooterDCMotors();
    }

    // Function to give scaled power to both shooter DC motors(opposite rotation)
    private void startShooterDCMotors() {
        // Ensure shooterDCPowerScale stays between 0 and 1
        leftShooterDC.setPower(shooterDCMotorPowerScaleFactor);
        rightShooterDC.setPower(shooterDCMotorPowerScaleFactor);
        telemetry.addData("DcMotor shooterDCPowerScale", shooterDCMotorPowerScaleFactor);
        telemetry.addData("DcMotor LeftShooter Power", leftShooterDC.getPower());
        telemetry.addData("DcMotor RightShooter Power", rightShooterDC.getPower());
        telemetry.update();
    }

    // Function to run Transfer DC Motor at scaled power
    private void runTransferMechDCMotor() {
        transferMechDC.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionTransfer", transferMechDC.getDirection());
        telemetry.update();
        startTransferMechDCMotor(transferDCMotorPowerScale);
    }

    // Function to give scaled power to Transfer DC Motor
    private void startTransferMechDCMotor(double transferDCPowerScale) {
        // Ensure transferDCPowerScale stays between 0 and 1
        transferDCPowerScale = Math.max(0, Math.min(transferDCPowerScale, 1));
        transferMechDC.setPower(transferDCPowerScale);
        telemetry.addData("DcMotor transferDCPowerScale", transferDCPowerScale);
        telemetry.addData("DcMotor Transfer Power", transferMechDC.getPower());
        telemetry.update();
    }

    // Function to run Intake DC Motor at scaled power
    private void runIntakeMechDCMotor() {
        intakeMechDC.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("DcMotor DirectionIntake", intakeMechDC.getDirection());
        telemetry.update();
        startIntakeMechDCMotor(intakeDCMotorPowerScale);
    }

    // Function to give scaled power to Intake DC Motor
    private void startIntakeMechDCMotor(double intakeDCPowerScale) {
        // Ensure intakeDCPowerScale stays between 0 and 1
        intakeDCPowerScale = Math.max(0, Math.min(intakeDCPowerScale, 1));
        intakeMechDC.setPower(intakeDCPowerScale);
        telemetry.addData("DcMotor intakeDCPowerScale", intakeDCPowerScale);
        telemetry.addData("DcMotor Intake Power", intakeMechDC.getPower());
        telemetry.update();
    }

    private void shootingAnglePositioner() {
        telemetry.addLine("shootingAnglePositioner");
        telemetry.update();
        // Limit within defined range
        rotorServoCurrentPosition = Math.max(SHOOTER_ROTATOR_SERVO_START_POS, rotorServoCurrentPosition);// Set new position
        // Set new position
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
        telemetry.addData("shootingAnglePositioner: rotorServoCurrentPosition", rotorServoCurrentPosition);
        telemetry.update();
    }

    private void operateFeederServo() {
        // Move up to 120°
        specStopperServo.setDirection(Servo.Direction.FORWARD);
        if(!stopperButtonStateForFirstTime) {
            stopperButtonStateForFirstTime = true;
        } else{
            specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
        }
        feederEnablerServo.setPosition(FEEDER_SERVO_POS_120_DEG);

        sleep(620); // adjust as needed for speed
        specStopperServo.setDirection(Servo.Direction.REVERSE);
        specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
        feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);

        sleep(620); // adjust as needed for speed
    }

    private void stopAllDCMotors() {
        telemetry.addData("stopAllDCMotors:  ", "started");
        stopShooterDCMotors();
        stopTransferMechDCMotor();
        stopIntakeMechDCMotor();
        telemetry.addData("stopAllDCMotors:  ", "ended");
    }

    // Function to stop both shooter DC motors (idle)
    private void stopShooterDCMotors() {
        leftShooterDC.setPower(0);
        rightShooterDC.setPower(0);
    }

    private void stopTransferMechDCMotor() {
        transferMechDC.setPower(0);
    }

    private void stopIntakeMechDCMotor() {
        intakeMechDC.setPower(0);
    }

    private void onRightTriggerPressed() {
        telemetry.addLine("RIGHT trigger clicked!");
        telemetry.update();
        rotorServoCurrentPosition = rotorServoCurrentPosition + STEP;
        // Limit within defined range
        rotorServoCurrentPosition = Math.min(SHOOTER_ROTATOR_SERVO_END_POS, rotorServoCurrentPosition);// Set new position
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
        telemetry.addData("RIGHT trigger clicked: rotorServoCurrentPosition", rotorServoCurrentPosition);
        telemetry.update();
    }
}




//package org.firstinspires.ftc.teamcode.apex.warrior.auto;
//
//import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigurator;
//
//import java.util.Locale;
//
//// Apex Team main Autonomous Mode
//@Autonomous(name = "Auto Drive - Red Team - Near to goal", group = "Auto Opmode")
//public class ApexTeamAutoModeRightNear extends LinearOpMode {
//    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
//    /* Declare OpMode members */
//    RobotHardwareConfigurator myRobotHW = new RobotHardwareConfigurator();
//    private static final double shooterDCMotorPowerScaleFactor = 0.42;
//    private static final double transferDCMotorPowerScale = 0.9;
//    private static final double intakeDCMotorPowerScale = 0.9;
//
//    // Servo positions (0.0 to 1.0) Rotator Assembly
//    private static final double SHOOTER_ROTATOR_SERVO_START_POS = 0.39;  //0.37 to 0.43 starting position
//    private static final double SHOOTER_ROTATOR_SERVO_END_POS = 0.43;  // 150° away from start
//    private static final double STEP = 0.01;  // // Step rate controls how fast the servo moves (smaller = slower)
//    private final double FEEDER_SERVO_POS_0_DEG = 0.0;   // represents 0 degrees
//    private final double FEEDER_SERVO_POS_120_DEG = 0.07; // approx 120 degrees on a 5-turn servo
//    private final double STOPPER_SERVO_POS_INITIAL_GATE = 0.60;
//    private final double STOPPER_SERVO_POS_OPEN_GATE = 0.70;
//    private boolean stopperButtonStateForFirstTime = false;
//
//    private DcMotor frontLeftChassisDC;
//    private DcMotor frontRightChassisDC;
//    private DcMotor backLeftChassisDC;
//    private DcMotor backRightChassisDC;
//    private DcMotor leftShooterDC;
//    private DcMotor rightShooterDC;
//    private DcMotor transferMechDC;
//    private DcMotor intakeMechDC;
//    private Servo shooterMechRotatorServo;
//    private Servo feederEnablerServo;
//    private Servo specStopperServo;
//
//    private boolean lastLeftTriggerState = false;
//    private boolean lastRightTriggerState = false;
//    private double rotorServoCurrentPosition;
//
//    ElapsedTime timer = new ElapsedTime();
//
//    PIDController xPID = new PIDController(0.01, 0.01, 0.0);
//    PIDController yPID = new PIDController(0.01, 0.01, 0.0);
//    PIDController thetaPID = new PIDController(0.01, 0.005, 0.0);
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        myRobotHW.configureRobotHardware(hardwareMap);
//        initMotorAndServo();
//        //odo = myRobotHW.getOdo();
//        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
//        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(19.8945588, DistanceUnit.MM);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();
//
//        telemetry.addData("Status", "HW Configuration Mapping Initialized");
//        telemetry.update();
//
//        telemetry.addData("Status", "HW Configuration Initialized");
//        telemetry.update();
//
//        telemetry.addData("Status", "Actuators Initialized");
//        telemetry.update();
//
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//        waitForStart();
//        telemetry.log().clear();
//
//        frontRightChassisDC.setDirection(REVERSE);
//        backRightChassisDC.setDirection(REVERSE);
//
//        shootingAnglePositioner();
//
//        /// //////////////////////////////////////////////
////        Thread controlThread = new Thread(() -> controlLoop(targetXMeters, targetHeadingDeg), "ControlLoop");
//        Thread controlThread = new Thread(this::controlLoop, "ControlLoop");
//        Thread shooterThread = new Thread(this::shooterLoop, "ShooterLoop");
//
//        controlThread.start();
//        shooterThread.start();
//
//        // Main thread just watches for stop and keeps telemetry alive
//        while (opModeIsActive() && controlThread.isAlive()) {
//            telemetry.addLine("Autonomous running...");
//            telemetry.update();
//            sleep(50);
//        }
//
//        // Signal threads to stop (if not already)
////        shouldStop.set(true);
//        controlThread.interrupt();
//        shooterThread.interrupt();
//
//        try {
//            controlThread.join(300);
//            shooterThread.join(300);
//        } catch (InterruptedException ignored) {
//            stopAllDCMotors();
//        }
//        telemetry.addLine("Autonomous complete.");
//        telemetry.update();
//    }
//
//    public void controlLoop() {
//        long lastTime = System.currentTimeMillis();
//
//        double bot_init_x0 = 0.0, bot_init_y0 = 0, bot_init_heading = 0;
//        long time_phase1 = System.currentTimeMillis();
//        long time_phase2 = 0;
//
//        while (opModeIsActive()) {
//            specStopperServo.setPosition(STOPPER_SERVO_POS_INITIAL_GATE);
//            odo.update();
//            Pose2D curPos = odo.getPosition();
//
//            double x_in_botAxis = curPos.getX(DistanceUnit.MM);
//            double y_in_botAxis = curPos.getY(DistanceUnit.MM);
//            double heading_deg = curPos.getHeading(AngleUnit.DEGREES);
//
//            double theta = Math.toRadians(heading_deg); // convert to radians
////            x_world =  x_bot * cosθ - y_bot * sinθ
////            y_world =  x_bot * sinθ + y_bot * cosθ
////            double intit_heading_rad = Math.toRadians(bot_init_heading)
//
////            double cur_bot_x_in_world_coord =  cur_bot_x_in_botAxis * Math.cos(intit_heading_rad) - cur_bot_y_in_botAxis * Math.sin(intit_heading_rad);
////            double cur_bot_y_in_world_coord =  cur_bot_x_in_botAxis * Math.sin(intit_heading_rad) + cur_bot_y_in_botAxis * Math.cos(intit_heading_rad);
//
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", x_in_botAxis,
//                    y_in_botAxis, heading_deg);
//            telemetry.addData("Position", data);
//            Pose2d target;
////            target = new Pose2d(-2110.0, 0.0, 45.0);
//            target = new Pose2d(-220.0, 45.0, -22.0);
//
//            runShooterDCMotors();
//            long curTime = System.currentTimeMillis();
////            if( curTime - time_phase1 > 5000){
////                shootingAnglePositioner();
////                runIntakeMechDCMotor();
////                runTransferMechDCMotor();
////                operateFeederServo();
////            }
//
//            long currentTime = System.currentTimeMillis();
//            double dt = (currentTime - lastTime) / 1e3;
//            lastTime = currentTime;
//
//            data = String.format(Locale.US, "{dt: %.3f}", dt);
//            telemetry.addData("dt: ", data);
//
//            double errorX = target.x - curPos.getX(DistanceUnit.MM);
//            double errorY = target.y - curPos.getY(DistanceUnit.MM);
//            double errorTheta = target.heading - curPos.getHeading(AngleUnit.DEGREES);
//
//            double vx = xPID.update(errorX, dt);
//            double vy = yPID.update(errorY, dt);
//            double omega = thetaPID.update(errorTheta, dt);
//
//            if(vx > 0.2)
//                vx = 0.2;
//            if(vx < -0.2)
//                vx = -0.2;
//
//            if(vy > 0.2)
//                vy = 0.2;
//            if(vy < -0.2)
//                vy = -0.2;
//
//            if(omega > 0.2)
//                omega = 0.2;
//            if(omega < -0.2)
//                omega = -0.2;
//
//            data = String.format(Locale.US, "{Vx: %.3f, Vy: %.3f, Heading Cmd: %.3f}", vx,
//                    vy, omega);
//            telemetry.addData("Control command: ", data);
//            telemetry.update();
//
//            double compensatedVx = vx * Math.cos(theta) + vy * Math.sin(theta);
//            double compensatedVy = -vx * Math.sin(theta) + vy * Math.cos(theta);
//
//            vx = compensatedVx;
//            vy = compensatedVy;
//
////            vx = 0.0; // 0.
////            vy = 0.0; // 0.5
////            omega = 0.0; // 0.35
//
//            double fl = -vx + vy + omega;
//            double fr = -vx - vy - omega;
//            double bl = -vx - vy + omega;
//            double br = -vx + vy - omega;
//            frontLeftChassisDC.setPower(fl);
//            frontRightChassisDC.setPower(fr);
//            backLeftChassisDC.setPower(bl);
//            backRightChassisDC.setPower(br);
//        }
//        telemetry.addData("Status", "Stopping Motors");
//        stopAllDCMotors();
//    }
//    public void shooterLoop() {
//        shootingAnglePositioner();
//        boolean flag = true;
//        long time_phase1 = System.currentTimeMillis();
//        while(opModeIsActive()) {
//            long curTime = System.currentTimeMillis();
//            if (curTime - time_phase1 > 4000 ) {
//                if(flag) {
//                    runIntakeMechDCMotor();
//                    runTransferMechDCMotor();
//                    flag = false;
//                } else {
//                    try {
//                        operateFeederServo();
//                        Thread.sleep(800);   // wait 800 ms
//                    } catch (InterruptedException exe) {
//                        telemetry.addData("InterruptedException", exe.getCause());
//                        break;               // stop cleanly if interrupted
//                    }
//                }
//            }
//        }
//    }
//        /// /////////////////////////////////////////////
//
//    private void initMotorAndServo() {
//        initiateChassisDCMotors();
//        initiateOtherDCMotors();
//        initiateServoMotors();
//    }
//
//    private void initiateChassisDCMotors() {
//        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Started");
//
//        backLeftChassisDC = myRobotHW.getBackLeftChassisDC();
//        backRightChassisDC = myRobotHW.getBackRightChassisDC();
//        frontLeftChassisDC = myRobotHW.getFrontLeftChassisDC();
//        frontRightChassisDC = myRobotHW.getFrontRightChassisDC();
//
//        backLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionBL", backLeftChassisDC.getDirection());
//        backLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("DcMotor ModeBL:", backLeftChassisDC.getCurrentPosition());
//        backLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        backRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionBR", backRightChassisDC.getDirection());
//        backRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("DcMotor ModeBR:", backRightChassisDC.getCurrentPosition());
//        backRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionFL", frontLeftChassisDC.getDirection());
//        frontLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("DcMotor ModeFL:", frontLeftChassisDC.getCurrentPosition());
//        frontLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        frontRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionFR", frontRightChassisDC.getDirection());
//        frontRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        telemetry.addData("DcMotor ModeFR:", frontRightChassisDC.getCurrentPosition());
//        frontRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        telemetry.addData("DcMotor initiateChassisDCMotors:", "Configuration Completed");
//        telemetry.update();
//    }
//
//    private void initiateOtherDCMotors() {
//        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Started");
//
//        leftShooterDC = myRobotHW.getLeftShooterDC();
//        rightShooterDC = myRobotHW.getRightShooterDC();
//        transferMechDC = myRobotHW.getTransferMechDC();
//        intakeMechDC = myRobotHW.getIntakeMechDC();
//
//        leftShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
//        leftShooterDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("DcMotor ModeLeftShooter:", leftShooterDC.getCurrentPosition());
//
//
//        rightShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
//        rightShooterDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("DcMotor ModeRightShooter:", rightShooterDC.getCurrentPosition());
//
//        transferMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionTransferMech", transferMechDC.getDirection());
//        transferMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("DcMotor ModeTransferMech:", transferMechDC.getCurrentPosition());
//
//        intakeMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionIntakeMech", intakeMechDC.getDirection());
//        intakeMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("DcMotor ModeIntakeMech:", intakeMechDC.getCurrentPosition());
//
//        telemetry.addData("DcMotor initiateOtherDCMotors:", "Configuration Completed");
//        telemetry.update();
//    }
//
//    private void initiateServoMotors() {
//        telemetry.addData("Servo initiateServoMotors:", "Configuration Started");
//        shooterMechRotatorServo = myRobotHW.getShooterMechRotatorServo();
//        feederEnablerServo = myRobotHW.getFeederEnablerServo();
//        specStopperServo = myRobotHW.getSpecStopperServo();
//        // Initialize shooterMechRotatorServo & feederEnablerServo to the starting position
//        specStopperServo.setDirection(Servo.Direction.FORWARD);
//        shooterMechRotatorServo.setPosition(SHOOTER_ROTATOR_SERVO_START_POS);
//        feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);
//        specStopperServo.setPosition(STOPPER_SERVO_POS_INITIAL_GATE);
//        telemetry.addData("Servo initiateServoMotors:", "Configuration Completed");
//        telemetry.addLine("Use triggers to move shooterMechRotatorServo");
//        telemetry.addLine("Right Trigger → upward movement");
//        telemetry.addLine("Left Trigger → downward movement");
//        telemetry.addData("shooterMechRotatorServo init position", specStopperServo.getPosition());
//        telemetry.addData("shooterMechRotatorServo init direction", specStopperServo.getDirection());
//        telemetry.update();
//    }
//
//    private void runShooterDCMotors() {
//        // Reverse one motor so they spin opposite directions
//        leftShooterDC.setDirection(DcMotor.Direction.FORWARD);
//        rightShooterDC.setDirection(DcMotor.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
//        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
//        telemetry.update();
//        startShooterDCMotors();
//    }
//
//    // Function to give scaled power to both shooter DC motors(opposite rotation)
//    private void startShooterDCMotors() {
//        // Ensure shooterDCPowerScale stays between 0 and 1
//        leftShooterDC.setPower(shooterDCMotorPowerScaleFactor);
//        rightShooterDC.setPower(shooterDCMotorPowerScaleFactor);
//        telemetry.addData("DcMotor shooterDCPowerScale", shooterDCMotorPowerScaleFactor);
//        telemetry.addData("DcMotor LeftShooter Power", leftShooterDC.getPower());
//        telemetry.addData("DcMotor RightShooter Power", rightShooterDC.getPower());
//        telemetry.update();
//    }
//
//    // Function to run Transfer DC Motor at scaled power
//    private void runTransferMechDCMotor() {
//        transferMechDC.setDirection(DcMotor.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionTransfer", transferMechDC.getDirection());
//        telemetry.update();
//        startTransferMechDCMotor(transferDCMotorPowerScale);
//    }
//
//    // Function to give scaled power to Transfer DC Motor
//    private void startTransferMechDCMotor(double transferDCPowerScale) {
//        // Ensure transferDCPowerScale stays between 0 and 1
//        transferDCPowerScale = Math.max(0, Math.min(transferDCPowerScale, 1));
//        transferMechDC.setPower(transferDCPowerScale);
//        telemetry.addData("DcMotor transferDCPowerScale", transferDCPowerScale);
//        telemetry.addData("DcMotor Transfer Power", transferMechDC.getPower());
//        telemetry.update();
//    }
//
//    // Function to run Intake DC Motor at scaled power
//    private void runIntakeMechDCMotor() {
//        intakeMechDC.setDirection(DcMotor.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionIntake", intakeMechDC.getDirection());
//        telemetry.update();
//        startIntakeMechDCMotor(intakeDCMotorPowerScale);
//    }
//
//    // Function to give scaled power to Intake DC Motor
//    private void startIntakeMechDCMotor(double intakeDCPowerScale) {
//        // Ensure intakeDCPowerScale stays between 0 and 1
//        intakeDCPowerScale = Math.max(0, Math.min(intakeDCPowerScale, 1));
//        intakeMechDC.setPower(intakeDCPowerScale);
//        telemetry.addData("DcMotor intakeDCPowerScale", intakeDCPowerScale);
//        telemetry.addData("DcMotor Intake Power", intakeMechDC.getPower());
//        telemetry.update();
//    }
//
//    private void shootingAnglePositioner() {
//        telemetry.addLine("shootingAnglePositioner");
//        telemetry.update();
//        // Limit within defined range
//        rotorServoCurrentPosition = Math.max(SHOOTER_ROTATOR_SERVO_START_POS, rotorServoCurrentPosition);// Set new position
//        // Set new position
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//        telemetry.addData("shootingAnglePositioner: rotorServoCurrentPosition", rotorServoCurrentPosition);
//        telemetry.update();
//    }
//
//    private void operateFeederServo() {
//        // Move up to 120°
//        specStopperServo.setDirection(Servo.Direction.FORWARD);
//        if(!stopperButtonStateForFirstTime) {
//            stopperButtonStateForFirstTime = true;
//        } else{
//            specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
//        }
//        feederEnablerServo.setPosition(FEEDER_SERVO_POS_120_DEG);
//        telemetry.addData("stopperButtonStateForFirstTime flag:", stopperButtonStateForFirstTime);
//        telemetry.addData("feederEnablerServo Position", feederEnablerServo.getPosition());
//        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
//        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
//        telemetry.update();
//        sleep(400); // adjust as needed for speed
//        specStopperServo.setDirection(Servo.Direction.REVERSE);
//        specStopperServo.setPosition(STOPPER_SERVO_POS_OPEN_GATE);
//        feederEnablerServo.setPosition(FEEDER_SERVO_POS_0_DEG);
//        telemetry.addData("feederEnablerServo Position", feederEnablerServo.getPosition());
//        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
//        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
//        telemetry.update();
//        sleep(400); // adjust as needed for speed
//    }
//
//    private void stopAllDCMotors() {
//        telemetry.addData("stopAllDCMotors:  ", "started");
//        stopShooterDCMotors();
//        stopTransferMechDCMotor();
//        stopIntakeMechDCMotor();
//        telemetry.addData("stopAllDCMotors:  ", "ended");
//    }
//
//    // Function to stop both shooter DC motors (idle)
//    private void stopShooterDCMotors() {
//        leftShooterDC.setPower(0);
//        rightShooterDC.setPower(0);
//    }
//
//    private void stopTransferMechDCMotor() {
//        transferMechDC.setPower(0);
//    }
//
//    private void stopIntakeMechDCMotor() {
//        intakeMechDC.setPower(0);
//    }
//
//    private void onRightTriggerPressed() {
//        telemetry.addLine("RIGHT trigger clicked!");
//        telemetry.update();
//        rotorServoCurrentPosition = rotorServoCurrentPosition + STEP;
//        // Limit within defined range
//        rotorServoCurrentPosition = Math.min(SHOOTER_ROTATOR_SERVO_END_POS, rotorServoCurrentPosition);// Set new position
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//        telemetry.addData("RIGHT trigger clicked: rotorServoCurrentPosition", rotorServoCurrentPosition);
//        telemetry.update();
//    }
//}
//
//
