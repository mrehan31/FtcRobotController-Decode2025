package org.firstinspires.ftc.teamcode.apex.warrior;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.apex.warrior.DcMotorConstant.chassisDCMotorPowerScale;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
// Apex Team main TeleOp Mode
@TeleOp(name = "Apex Warrior Robot Manual Drive - 2025", group = "Linear Opmode")
public class ApexTeamTeleOpMode extends LinearOpMode {

    /* Declare OpMode members */
    RobotHardwareConfigurator myRobotHW = new RobotHardwareConfigurator();
    private static final double STEP = 0.01;
    private boolean stopperButtonStateForFirstTime = false;
    private DcMotor frontLeftChassisDC;
    private DcMotor frontRightChassisDC;
    private DcMotor backLeftChassisDC;
    private DcMotor backRightChassisDC;
//    private DcMotor leftShooterDC;
//    private DcMotor rightShooterDC;
    private DcMotorEx leftShooterDC;
    private DcMotorEx rightShooterDC;
    private static final double MAX_SHOOTER_TICKS_PER_SEC = 2800.0;
    private DcMotor transferMechDC;
    private DcMotor intakeMechDC;
    private Servo shooterMechRotatorServo;
    private Servo feederEnablerServo;
    private Servo specStopperServo;
    private boolean lastLeftTriggerStateOnGamepad1 = false;
    private boolean lastRightTriggerStateOnGamepad1 = false;
    private boolean lastLeftTriggerStateOnGamepad2 = false;
    private boolean lastRightTriggerStateOnGamepad2 = false;
    private double rotorServoCurrentPosition;
    private Limelight3A limelight;
    private ServoImplEx indicatorLight;

    // --- NON-BLOCKING FEEDER SEQUENCE ---
    private ElapsedTime feederTimer = new ElapsedTime();
    private int feederState = 0; // 0 = Idle, 1 = Stage 1, 2 = Stage 2

    // --- NEW: STATE TRACKER FOR PWM ---
    private boolean isLightEnabled = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "HW Configuration Mapping Initialized");
        myRobotHW.configureRobotHardware(hardwareMap);
        telemetry.update();

        telemetry.addData("Status", "DC Motors & Servo Motors - Configuration Initialized");
        initMotorAndServo();
        telemetry.update();

        telemetry.addData(">>>", "Robot Ready.  Press Play.");
        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
        telemetry.update();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();
        indicatorLight = hardwareMap.get(ServoImplEx.class, "indicator_light");
        indicatorLight.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Ensure light starts totally off and tracked correctly
        indicatorLight.setPwmDisable();
        isLightEnabled = false;

        telemetry.addLine("RGB Indicator Initialized.");
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();
        telemetry.log().clear();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            // Flag to check if we actually found a tag this specific loop
            boolean tagDetectedThisFrame = false;

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();

                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());

                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                if (!fiducialResults.isEmpty()) {
                    tagDetectedThisFrame = true;

                    // Only send the enable command if it was previously off
                    indicatorLight.setPwmEnable();

                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());

                        if(fr.getTargetXDegrees() < 11 && fr.getTargetXDegrees() > 6){
                            indicatorLight.setPosition(0.5);
                        } else if(fr.getTargetXDegrees() >= 11){
                            indicatorLight.setPosition(0.7);
                        } else if(fr.getTargetXDegrees() <= 6){
                            indicatorLight.setPosition(0.32);
                        }
                    }
                }

                // (Omitted other result printouts for brevity, but they would go here)
            } else {
                tagDetectedThisFrame = false;
                indicatorLight.setPosition(0.75);

                telemetry.addData("Limelight", "No valid targets available");
            }

            // Cleanly turn off the light if no tag was found, but ONLY send command once
            if (!tagDetectedThisFrame) {
                indicatorLight.setPosition(0.75);
            }

            setChassisDCMotorsDirection();
            double frontLeftChassisPow, frontRightChassisPow, backLeftChassisPow, backRightChassisPow;

            handleFeederSequence();

            boolean leftTriggerPressedOnGamepad1  = gamepad1.left_trigger  > 0.5;
            boolean rightTriggerPressedOnGamepad1 = gamepad1.right_trigger > 0.5;
            boolean leftTriggerPressedOnGamepad2  = gamepad2.left_trigger  > 0.5;
            boolean rightTriggerPressedOnGamepad2 = gamepad2.right_trigger > 0.5;

            if (leftTriggerPressedOnGamepad1 && !lastLeftTriggerStateOnGamepad1) onLeftTriggerPressedOnGamepad1();
            if (rightTriggerPressedOnGamepad1 && !lastRightTriggerStateOnGamepad1) onRightTriggerPressedOnGamepad1();
            if (leftTriggerPressedOnGamepad2 && !lastLeftTriggerStateOnGamepad2) onLeftTriggerPressedOnGamepad2();
            if (rightTriggerPressedOnGamepad2 && !lastRightTriggerStateOnGamepad2) onRightTriggerPressedOnGamepad2();

            lastLeftTriggerStateOnGamepad1 = leftTriggerPressedOnGamepad1;
            lastRightTriggerStateOnGamepad1 = rightTriggerPressedOnGamepad1;
            lastLeftTriggerStateOnGamepad2 = leftTriggerPressedOnGamepad2;
            lastRightTriggerStateOnGamepad2 = rightTriggerPressedOnGamepad2;

            mecanumDriveGamepadOneJoyStickControlled();

            /* GamePade 1 -> Joystick Controls */
            if (gamepad1.dpad_up) {
                frontLeftChassisPow = chassisDCMotorPowerScale; frontRightChassisPow = chassisDCMotorPowerScale;
                backLeftChassisPow = chassisDCMotorPowerScale; backRightChassisPow = chassisDCMotorPowerScale;
                frontLeftChassisDC.setPower(frontLeftChassisPow); frontRightChassisDC.setPower(frontRightChassisPow);
                backLeftChassisDC.setPower(backLeftChassisPow); backRightChassisDC.setPower(backRightChassisPow);
            } else if (gamepad1.dpad_down) {
                frontLeftChassisPow = -chassisDCMotorPowerScale; frontRightChassisPow = -chassisDCMotorPowerScale;
                backLeftChassisPow = -chassisDCMotorPowerScale; backRightChassisPow = -chassisDCMotorPowerScale;
                frontLeftChassisDC.setPower(frontLeftChassisPow); frontRightChassisDC.setPower(frontRightChassisPow);
                backLeftChassisDC.setPower(backLeftChassisPow); backRightChassisDC.setPower(backRightChassisPow);
            } else if (gamepad1.dpad_left) {
                frontLeftChassisPow = -chassisDCMotorPowerScale; frontRightChassisPow = chassisDCMotorPowerScale;
                backLeftChassisPow = chassisDCMotorPowerScale; backRightChassisPow = -chassisDCMotorPowerScale;
                frontLeftChassisDC.setPower(frontLeftChassisPow); frontRightChassisDC.setPower(frontRightChassisPow);
                backLeftChassisDC.setPower(backLeftChassisPow); backRightChassisDC.setPower(backRightChassisPow);
            } else if (gamepad1.dpad_right) {
                frontLeftChassisPow = chassisDCMotorPowerScale; frontRightChassisPow = -chassisDCMotorPowerScale;
                backLeftChassisPow = -chassisDCMotorPowerScale; backRightChassisPow = chassisDCMotorPowerScale;
                frontLeftChassisDC.setPower(frontLeftChassisPow); frontRightChassisDC.setPower(frontRightChassisPow);
                backLeftChassisDC.setPower(backLeftChassisPow); backRightChassisDC.setPower(backRightChassisPow);
            } else if (gamepad1.x) {
                moveRotatorAssemblyToSuppliedPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_DEFAULT_POS_NEAR, "GamePad 1 x clicked!");
            } else if (gamepad1.b) {
                moveRotatorAssemblyToSuppliedPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_DEFAULT_POS_FAR, "GamePad 1 b clicked!");
            }

            /* GamePade 2 -> Joystick Controls */
            if (gamepad2.x) {
                runIntakeMechDCMotor();
            } else if (gamepad2.b) {
                stopIntakeMechDCMotor();
            } else if (gamepad2.y) {
                runTransferMechDCMotor();
            } else if (gamepad2.a) {
                runTransferMechDCMotorInReverse();
            } else if (gamepad2.dpad_up) {
//                stopShooterDCMotors();
                runShooterDCMotors(DcMotorConstant.shooterDCMotorPowerScaleFactorNearRegion);
            } else if (gamepad2.dpad_down) {
                stopShooterDCMotors();
            } else if (gamepad2.left_bumper) {
//                stopShooterDCMotors();
                runShooterDCMotors(DcMotorConstant.shooterDCMotorPowerScaleFactorFarRegion);
            }

            telemetry.update();
            idle();
        }
        limelight.stop();
    }

    private void handleFeederSequence() {
        if (gamepad2.right_bumper && feederState == 0) {
            feederState = 1;
            feederTimer.reset();

            specStopperServo.setDirection(Servo.Direction.FORWARD);
            if(!stopperButtonStateForFirstTime) {
                stopperButtonStateForFirstTime = true;
            } else{
                specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_OPEN_GATE);
            }
            feederEnablerServo.setPosition(ServoMotorConstant.FEEDER_SERVO_POS_120_DEG);
        }

        if (feederState == 1 && feederTimer.milliseconds() > 400) {
            feederState = 2;
            feederTimer.reset();

            specStopperServo.setDirection(Servo.Direction.REVERSE);
            specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_OPEN_GATE);
            feederEnablerServo.setPosition(ServoMotorConstant.FEEDER_SERVO_POS_0_DEG);
        }
        else if (feederState == 2 && feederTimer.milliseconds() > 400) {
            feederState = 0;
            specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_INITIAL_GATE);
        }
    }

    private void setChassisDCMotorsDirection() {
        frontRightChassisDC.setDirection(REVERSE);
        backRightChassisDC.setDirection(REVERSE);
    }

    private void moveRotatorAssemblyToSuppliedPosition(double shooterRotatorServoPosition, String eventSource) {
        rotorServoCurrentPosition = shooterRotatorServoPosition;
        telemetry.addData(eventSource, rotorServoCurrentPosition);
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
    }

    private void onLeftTriggerPressedOnGamepad1() {
        rotorServoCurrentPosition = Math.max(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_START_POS_FAR, rotorServoCurrentPosition - STEP);
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
    }

    private void onRightTriggerPressedOnGamepad1() {
        rotorServoCurrentPosition = Math.min(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_END_POS_FAR, rotorServoCurrentPosition + STEP);
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
    }

    private void onLeftTriggerPressedOnGamepad2() {
        rotorServoCurrentPosition = Math.max(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_START_POS_NEAR, rotorServoCurrentPosition - STEP);
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
    }

    private void onRightTriggerPressedOnGamepad2() {
        rotorServoCurrentPosition = Math.min(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_END_POS_NEAR, rotorServoCurrentPosition + STEP);
        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
    }

    private void initMotorAndServo() {
        initiateChassisDCMotors();
        initiateOtherDCMotors();
        initiateServoMotors();
    }

    private void initiateChassisDCMotors() {
        backLeftChassisDC = myRobotHW.getBackLeftChassisDC();
        backRightChassisDC = myRobotHW.getBackRightChassisDC();
        frontLeftChassisDC = myRobotHW.getFrontLeftChassisDC();
        frontRightChassisDC = myRobotHW.getFrontRightChassisDC();

        backLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRightChassisDC.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightChassisDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightChassisDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initiateOtherDCMotors() {
//        leftShooterDC = myRobotHW.getLeftShooterDC();
//        rightShooterDC = myRobotHW.getRightShooterDC();
        // code added for encoder based speed control
        leftShooterDC = (DcMotorEx) myRobotHW.getLeftShooterDC();
        rightShooterDC = (DcMotorEx) myRobotHW.getRightShooterDC();

        ///////////////////////////////////////////////////////////////////
//        leftShooterDC = (DcMotorEx) myRobotHW.getLeftShooterDC();
//        rightShooterDC = (DcMotorEx) myRobotHW.getRightShooterDC();

        // 1. Set mode first
        leftShooterDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2. Define custom PIDF Coefficients for a Flywheel
        // F = 32767 / Max Velocity (e.g., 32767 / 2800 ≈ 11.7)
        // P = Usually around 10% of F. I and D should be 0 for flywheels.
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(3, 0.0, 0, 50);

        // 3. Apply the custom coefficients
        leftShooterDC.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);
        rightShooterDC.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPIDF);



        //////////////////////////////////////////////////////





        leftShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooterDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooterDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooterDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //-------------------------------------------------------
        transferMechDC = myRobotHW.getTransferMechDC();
        intakeMechDC = myRobotHW.getIntakeMechDC();

//        leftShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftShooterDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        rightShooterDC.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightShooterDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transferMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMechDC.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMechDC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initiateServoMotors() {
        shooterMechRotatorServo = myRobotHW.getShooterMechRotatorServo();
        feederEnablerServo = myRobotHW.getFeederEnablerServo();
        specStopperServo = myRobotHW.getSpecStopperServo();

        specStopperServo.setDirection(Servo.Direction.FORWARD);
        shooterMechRotatorServo.setPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_DEFAULT_POS_NEAR);
        feederEnablerServo.setPosition(ServoMotorConstant.FEEDER_SERVO_POS_0_DEG);
        specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_INITIAL_GATE);
    }

    private void mecanumDriveGamepadOneJoyStickControlled() {
        setChassisDCMotorsDirection();
        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double v1 = h * Math.sin(robotAngle) - rightX;
        final double v2 = h * Math.cos(robotAngle) + rightX;
        final double v3 = h * Math.cos(robotAngle) - rightX;
        final double v4 = h * Math.sin(robotAngle) + rightX;

        frontLeftChassisDC.setPower(v1);
        frontRightChassisDC.setPower(v2);
        backLeftChassisDC.setPower(v3);
        backRightChassisDC.setPower(v4);
    }

    private void runShooterDCMotors(double shooterDCMotorPowerScaleFactor) {
//        leftShooterDC.setDirection(DcMotor.Direction.FORWARD);
//        rightShooterDC.setDirection(DcMotor.Direction.FORWARD);
        startShooterDCMotors(shooterDCMotorPowerScaleFactor);
    }

//    private void startShooterDCMotors(double shooterDCMotorPowerScaleFactor) {
//        shooterDCMotorPowerScaleFactor = Math.max(0, Math.min(shooterDCMotorPowerScaleFactor, 1));
//        leftShooterDC.setPower(shooterDCMotorPowerScaleFactor);
//        rightShooterDC.setPower(shooterDCMotorPowerScaleFactor);
//    }

    private void startShooterDCMotors(double shooterDCMotorPowerScaleFactor) {
    // Clamp the scale factor between 0 and 1
        shooterDCMotorPowerScaleFactor = Math.max(0, Math.min(shooterDCMotorPowerScaleFactor, 1));

        // Calculate the target velocity in Ticks Per Second
        double targetVelocity = shooterDCMotorPowerScaleFactor * MAX_SHOOTER_TICKS_PER_SEC;

        leftShooterDC.setVelocity(targetVelocity);
        rightShooterDC.setVelocity(targetVelocity);
    }

    private void runTransferMechDCMotor() {
        transferMechDC.setDirection(DcMotor.Direction.FORWARD);
        startTransferMechDCMotor(DcMotorConstant.transferDCMotorPowerScale);
    }

    private void startTransferMechDCMotor(double transferDCPowerScale) {
        transferDCPowerScale = Math.max(0, Math.min(transferDCPowerScale, 1));
        transferMechDC.setPower(transferDCPowerScale);
    }

    private void runIntakeMechDCMotor() {
        intakeMechDC.setDirection(DcMotor.Direction.FORWARD);
        startIntakeMechDCMotor(DcMotorConstant.intakeDCMotorPowerScale);
    }

    private void startIntakeMechDCMotor(double intakeDCPowerScale) {
        intakeDCPowerScale = Math.max(0, Math.min(intakeDCPowerScale, 1));
        intakeMechDC.setPower(intakeDCPowerScale);
    }

    private void runTransferMechDCMotorInReverse() {
        transferMechDC.setDirection(DcMotor.Direction.REVERSE);
        startTransferMechDCMotor(DcMotorConstant.transferDCMotorPowerScale);
    }

//    private void stopShooterDCMotors() {
//        leftShooterDC.setPower(0);
//        rightShooterDC.setPower(0);
//    }
    private void stopShooterDCMotors() {
        leftShooterDC.setVelocity(0);
        rightShooterDC.setVelocity(0);
    }

    private void stopTransferMechDCMotor() {
        transferMechDC.setPower(0);
    }

    private void stopIntakeMechDCMotor() {
        intakeMechDC.setPower(0);
    }
}






//package org.firstinspires.ftc.teamcode.apex.warrior;
//
//import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
//
//import static org.firstinspires.ftc.teamcode.apex.warrior.DcMotorConstant.chassisDCMotorPowerScale;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.LLStatus;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PwmControl;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import java.util.List;
//
//// Apex Team main TeleOp Mode
//@TeleOp(name = "Apex Warrior Robot Manual Drive - 2025", group = "Linear Opmode")
//public class ApexTeamTeleOpMode extends LinearOpMode {
//
//    /* Declare OpMode members */
//    RobotHardwareConfigurator myRobotHW = new RobotHardwareConfigurator();
//    private static final double STEP = 0.01;  // // Step rate controls how fast the servo moves (smaller = slower)
//    private boolean stopperButtonStateForFirstTime = false;
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
//    private boolean lastLeftTriggerStateOnGamepad1 = false;
//    private boolean lastRightTriggerStateOnGamepad1 = false;
//    private boolean lastLeftTriggerStateOnGamepad2 = false;
//    private boolean lastRightTriggerStateOnGamepad2 = false;
//    private double rotorServoCurrentPosition;
//    private Limelight3A limelight;
//    private ServoImplEx indicatorLight;
//
//
//    @Override
//    public void runOpMode() {
//
//        telemetry.addData("Status", "HW Configuration Mapping Initialized");
//        myRobotHW.configureRobotHardware(hardwareMap);
//        telemetry.update();
//
//        telemetry.addData("Status", "DC Motors & Servo Motors - Configuration Initialized");
//        //Initialize DC Motors & Servo Motors
//        initMotorAndServo();
//        telemetry.update();
//
//        telemetry.addData(">>>", "Robot Ready.  Press Play.");
//        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
//        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
//        telemetry.update();
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);
//
//        limelight.start();
//        indicatorLight = hardwareMap.get(ServoImplEx.class, "indicator_light");
//        // Force the PWM range to 500-2500µs so 0.0 and 1.0 map correctly
//        indicatorLight.setPwmRange(new PwmControl.PwmRange(500, 2500));
//
//        telemetry.addLine("RGB Indicator Initialized.");
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//
//        waitForStart();
//        telemetry.log().clear();
//
//        while (opModeIsActive()) {
//            LLStatus status = limelight.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
//
//            LLResult result = limelight.getLatestResult();
//            if (result.isValid()) {
//                // Access general information
//                indicatorLight.setPwmEnable();
//                Pose3D botpose = result.getBotpose();
//                double captureLatency = result.getCaptureLatency();
//                double targetingLatency = result.getTargetingLatency();
//                double parseLatency = result.getParseLatency();
////                telemetry.addData("LL Latency", captureLatency + targetingLatency);
////                telemetry.addData("Parse Latency", parseLatency);
////                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
//
//                telemetry.addData("tx", result.getTx());
////                telemetry.addData("txnc", result.getTxNC());
//                telemetry.addData("ty", result.getTy());
////                telemetry.addData("tync", result.getTyNC());
//
////                telemetry.addData("Botpose", botpose.toString());
//
//                // Access barcode results
//                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
//                for (LLResultTypes.BarcodeResult br : barcodeResults) {
//                    telemetry.addData("Barcode", "Data: %s", br.getData());
//                }
//
//                // Access classifier results
//                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
//                for (LLResultTypes.ClassifierResult cr : classifierResults) {
//                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
//                }
//
//                // Access detector results
//                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
//                for (LLResultTypes.DetectorResult dr : detectorResults) {
//                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
//                }
//
//                // Access fiducial results
//                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//                for (LLResultTypes.FiducialResult fr : fiducialResults) {
//                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//                    if(fr.getTargetXDegrees() < 11 && fr.getTargetXDegrees() > 6){
//                        indicatorLight.setPosition(0.5);
//                    }
//                    if(fr.getTargetXDegrees() >= 11)
//                        indicatorLight.setPosition(0.7);
//                    if(fr.getTargetXDegrees() < 6)
//                        indicatorLight.setPosition(0.32);
//                }
//
//                // Access color results
//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//                for (LLResultTypes.ColorResult cr : colorResults) {
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//                }
//            } else {
//                indicatorLight.setPwmDisable();
//                telemetry.addData("Limelight", "No data available");
//            }
//
//            setChassisDCMotorsDirection();
//            // variable used for all chassis DC Motors, used only in case of DPAD controlled operations
//            double frontLeftChassisPow, frontRightChassisPow, backLeftChassisPow, backRightChassisPow;
//
//            specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_INITIAL_GATE);
//
//            // Current trigger states of GamePad 2 (threshold of 0.5)
//            boolean leftTriggerPressedOnGamepad1  = gamepad1.left_trigger  > 0.5;
//            boolean rightTriggerPressedOnGamepad1 = gamepad1.right_trigger > 0.5;
//
//            // Current trigger states of GamePad 2 (threshold of 0.5)
//            boolean leftTriggerPressedOnGamepad2  = gamepad2.left_trigger  > 0.5;
//            boolean rightTriggerPressedOnGamepad2 = gamepad2.right_trigger > 0.5;
//
//            // ---------- Left Trigger Edge Detect On Gamepad1 ----------
//            if (leftTriggerPressedOnGamepad1 && !lastLeftTriggerStateOnGamepad1) {
//                onLeftTriggerPressedOnGamepad1();
//            }
//
//            // ---------- Right Trigger Edge Detect On Gamepad1 ----------
//            if (rightTriggerPressedOnGamepad1 && !lastRightTriggerStateOnGamepad1) {
//                onRightTriggerPressedOnGamepad1();
//            }
//
//            // ---------- Left Trigger Edge Detect On Gamepad2 ----------
//            if (leftTriggerPressedOnGamepad2 && !lastLeftTriggerStateOnGamepad2) {
//                onLeftTriggerPressedOnGamepad2();
//            }
//
//            // ---------- Right Trigger Edge Detect On Gamepad2 ----------
//            if (rightTriggerPressedOnGamepad2 && !lastRightTriggerStateOnGamepad2) {
//                onRightTriggerPressedOnGamepad2();
//            }
//
//            // Update previous states
//            lastLeftTriggerStateOnGamepad1 = leftTriggerPressedOnGamepad1;
//            lastRightTriggerStateOnGamepad1 = rightTriggerPressedOnGamepad1;
//
//            lastLeftTriggerStateOnGamepad2 = leftTriggerPressedOnGamepad2;
//            lastRightTriggerStateOnGamepad2 = rightTriggerPressedOnGamepad2;
//
//            mecanumDriveGamepadOneJoyStickControlled();
//
//            /* GamePade 1 -> Joystick Controls Starts */
//            if (gamepad1.dpad_up) {
//                // Forward motion [scaled power, slow movement]
//                frontLeftChassisPow = chassisDCMotorPowerScale; frontRightChassisPow = chassisDCMotorPowerScale;
//                backLeftChassisPow = chassisDCMotorPowerScale; backRightChassisPow = chassisDCMotorPowerScale;
//                // Apply powers
//                frontLeftChassisDC.setPower(frontLeftChassisPow);
//                frontRightChassisDC.setPower(frontRightChassisPow);
//                backLeftChassisDC.setPower(backLeftChassisPow);
//                backRightChassisDC.setPower(backRightChassisPow);
//            } else if (gamepad1.dpad_down) {
//                // Backward motion [scaled power, slow movement]
//                frontLeftChassisPow = -chassisDCMotorPowerScale; frontRightChassisPow = -chassisDCMotorPowerScale;
//                backLeftChassisPow = -chassisDCMotorPowerScale; backRightChassisPow = -chassisDCMotorPowerScale;
//                // Apply powers
//                frontLeftChassisDC.setPower(frontLeftChassisPow);
//                frontRightChassisDC.setPower(frontRightChassisPow);
//                backLeftChassisDC.setPower(backLeftChassisPow);
//                backRightChassisDC.setPower(backRightChassisPow);
//            } else if (gamepad1.dpad_left) {
//                // Strafe Left (mecanum) [scaled power, slow movement]
//                frontLeftChassisPow = -chassisDCMotorPowerScale; frontRightChassisPow = chassisDCMotorPowerScale;
//                backLeftChassisPow = chassisDCMotorPowerScale; backRightChassisPow = -chassisDCMotorPowerScale;
//                // Apply powers
//                frontLeftChassisDC.setPower(frontLeftChassisPow);
//                frontRightChassisDC.setPower(frontRightChassisPow);
//                backLeftChassisDC.setPower(backLeftChassisPow);
//                backRightChassisDC.setPower(backRightChassisPow);
//            } else if (gamepad1.dpad_right) {
//                // Strafe Right (mecanum) [scaled power, slow movement]
//                frontLeftChassisPow = chassisDCMotorPowerScale; frontRightChassisPow = -chassisDCMotorPowerScale;
//                backLeftChassisPow = -chassisDCMotorPowerScale; backRightChassisPow = chassisDCMotorPowerScale;
//                // Apply powers
//                frontLeftChassisDC.setPower(frontLeftChassisPow);
//                frontRightChassisDC.setPower(frontRightChassisPow);
//                backLeftChassisDC.setPower(backLeftChassisPow);
//                backRightChassisDC.setPower(backRightChassisPow);
//            } else if (gamepad1.left_bumper) {
//                //This function is used for validation of range only. Not to be used in actual operation.
//                //moveRotatorAssemblyToSuppliedPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_END_POS_FAR, "GamePad 1 Left Bumper clicked!");
//            } else if (gamepad1.right_bumper) {
//                //This function is used for validation of range only. Not to be used in actual operation.
//                //moveRotatorAssemblyToSuppliedPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_START_POS_NEAR, "GamePad 1 Right Bumper clicked!");
//            } else if (gamepad1.x) {
//                moveRotatorAssemblyToSuppliedPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_DEFAULT_POS_NEAR, "GamePad 1 function x clicked!");
//            }
//            else if (gamepad1.b) {
//                moveRotatorAssemblyToSuppliedPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_DEFAULT_POS_FAR, "GamePad 1 function b clicked!");
//            }
//            /* GamePade 1 -> Joystick Controls ends */
//
//            /* GamePade 2 -> Joystick Controls Starts */
//            /* GamePade 2 -> Intake DC Motor movement - START */
//            if (gamepad2.x) {
//                runIntakeMechDCMotor();
//            }
//            /* GamePade 2 -> Intake DC Motor movement - STOP */
//            else if (gamepad2.b) {
//                stopIntakeMechDCMotor();
//            }
//            /* GamePade 2 -> Transfer DC Motor movement - START */
//            else if (gamepad2.y) {
//                runTransferMechDCMotor();
//            }
//            /* GamePade 2 -> Transfer DC Motor movement - STOP */
//            else if (gamepad2.a) {
//                runTransferMechDCMotorInReverse();
//            }
//            /* GamePade 2 -> Shooter DC Motor movement - For Near Region START */
//            else if (gamepad2.dpad_up) {
//                stopShooterDCMotors();
//                sleep(10);
//                runShooterDCMotors(DcMotorConstant.shooterDCMotorPowerScaleFactorNearRegion);
//            }
//            /* GamePade 2 -> Shooter DC Motor movement - STOP */
//            else if (gamepad2.dpad_down) {
//                stopShooterDCMotors();
//            }
//            /* GamePade 2 -> Shooter DC Motor movement - For Far Region START */
//            else if (gamepad2.left_bumper) {
//                stopShooterDCMotors();
//                sleep(10);
//                runShooterDCMotors(DcMotorConstant.shooterDCMotorPowerScaleFactorFarRegion);
//            }
//            /* GamePade 2 -> feeder Servo Motor movement - Release the artifact */
//            else if (gamepad2.right_bumper) {
//                operateFeederServo();
//            }
//            telemetry.update();
//            idle();
//        }
//        limelight.stop();
//    }
//
//    /**
//     * Set chassis DC Motor directions
//     * Default direction is FORWARD
//     */
//    private void setChassisDCMotorsDirection() {
//        frontRightChassisDC.setDirection(REVERSE);
//        backRightChassisDC.setDirection(REVERSE);
//    }
//
//    private void moveRotatorAssemblyToSuppliedPosition(double shooterRotatorServoPosition, String eventSource) {
//        rotorServoCurrentPosition = shooterRotatorServoPosition;
//        telemetry.addData(eventSource, rotorServoCurrentPosition);
//        telemetry.update();
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//    }
//
//    // 🔴 Called once when LEFT TRIGGER is pressed on Gamepad 1
//    private void onLeftTriggerPressedOnGamepad1() {
//        telemetry.addLine("LEFT trigger clicked! - Gamepad 1");
//        telemetry.update();
//        rotorServoCurrentPosition = rotorServoCurrentPosition - STEP;
//        // Limit within defined range
//        rotorServoCurrentPosition = Math.max(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_START_POS_FAR, rotorServoCurrentPosition);// Set new position
//        // Set new position
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//        telemetry.addData("LEFT trigger clicked - Gamepad 1: rotorServoCurrentPosition", rotorServoCurrentPosition);
//        telemetry.update();
//    }
//
//    // 🔴 Called once when RIGHT TRIGGER is pressed on Gamepad 1
//    private void onRightTriggerPressedOnGamepad1() {
//        telemetry.addLine("RIGHT trigger clicked! - Gamepad 1");
//        telemetry.update();
//        rotorServoCurrentPosition = rotorServoCurrentPosition + STEP;
//        // Limit within defined range
//        rotorServoCurrentPosition = Math.min(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_END_POS_FAR, rotorServoCurrentPosition);// Set new position
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//        telemetry.addData("RIGHT trigger clicked - Gamepad 1: rotorServoCurrentPosition", rotorServoCurrentPosition);
//        telemetry.update();
//    }
//
//    // 🔵 Called once when LEFT TRIGGER is pressed on Gamepad 2
//    private void onLeftTriggerPressedOnGamepad2() {
//        telemetry.addLine("LEFT trigger clicked! - Gamepad 2");
//        telemetry.update();
//        rotorServoCurrentPosition = rotorServoCurrentPosition - STEP;
//        // Limit within defined range
//        rotorServoCurrentPosition = Math.max(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_START_POS_NEAR, rotorServoCurrentPosition);// Set new position
//        // Set new position
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//        telemetry.addData("LEFT trigger clicked - Gamepad 2: rotorServoCurrentPosition", rotorServoCurrentPosition);
//        telemetry.update();
//    }
//
//    // 🔴 Called once when RIGHT TRIGGER is pressed on Gamepad 2
//    private void onRightTriggerPressedOnGamepad2() {
//        telemetry.addLine("RIGHT trigger clicked! - Gamepad 2");
//        telemetry.update();
//        rotorServoCurrentPosition = rotorServoCurrentPosition + STEP;
//        // Limit within defined range
//        rotorServoCurrentPosition = Math.min(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_END_POS_NEAR, rotorServoCurrentPosition);// Set new position
//        shooterMechRotatorServo.setPosition(rotorServoCurrentPosition);
//        telemetry.addData("RIGHT trigger clicked - Gamepad 2: rotorServoCurrentPosition", rotorServoCurrentPosition);
//        telemetry.update();
//    }
//
//    private void operateFeederServo() {
//        // Move up to 120°
//        specStopperServo.setDirection(Servo.Direction.FORWARD);
//        if(!stopperButtonStateForFirstTime) {
//            stopperButtonStateForFirstTime = true;
//        } else{
//            specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_OPEN_GATE);
//        }
//        feederEnablerServo.setPosition(ServoMotorConstant.FEEDER_SERVO_POS_120_DEG);
//        telemetry.addData("stopperButtonStateForFirstTime flag:", stopperButtonStateForFirstTime);
//        telemetry.addData("feederEnablerServo Position", feederEnablerServo.getPosition());
//        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
//        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
//        telemetry.update();
//        sleep(400); // adjust as needed for speed
//        specStopperServo.setDirection(Servo.Direction.REVERSE);
//        specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_OPEN_GATE);
//        feederEnablerServo.setPosition(ServoMotorConstant.FEEDER_SERVO_POS_0_DEG);
//        telemetry.addData("feederEnablerServo Position", feederEnablerServo.getPosition());
//        telemetry.addData("specStopperServo Position", specStopperServo.getPosition());
//        telemetry.addData("specStopperServo direction", specStopperServo.getDirection());
//        telemetry.update();
//        sleep(400); // adjust as needed for speed
//    }
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
//        shooterMechRotatorServo.setPosition(ServoMotorConstant.SHOOTER_ROTATOR_SERVO_DEFAULT_POS_NEAR);
//        feederEnablerServo.setPosition(ServoMotorConstant.FEEDER_SERVO_POS_0_DEG);
//        specStopperServo.setPosition(ServoMotorConstant.STOPPER_SERVO_POS_INITIAL_GATE);
//        telemetry.addData("Servo initiateServoMotors:", "Configuration Completed");
//        telemetry.addLine("Use triggers to move shooterMechRotatorServo");
//        telemetry.addLine("Right Trigger → upward movement");
//        telemetry.addLine("Left Trigger → downward movement");
//        telemetry.addData("shooterMechRotatorServo init position", specStopperServo.getPosition());
//        telemetry.addData("shooterMechRotatorServo init direction", specStopperServo.getDirection());
//        telemetry.update();
//    }
//
//    private void mecanumDriveGamepadOneJoyStickControlled() {
//
//        setChassisDCMotorsDirection();
//        double h = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
//        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.left_stick_x;
//        final double v1 = h * Math.sin(robotAngle) - rightX;
//        final double v2 = h * Math.cos(robotAngle) + rightX;
//        final double v3 = h * Math.cos(robotAngle) - rightX;
//        final double v4 = h * Math.sin(robotAngle) + rightX;
//
//        frontLeftChassisDC.setPower(v1);
//        frontRightChassisDC.setPower(v2);
//        backLeftChassisDC.setPower(v3);
//        backRightChassisDC.setPower(v4);
//    }
//
//    // Function to run Shooter DC Motors at scaled power
//    private void runShooterDCMotors(double shooterDCMotorPowerScaleFactor) {
//        leftShooterDC.setDirection(DcMotor.Direction.FORWARD);
//        rightShooterDC.setDirection(DcMotor.Direction.FORWARD);
//        telemetry.addData("DcMotor DirectionLeftShooter", leftShooterDC.getDirection());
//        telemetry.addData("DcMotor DirectionRightShooter", rightShooterDC.getDirection());
//        telemetry.update();
//        startShooterDCMotors(shooterDCMotorPowerScaleFactor);
//    }
//
//    // Function to give scaled power to both shooter DC motors(opposite rotation)
//    private void startShooterDCMotors(double shooterDCMotorPowerScaleFactor) {
//        // Ensure shooterDCPowerScale stays between 0 and 1
//        shooterDCMotorPowerScaleFactor = Math.max(0, Math.min(shooterDCMotorPowerScaleFactor, 1));
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
//        startTransferMechDCMotor(DcMotorConstant.transferDCMotorPowerScale);
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
//        startIntakeMechDCMotor(DcMotorConstant.intakeDCMotorPowerScale);
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
//    // Function to run Transfer DC Motor at scaled power in Reverse direction
//    private void runTransferMechDCMotorInReverse() {
//        transferMechDC.setDirection(DcMotor.Direction.REVERSE);
//        telemetry.addData("DcMotor DirectionTransfer", transferMechDC.getDirection());
//        telemetry.update();
//        startTransferMechDCMotor(DcMotorConstant.transferDCMotorPowerScale);
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
//    private void stopChassisDCMotors() {
//        // Optional but recommended
//        frontLeftChassisDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightChassisDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftChassisDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightChassisDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
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
//}