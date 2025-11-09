package org.firstinspires.ftc.teamcode.apex.warrior;

import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.CHASSIS_BACK_LEFT_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.CHASSIS_BACK_RIGHT_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.CHASSIS_FRONT_LEFT_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.CHASSIS_FRONT_RIGHT_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.FEEDER_ROTATOR_SERVO;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.INTAKE_MECH_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.SHOOTER_LEFT_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.SHOOTER_RIGHT_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.SPEC_STOPPER_SERVO;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.TRANSFER_MECH_DC;
import static org.firstinspires.ftc.teamcode.apex.warrior.RobotHardwareConfigConstant.SHOOTER_ROTATOR_SERVO;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.apex.warrior.auto.GoBildaPinpointDriver;

/**
 * This class is used to store configuration information of all the motors used in the robot.
 * Total 8 DC motors are configured.
 * 4 DC motors are used to control chassis.
 * 2 DC motors are used to control shooting mechanism.
 * 1 DC motor is used to control transfer mechanism.
 * 1 DC motor is used to control intake mechanism.
 * Total 2 Servo motors are configure
 * 1 servo motor is used to control rotating mechanism.
 * 1 servo motor is used to help in positioning artifact for shooting mechanism.
 * 1 servo motor is used to stop artifact and prevent it from reaching it to shooting mechanism.
 * 1 GoBilda Pinpoint Driver used for odometry.
 */
public class RobotHardwareConfigurator {

    //DC motors used in chassis
    private DcMotor frontLeftChassisDC;
    private DcMotor frontRightChassisDC;
    private DcMotor backLeftChassisDC;
    private DcMotor backRightChassisDC;

    //Other DC motors used
    private DcMotor leftShooterDC;
    private DcMotor rightShooterDC;
    private DcMotor transferMechDC;
    private DcMotor intakeMechDC;

    //Servo motors used
    private Servo shooterMechRotatorServo;
    private Servo feederEnablerServo;
    private Servo specStopperServo;

    private GoBildaPinpointDriver odo;

    /* Initialize the hardware variables. Note that the strings used here as parameters
     * which must should correspond to the names assigned during the robot configuration
     * step (using the FTC Robot Controller app on the Device).
     */
    public void configureRobotHardware(HardwareMap hardwareMap) {

        //chassis_DC_motor - 4, Control Hub
        frontLeftChassisDC = hardwareMap.dcMotor.get(CHASSIS_FRONT_LEFT_DC);       //chassis_DC_motor - 1, Control Hub- Port 0
        frontRightChassisDC = hardwareMap.dcMotor.get(CHASSIS_FRONT_RIGHT_DC);     //chassis_DC_motor - 2, Control Hub- Port 1
        backLeftChassisDC = hardwareMap.dcMotor.get(CHASSIS_BACK_LEFT_DC);         //chassis_DC_motor - 3, Control Hub- Port 2
        backRightChassisDC = hardwareMap.dcMotor.get(CHASSIS_BACK_RIGHT_DC);       //chassis_DC_motor - 4, Control Hub- Port 3

        //chassis_DC_motor - 4, Control Hub
        leftShooterDC = hardwareMap.dcMotor.get(SHOOTER_LEFT_DC);                  //leftShooterDC , Expansion Hub- Port 0
        rightShooterDC = hardwareMap.dcMotor.get(SHOOTER_RIGHT_DC);                //rightShooterDC, Expansion Hub- Port 1
        transferMechDC = hardwareMap.dcMotor.get(TRANSFER_MECH_DC);                //transferMechDC, Expansion Hub- Port 2
        intakeMechDC = hardwareMap.dcMotor.get(INTAKE_MECH_DC);                    //intakeMechDC, Expansion Hub- Port 3

        //servo - 3, Control Hub
        shooterMechRotatorServo = hardwareMap.servo.get(SHOOTER_ROTATOR_SERVO);    //shooterMechRotatorServo, Control Hub- Port 0
        feederEnablerServo = hardwareMap.servo.get(FEEDER_ROTATOR_SERVO);          //feederEnablerServo, Control Hub- Port 1
        specStopperServo = hardwareMap.servo.get(SPEC_STOPPER_SERVO);              //specStopperServo, Control Hub- Port 2
        //Sensor odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");       // odo, Control Hub- Port 0

    }

    public DcMotor getFrontRightChassisDC() {
        return frontRightChassisDC;
    }

    public DcMotor getFrontLeftChassisDC() {
        return frontLeftChassisDC;
    }

    public DcMotor getBackLeftChassisDC() {
        return backLeftChassisDC;
    }

    public DcMotor getBackRightChassisDC() {
        return backRightChassisDC;
    }

    public DcMotor getLeftShooterDC() {
        return leftShooterDC;
    }

    public DcMotor getRightShooterDC() {
        return rightShooterDC;
    }

    public DcMotor getTransferMechDC() {
        return transferMechDC;
    }

    public DcMotor getIntakeMechDC() {
        return intakeMechDC;
    }

    public Servo getShooterMechRotatorServo() {
        return shooterMechRotatorServo;
    }

    public Servo getFeederEnablerServo() {
        return feederEnablerServo;
    }

    public Servo getSpecStopperServo() {
        return specStopperServo;
    }

    public GoBildaPinpointDriver getOdo() {
        return odo;
    }
}