package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Calendar;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Bell", group = "Autonomous")
public class Bell extends OpMode {


    private DcMotor l;
    private DcMotor r;
    private DcMotor b;
    private IMU imu;
    
    private boolean[] ran = new boolean[4];
    private int numberRan = 0;
    
    private ElapsedTime moveTimer = new ElapsedTime();
    private boolean isMoving = false;
    private int direction = 1;
    
    enum State { WAITING, RUNNING_ACTION, DONE }

    State state = State.WAITING;

    @Override
    public void init() {
        l = hardwareMap.dcMotor.get("l");
        r = hardwareMap.dcMotor.get("r");
        b = hardwareMap.dcMotor.get("b");
        
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        l.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setDirection(DcMotorSimple.Direction.FORWARD);
        b.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    

    @Override
    public void loop() {
        Calendar now = Calendar.getInstance();
        int hour = now.get(Calendar.HOUR_OF_DAY);
        int minute = now.get(Calendar.MINUTE);
        int second = now.get(Calendar.SECOND);
        telemetry.addData("State", state);
        telemetry.addData("Time", String.format("%02d:%02d:%02d", hour, minute, second));
        telemetry.addData("numberRan", numberRan);
        telemetry.update();
        switch (state) {
            case WAITING:
                if (hasReachedTime(hour, minute, second, 19, 46, 0) && (!ran[0])
                || hasReachedTime(hour, minute, second, 19, 46, 30) && (!ran[1])
                || hasReachedTime(hour, minute, second, 19, 47, 0) && (!ran[2])
                || hasReachedTime(hour, minute, second, 19, 47, 30) && (!ran[3])) {
                    state = State.RUNNING_ACTION;
                }
                break;

            case RUNNING_ACTION:
                if (!isMoving) {
                    moveTimer.reset();
                    isMoving = true;
                }
            
                if (moveTimer.seconds() < 10.0) {
                    double correction = getCorrection();
                    double basePower = 0.25;
            
                    double leftPower = (basePower - correction * direction) * direction;
                    double rightPower = (basePower + correction * direction) * direction;
            
                    l.setPower(leftPower);
                    r.setPower(rightPower);
                    b.setPower(-1);
                } else {
                    l.setPower(0);
                    r.setPower(0);
                    b.setPower(0);
                    ran[numberRan] = true;
                    numberRan++;
                    direction *= -1;
                    if (numberRan == 4) {
                        state = State.DONE;
                    } else {
                        state = State.WAITING;
                    }
                    isMoving = false;
                }
                break;
            case DONE:
                break;
        }
    }
    
    private boolean hasReachedTime(int hNow, int mNow, int sNow, int hTarget, int mTarget, int sTarget) {
        return hNow > hTarget ||
               (hNow == hTarget && (mNow > mTarget || (mNow == mTarget && sNow >= sTarget)));
    }

    private double getYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private double getCorrection() {
        double yaw = getYaw();
        double gain = 0.01;
        double bias = -0.07;
        if (direction == -1) bias = 0.07;
        return -yaw * gain + bias;
    }
}
