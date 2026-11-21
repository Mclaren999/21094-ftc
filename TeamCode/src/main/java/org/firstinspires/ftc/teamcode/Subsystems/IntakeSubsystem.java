package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class IntakeSubsystem {

    /* ================= HARDWARE ================= */

    private final DcMotorEx intake;
    private final DcMotorEx intake2;
    private final DcMotorEx transfer;
    private final Servo flower;

    /* ================= STATES ================= */

    public enum IntakeState {
        INTAKE,
        TRANSFER,
        REVERSE,
        OFF       // полная остановка (всё в 0)
    }

    public enum FlowerState {
        OPEN,
        CLOSED
    }

    private IntakeState istate = IntakeState.OFF;
    private FlowerState flowerState = FlowerState.CLOSED;

    /* ================= CONSTANTS ================= */
    // Не final — чтобы их можно было менять через Panels (@Configurable)

    // Flower servo
    public static double FLOWER_CLOSED = 0;
    public static double FLOWER_OPEN   = 1;

    // Знак мощности: + = мячи едут внутрь / к шутеру, - = наружу.
    // Если на роботе мотор крутит не туда, меняйте его Direction ниже, а не эти числа.

    // INTAKE: интейк тянет внутрь, transfer крутится назад и не пускает мячи к шутеру
    public static double INTAKE_INTAKE_POWER   = -1.0;
    public static double INTAKE_TRANSFER_POWER = -1.0;

    // TRANSFER: всё в одну сторону — мячи идут в шутер
    public static double TRANSFER_INTAKE_POWER   = -1;
    public static double TRANSFER_TRANSFER_POWER = 0.8;

    // REVERSE: всё в обратную сторону — мячи выбрасываются
    public static double REVERSE_INTAKE_POWER   = 1.0;  // противоположно INTAKE_INTAKE_POWER (-1), иначе реверс = обычный интейк
    public static double REVERSE_TRANSFER_POWER = -1.0;

    /* ================= INIT ================= */

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorEx.Direction.FORWARD);

        // Моторы интейка стоят друг напротив друга, поэтому второй развёрнут:
        // при одинаковой мощности оба тянут мячи в одну сторону.
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        intake2.setDirection(DcMotorEx.Direction.REVERSE);

        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorEx.Direction.FORWARD);

        flower = hardwareMap.get(Servo.class, "flower");

        intake.setPower(0);
        intake2.setPower(0);
        transfer.setPower(0);
        flower.setPosition(FLOWER_CLOSED);
    }

    /* ================= COMMANDS ================= */

    public void intake()   { istate = IntakeState.INTAKE; }
    public void transfer() { istate = IntakeState.TRANSFER; }
    public void reverse()  { istate = IntakeState.REVERSE; }
    public void stop()     { istate = IntakeState.OFF; }
    public void off()      { istate = IntakeState.OFF; }

    public void flowerO() { flowerState = FlowerState.OPEN; }
    public void flowerC() { flowerState = FlowerState.CLOSED; }

    public void overIntake() {
        // сюда можно добавить сенсор
    }

    public IntakeState getState() { return istate; }
    public FlowerState getFlowerState() { return flowerState; }

    /* ================= UPDATE ================= */

    public void update() {
        switch (istate) {
            case INTAKE:
                setOutputs(INTAKE_INTAKE_POWER, INTAKE_TRANSFER_POWER);
                break;

            case TRANSFER:
                setOutputs(TRANSFER_INTAKE_POWER, TRANSFER_TRANSFER_POWER);
                break;

            case REVERSE:
                setOutputs(REVERSE_INTAKE_POWER, REVERSE_TRANSFER_POWER);
                break;

            case OFF:
            default: // на всякий случай — безопасная остановка
                setOutputs(0, 0);
                break;
        }

        flower.setPosition(flowerState == FlowerState.OPEN ? FLOWER_OPEN : FLOWER_CLOSED);
    }

    private void setOutputs(double intakePower, double transferPower) {
        intake.setPower(intakePower);
        intake2.setPower(intakePower);
        transfer.setPower(transferPower);
    }
}
