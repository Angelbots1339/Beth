package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

/**
 * Doing lots of printing in Java creates a large overhead
 * This Instrument class is designed to put that printing in a seperate thread
 * That way we can prevent loop overrun messages from occurring
 */
class Instrument extends Thread {
    private final CANCoder _CANCoder;

    public Instrument(CANCoder coder) {
        this._CANCoder = coder;
    }
    
    void printFaults(CANCoderFaults faults) {
        System.out.printf("Hardware fault: %s\t    Under Voltage fault: %s\t    Reset During Enable fault: %s\t    API Error fault: %s%n",
                faults.HardwareFault ? "True " : "False",
                faults.UnderVoltage ? "True " : "False",
                faults.ResetDuringEn ? "True " : "False",
                faults.APIError ? "True " : "False");
    }
    void printFaults(CANCoderStickyFaults faults) {
        System.out.printf("Hardware fault: %s\t    Under Voltage fault: %s\t    Reset During Enable fault: %s\t     API Error fault: %s%n",
                faults.HardwareFault ? "True " : "False",
                faults.UnderVoltage ? "True " : "False",
                faults.ResetDuringEn ? "True " : "False",
                faults.APIError ? "True " : "False");
    }
    void printValue(double val, String units, double timestamp) {
        System.out.printf("%20f %-20s @ %f%n", val, units, timestamp);
    }
    void printValue(MagnetFieldStrength val, String units, double timestamp) {
        System.out.printf("%20s %-20s @ %f%n", val.toString(), units, timestamp);
    }

    public void run() {
        /* Report position, absolute position, velocity, battery voltage */
        double posValue = _CANCoder.getPosition();
        String posUnits = _CANCoder.getLastUnitString();
        double posTstmp = _CANCoder.getLastTimestamp();

        double absValue = _CANCoder.getAbsolutePosition();
        String absUnits = _CANCoder.getLastUnitString();
        double absTstmp = _CANCoder.getLastTimestamp();

        double velValue = _CANCoder.getVelocity();
        String velUnits = _CANCoder.getLastUnitString();
        double velTstmp = _CANCoder.getLastTimestamp();

        double batValue = _CANCoder.getBusVoltage();
        String batUnits = _CANCoder.getLastUnitString();
        double batTstmp = _CANCoder.getLastTimestamp();

        /* Report miscellaneous attributes about the CANCoder */
        MagnetFieldStrength magnetStrength = _CANCoder.getMagnetFieldStrength();
        String magnetStrengthUnits = _CANCoder.getLastUnitString();
        double magnetStrengthTstmp = _CANCoder.getLastTimestamp();

        System.out.print("Position: ");
        printValue(posValue, posUnits, posTstmp);
        System.out.print("Abs Pos : ");
        printValue(absValue, absUnits, absTstmp);
        System.out.print("Velocity: ");
        printValue(velValue, velUnits, velTstmp);
        System.out.print("Battery : ");
        printValue(batValue, batUnits, batTstmp);
        System.out.print("Strength: ");
        printValue(magnetStrength, magnetStrengthUnits, magnetStrengthTstmp);

        /* Fault reporting */
        CANCoderFaults faults = new CANCoderFaults();
        _CANCoder.getFaults(faults);
        CANCoderStickyFaults stickyFaults = new CANCoderStickyFaults();
        _CANCoder.getStickyFaults(stickyFaults);

        System.out.println("Faults:");
        printFaults(faults);
        System.out.println("Sticky Faults:");
        printFaults(stickyFaults);

        System.out.println();
        System.out.println();
    }
}
