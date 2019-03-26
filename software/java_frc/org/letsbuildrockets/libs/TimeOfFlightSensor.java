package org.letsbuildrockets.libs;

import org.letsbuildrockets.libs.CustomCAN;
import org.letsbuildrockets.libs.VersionNumber;
/**
 * TimeOfFlightSensor
 * V1.1
 */
public class TimeOfFlightSensor {


    // Control Bytes
    private static final byte CTRL_SEND_ERROR = 1;
    private static final byte CTRL_SEND_DISTANCE = 2;
    private static final byte CTRL_SET_NEW_ADDR = 3;
    private static final byte CTRL_GET_FIRMWARE_VERSION = 4;
    private static final byte CTRL_SEND_FIRMWARE_VERSION = 5;

    // Error codes
    private static final byte ERROR_NONE = 0;
    private static final byte ERROR_OUT_OF_RANGE = 1;
    private static final byte ERROR_WRITING_TO_CAN = 2;
    private static final byte ERROR_INIT_CAN = 3;
    private static final byte ERROR_INIT_VL53L0X = 4;
    private static final byte ERROR_BAD_CTRL_BYTE = 5;
    private static final byte ERROR_NOT_ENOUGH_DATA_BYTES = 6;


    int _ID, _distance, _error;
    VersionNumber _firmwareVersion;
    CustomCAN tofsensor;
    static int TOFCount = 0;

    public TimeOfFlightSensor(int ID) {
        tofsensor = new CustomCAN("TOF"+String.valueOf(TOFCount), ID);
        _ID = ID;
        TOFCount++;
        sendByte(CTRL_GET_FIRMWARE_VERSION);
        readBuffer();
    }

    private void readBuffer() {
        try {
            byte dat[] = tofsensor.readSafely();
            // for (byte byteme : dat) {
                // System.out.printf("rec: 0x%02x\n", byteme);
            // }
            switch (dat[0]) {
                case CTRL_SEND_ERROR:
                    if(dat.length == 2){
                        _error = dat[1];
                    }
                    break;
                case CTRL_SEND_DISTANCE:
                    if(dat.length == 3) {
                        _error = dat[0] & 0xFF;
                        _distance = (dat[1]&0xFF) << 8 | (dat[2]&0xFF);
                        // System.out.println("distance: " + _distance);
                    }
                    break;
                case CTRL_SEND_FIRMWARE_VERSION:
                    if(dat.length == 3){
                        _firmwareVersion.major = dat[1];
                        _firmwareVersion.minor = dat[2];
                    }
                    break;
            
                default:
                    break;
            }
        } catch (CANMessageUnavailableException e) {
            //System.out.println("CAN error "+e.getMessage());
        }
    }

    private void sendByte(byte byteme) {
        byte dat[] = new byte[8];
        dat[0] = byteme;
        tofsensor.writeSafely(dat);
    }

    public void setHarwareCANAddress(short newID) {
        if(newID < 0x0620) {
            System.err.println("The new address for " + tofsensor.getName() + " must be >= 0x0620!");
            return;
        }
        if(newID > 0x0FFF) {
            System.err.println("The new address for " + tofsensor.getName() + " must be <= 0x0FFF!");
            return;
        }
        byte dat[] = new byte[8];
        dat[0] = CTRL_SET_NEW_ADDR;
        dat[1] = (byte)((newID >> 8) & 0xFF);
        dat[2] = (byte)(newID & 0xFF);
        tofsensor.writeSafely(dat);
    }

    public VersionNumber getFirwareVersion() {
        sendByte(CTRL_GET_FIRMWARE_VERSION);
        readBuffer();
        return _firmwareVersion;
    }

    public int getDistance() {
        readBuffer();
        return _distance;
    }

    public int getError() {
        readBuffer();
        return _error;
    }

    public boolean inRange() {
        readBuffer();
        return (_error == 0);
    }

}