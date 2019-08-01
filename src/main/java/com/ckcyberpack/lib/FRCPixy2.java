package com.ckcyberpack.lib;

import edu.wpi.first.wpilibj.SPI;

public class FRCPixy2 {

    private SPI pixy;
    private static final int pixySPIClock = 1000000; //2MHZ maximum from Arduino Spec

    //Command Bytes
    static final byte PIXYSTARTNOCHECK1 = (byte)0xae;
    static final byte PIXYSTARTNOCHECK2 = (byte)0xc1;

    static final byte PIXY00 = (byte)0x00;

    static final byte PIXY_TYPE_REQUEST_VERSION = (byte)0x0e;
    static final byte PIXY_TYPE_RESPONSE_VERSION = (byte) 0x0f;

    static final byte PIXY_CCC_RESPONSE_BLOCKS = (byte) 0x21;
    static final byte PIXY_CCC_REQUEST_BLOCKS = (byte) 0x20;

    public enum PixyCommands {
        VERSION,
        GETBLOCKS
    }

    //Assume Port 0 if none supplied
    public FRCPixy2() {
        this(SPI.Port.kMXP);
    }

    public FRCPixy2(SPI.Port port) {
        pixy = new SPI(port);

        //Setup as per Pixycam SPI spec
        pixy.setClockRate(pixySPIClock); //Set Clockrate, FRC = 500khz default
        pixy.setMSBFirst(); //Most Significant Bit First
        pixy.setClockActiveHigh(); //SPI SCK is low when idle
        pixy.setChipSelectActiveLow(); //Slave Select is LOW

    }


    private byte[] sendCommand(FRCPixy2.PixyCommands pCommand) {
        byte[] sendBytes = new byte[17];
        byte[] receiveBytes = new byte[17];

        switch (pCommand) {
            case VERSION:

                sendBytes[0] = PIXYSTARTNOCHECK1;
                sendBytes[1] = PIXYSTARTNOCHECK2;
                sendBytes[2] = PIXY_TYPE_REQUEST_VERSION;
                sendBytes[3] = PIXY00;
        }

        pixy.transaction(sendBytes, receiveBytes, sendBytes.length);
        for (int i = 0; i < receiveBytes.length; i++){
            System.out.println(String.format("%02X ", receiveBytes[i]));
        }

        return receiveBytes;
    }


    public FRCPixyVersion checkVersion() {
        System.out.println("Pixy - check version");

        byte[] response = sendCommand(PixyCommands.VERSION);

        if (response[2] == PIXY_TYPE_RESPONSE_VERSION) {
            int major = response[8] & 0xff;
            int minor = response[9] & 0xff;
            int build = (response[11] & 0xff) + (response[10] & 0xff);
            String fwType = "";

            for (int i = 0; i < 10; i++) {
                fwType = fwType + (char) (response[10 + i] & 0xff);
            }

            return new FRCPixyVersion(major, minor, build, fwType);
        }
        else {
                return null; //Noresponse
        }

    }

    public FRCPixyBlock getBlocks(int sigmap){
        return getBlocks(sigmap, 2);
    }

    public FRCPixyBlock getBlocks(int sigmap, int maxBlocks){
        System.out.println("Pixy - get blocks");

        byte[] response = sendCommand(PixyCommands.GETBLOCKS);

        if (response[2] == PIXY_CCC_RESPONSE_BLOCKS){
            int x = ((response[8] & 0xff) << 8) | (response[7] & 0xff);
            int y = ((response[10] & 0xff) << 8) | (response[9] & 0xff);
            int width = ((response[12] & 0xff) << 8) | (response[11] & 0xff);
            int height = ((response[14] & 0xff) << 8) | (response[13] & 0xff);
            int idx = (response[16] & 0xff);
            int age = (response[17] & 0xff);

            return new FRCPixyBlock(x,y,width,height,idx,age);
        }
        else{
            return null;
        }
    }
}
