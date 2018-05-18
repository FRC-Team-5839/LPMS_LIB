package com.team5839.lpms;

public class IMUConfig {
	
	/**********************************************/
	/* Device SYS_CONFIG Data	    		      */
	/**********************************************/
	public static final byte LPMS_TURN_ON_LED 				= IMUProtocol.decodebit2Byte("10000000");	
	public static final byte LPMS_TURN_OFF_LED				= IMUProtocol.decodebit2Byte("00000000");
	
	/**********************************************/
	/* Device DATA_CTRL Data	    		      */
	/**********************************************/
	
	public static final byte LPMS_5Hz						= IMUProtocol.decodebit2Byte("00000000");
	public static final byte LPMS_10Hz						= IMUProtocol.decodebit2Byte("10000000");
	public static final byte LPMS_50Hz						= IMUProtocol.decodebit2Byte("01000000");
	public static final byte LPMS_100Hz						= IMUProtocol.decodebit2Byte("11000000");
	public static final byte LPMS_200Hz						= IMUProtocol.decodebit2Byte("00100000");
	public static final byte LPMS_400Hz						= IMUProtocol.decodebit2Byte("10100000");
	public static final byte LPMS_5Hz_RESET					= IMUProtocol.decodebit2Byte("00000100");
	public static final byte LPMS_10Hz_RESET				= IMUProtocol.decodebit2Byte("10000100");
	public static final byte LPMS_50Hz_RESET				= IMUProtocol.decodebit2Byte("01000100");
	public static final byte LPMS_100Hz_RESET				= IMUProtocol.decodebit2Byte("11000100");
	public static final byte LPMS_200Hz_RESET				= IMUProtocol.decodebit2Byte("00100100");
	public static final byte LPMS_400Hz_RESET				= IMUProtocol.decodebit2Byte("10100100");
	
	/**********************************************/
	/* Device DATA_ENABLE Data	    		      */
	/**********************************************/
	
	public static final byte LPMS_TEMP						= IMUProtocol.decodebit2Byte("00000001");
	public static final byte LPMS_LIN_ACC					= IMUProtocol.decodebit2Byte("00000010");
	public static final byte LPMS_QUAT						= IMUProtocol.decodebit2Byte("00000100");
	public static final byte LPMS_EULER						= IMUProtocol.decodebit2Byte("00001000");
	public static final byte LPMS_MAG						= IMUProtocol.decodebit2Byte("00010000");
	public static final byte LPMS_GYRO						= IMUProtocol.decodebit2Byte("00100000");
	public static final byte LPMS_ACC						= IMUProtocol.decodebit2Byte("01000000");
	public static final byte LPMS_TIMESTAMP					= IMUProtocol.decodebit2Byte("10000000");
	
	/**********************************************/
	/* Device CTRL_0_A Data		    		      */
	/**********************************************/
	
	public static final byte LPMS_ACC_2G					= IMUProtocol.decodebit2Byte("00000000");
	public static final byte LPMS_ACC_16G					= IMUProtocol.decodebit2Byte("00100000");
	public static final byte LPMS_ACC_4G					= IMUProtocol.decodebit2Byte("00010000");
	public static final byte LPMS_ACC_8G					= IMUProtocol.decodebit2Byte("00110000");
	
	/**********************************************/
	/* Device CTRL_1_G Data		    		      */
	/**********************************************/
	
	public static final byte LPMS_CALIBRATING_GYRO			= IMUProtocol.decodebit2Byte("00000001");
	public static final byte LPMS_GYRO_245dps				= IMUProtocol.decodebit2Byte("00000000");
	public static final byte LPMS_GYRO_500dps				= IMUProtocol.decodebit2Byte("00100000");
	public static final byte LPMS_GYRO_1000dps				= IMUProtocol.decodebit2Byte("00010000");
	public static final byte LPMS_GYRO_2000dps				= IMUProtocol.decodebit2Byte("00110000");
	public static final byte LPMS_GYRO_125dps				= IMUProtocol.decodebit2Byte("01000000");
	
	/**********************************************/
	/* Device CTRL_1_G Data		    		      */
	/**********************************************/
	
	public static final byte LPMS_CALIBRATING_MAG			= IMUProtocol.decodebit2Byte("00000001");
	public static final byte LPMS_MAG_4gauss				= IMUProtocol.decodebit2Byte("00000000");
	public static final byte LPMS_MAG_8gauss				= IMUProtocol.decodebit2Byte("00000100");
	public static final byte LPMS_MAG_12gauss				= IMUProtocol.decodebit2Byte("00000010");
	public static final byte LPMS_MAG_16gauss				= IMUProtocol.decodebit2Byte("00000110");
	
	/**********************************************/
	/* Device FILTER_CONFIG Data 				  */
	/* LPFC=Low pass filter coefficient		  	  */
	/**********************************************/
	
	public static final byte LPMS_LPFC_NONE_GYR_ACC_MAG		= IMUProtocol.decodebit2Byte("01000000");
	public static final byte LPMS_LPFC_NONE_GYR_ACC			= IMUProtocol.decodebit2Byte("10000000");
	

}
