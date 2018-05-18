package com.team5839.lpms;

public class IMURegisters {
	
	/**********************************************/
	/* Device Status and Control Registers        */
	/**********************************************/
	
	public static final byte LPMS_FUN_CONFIG				= 0x00;
	public static final byte LPMS_SYS_CONFIG				= 0x01;
	public static final byte LPMS_DATA_CTRL				= 0x02;
	public static final byte LPMS_DATA_ENABLE			= 0x03;
	public static final byte LPMS_CTRL_0_A				= 0x04;
	public static final byte LPMS_CTRL_1_G				= 0x05;
	public static final byte LPMS_CTRL_2_M				= 0x06;
	public static final byte LPMS_STATUS					= 0x07;
	public static final byte LPMS_FILTER_CONFIG			= 0x08;
	
	/**********************************************/
	/* Device Timestamp Registers     		      */
	/**********************************************/
	
	public static final byte LPMS_TIMESTAMP_0			= 0x20;
	public static final byte LPMS_TIMESTAMP_1			= 0x21;
	public static final byte LPMS_TIMESTAMP_2			= 0x22;
	public static final byte LPMS_TIMESTAMP_3			= 0x23;
	
	/**********************************************/
	/* Device Raw Data Registers     		      */
	/**********************************************/
	
	public static final byte LPMS_ACC_X_0				= 0x24;
	public static final byte LPMS_ACC_X_1				= 0x25;
	public static final byte LPMS_ACC_X_2				= 0x26;
	public static final byte LPMS_ACC_X_3				= 0x27;
	public static final byte LPMS_ACC_Y_0				= 0x28;
	public static final byte LPMS_ACC_Y_1				= 0x29;
	public static final byte LPMS_ACC_Y_2				= 0x2A;
	public static final byte LPMS_ACC_Y_3				= 0x2B;
	public static final byte LPMS_ACC_Z_0				= 0x2C;
	public static final byte LPMS_ACC_Z_1				= 0x2D;
	public static final byte LPMS_ACC_Z_2				= 0x2E;
	public static final byte LPMS_ACC_Z_3				= 0x2F;
	public static final byte LPMS_GYR_X_0				= 0x30;
	public static final byte LPMS_GYR_X_1				= 0x31;
	public static final byte LPMS_GYR_X_2				= 0x32;
	public static final byte LPMS_GYR_X_3				= 0x33;
	public static final byte LPMS_GYR_Y_0				= 0x34;
	public static final byte LPMS_GYR_Y_1				= 0x35;
	public static final byte LPMS_GYR_Y_2				= 0x36;
	public static final byte LPMS_GYR_Y_3				= 0x37;
	public static final byte LPMS_GYR_Z_0				= 0x38;
	public static final byte LPMS_GYR_Z_1				= 0x39;
	public static final byte LPMS_GYR_Z_2				= 0x3A;
	public static final byte LPMS_GYR_Z_3				= 0x3B;
	public static final byte LPMS_MAG_X_0				= 0x3C;
	public static final byte LPMS_MAG_X_1				= 0x3D;
	public static final byte LPMS_MAG_X_2				= 0x3E;
	public static final byte LPMS_MAG_X_3				= 0x3F;
	public static final byte LPMS_MAG_Y_0				= 0x40;
	public static final byte LPMS_MAG_Y_1				= 0x41;
	public static final byte LPMS_MAG_Y_2				= 0x42;
	public static final byte LPMS_MAG_Y_3				= 0x43;
	public static final byte LPMS_MAG_Z_0				= 0x44;
	public static final byte LPMS_MAG_Z_1				= 0x45;
	public static final byte LPMS_MAG_Z_2				= 0x46;
	public static final byte LPMS_MAG_Z_3				= 0x47;
	
	/**********************************************/
	/* Processed Data Registers                   */
	/**********************************************/
	
	public static final byte LPMS_EULER_X_0				= 0x48;
	public static final byte LPMS_EULER_X_1				= 0x49;
	public static final byte LPMS_EULER_X_2				= 0x4A;
	public static final byte LPMS_EULER_X_3				= 0x4B;
	public static final byte LPMS_EULER_Y_0				= 0x4C;
	public static final byte LPMS_EULER_Y_1				= 0x4D;
	public static final byte LPMS_EULER_Y_2				= 0x4E;
	public static final byte LPMS_EULER_Y_3				= 0x4F;
	public static final byte LPMS_EULER_Z_0				= 0x50;
	public static final byte LPMS_EULER_Z_1				= 0x51;
	public static final byte LPMS_EULER_Z_2				= 0x52;
	public static final byte LPMS_EULER_Z_3				= 0x53;
	public static final byte LPMS_QUAT_W_0				= 0x54;
	public static final byte LPMS_QUAT_W_1				= 0x55;
	public static final byte LPMS_QUAT_W_2				= 0x56;
	public static final byte LPMS_QUAT_W_3				= 0x57;
	public static final byte LPMS_QUAT_X_0				= 0x58;
	public static final byte LPMS_QUAT_X_1				= 0x59;
	public static final byte LPMS_QUAT_X_2				= 0x5A;
	public static final byte LPMS_QUAT_X_3				= 0x5B;
	public static final byte LPMS_QUAT_Y_0				= 0x5C;
	public static final byte LPMS_QUAT_Y_1				= 0x5D;
	public static final byte LPMS_QUAT_Y_2				= 0x5E;
	public static final byte LPMS_QUAT_Y_3				= 0x5F;
	public static final byte LPMS_QUAT_Z_0				= 0x60;
	public static final byte LPMS_QUAT_Z_1				= 0x61;
	public static final byte LPMS_QUAT_Z_2				= 0x62;
	public static final byte LPMS_QUAT_Z_3				= 0x63;
	public static final byte LPMS_LIN_ACC_X_0			= 0x64;
	public static final byte LPMS_LIN_ACC_X_1			= 0x65;
	public static final byte LPMS_LIN_ACC_X_2			= 0x66;
	public static final byte LPMS_LIN_ACC_X_3			= 0x67;
	public static final byte LPMS_LIN_ACC_Y_0			= 0x68;
	public static final byte LPMS_LIN_ACC_Y_1			= 0x69;
	public static final byte LPMS_LIN_ACC_Y_2			= 0x6A;
	public static final byte LPMS_LIN_ACC_Y_3			= 0x6B;
	public static final byte LPMS_LIN_ACC_Z_0			= 0x6C;
	public static final byte LPMS_LIN_ACC_Z_1			= 0x6D;
	public static final byte LPMS_LIN_ACC_Z_2			= 0x6E;
	public static final byte LPMS_LIN_ACC_Z_3			= 0x6F;
	public static final byte LPMS_TEMP_0					= 0x70;
	public static final byte LPMS_TEMP_1					= 0x71;
	public static final byte LPMS_TEMP_2					= 0x72;
	public static final byte LPMS_TEMP_3					= 0x73;
	
	/**********************************************/
	/* Device Identification Registers            */
	/**********************************************/
	
	public static final byte LPMS_WHO_AM_I				= 0x74;
	public static final byte LPMS_FIRMWARE_VERSION_0		= 0x75;
	public static final byte LPMS_FIRMWARE_VERSION_1		= 0x76;

	


	

	

}
