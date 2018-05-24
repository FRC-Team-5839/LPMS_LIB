package com.team5839.lpms;

public class IMUProtocol {

	
	public static float decodebyte2float(byte[] b) {
		int buff;                                             
	    buff = b[0];                                  
	    buff &= 0xff;                                         
	    buff |= ((long) b[1] << 8);                   
	    buff &= 0xffff;                                       
	    buff |= ((long) b[2] << 16);                  
	    buff &= 0xffffff;                                     
	    buff |= ((long) b[3] << 24);                  
	    return Float.intBitsToFloat(buff);    
		
	                   
	}  
	
	public static float decodebyte2float(byte[] byte1, byte[] byte2, byte[] byte3, byte[] byte4) {
		byte[] b = {byte1[0], byte2[0], byte3[0], byte4[0]};
	    int buff;                                             
	    buff = b[0];                                  
	    buff &= 0xff;                                         
	    buff |= ((long) b[1] << 8);                   
	    buff &= 0xffff;                                       
	    buff |= ((long) b[2] << 16);                  
	    buff &= 0xffffff;                                     
	    buff |= ((long) b[3] << 24);                  
	    return Float.intBitsToFloat(buff);                    
	}  
    
    public static byte decodebit2Byte(String bit) {  
        int goal, len;  
        if (null == bit) {  
            return 0;  
        }  
        len = bit.length();  
        if (len != 4 && len != 8) {  
            return 0;  
        }  
        if (len == 8) { 
            if (bit.charAt(0) == '0') {
                goal = Integer.parseInt(bit, 2);  
            } else { 
                goal = Integer.parseInt(bit, 2) - 256;  
            }  
        } else {  
            goal = Integer.parseInt(bit, 2);  
        }  
        return (byte) goal;  
    }
    
   public static byte[] decodebyte2bitarray(byte b) {  
       byte[] array = new byte[8];  
       for (int i = 7; i >= 0; i--) {  
           array[i] = (byte)(b & 1);  
           b = (byte) (b >> 1);  
       }  
       return array;  
   }  

   public static String decodebyte2Bitstr(byte b) {  
       return ""  
               + (byte) ((b >> 7) & 0x1) + (byte) ((b >> 6) & 0x1)  
               + (byte) ((b >> 5) & 0x1) + (byte) ((b >> 4) & 0x1)  
               + (byte) ((b >> 3) & 0x1) + (byte) ((b >> 2) & 0x1)  
               + (byte) ((b >> 1) & 0x1) + (byte) ((b >> 0) & 0x1);  
   }
}
