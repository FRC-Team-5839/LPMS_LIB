package com.team5839.lpms;

public class IMUProtocol {

	
	public static float decodebyte2float(byte[] b, int index) {    
	    int l;                                             
	    l = b[index + 0];                                  
	    l &= 0xff;                                         
	    l |= ((long) b[index + 1] << 8);                   
	    l &= 0xffff;                                       
	    l |= ((long) b[index + 2] << 16);                  
	    l &= 0xffffff;                                     
	    l |= ((long) b[index + 3] << 24);                  
	    return Float.intBitsToFloat(l);                    
	}  
    
    public static byte decodebit2Byte(String bit) {  
        int re, len;  
        if (null == bit) {  
            return 0;  
        }  
        len = bit.length();  
        if (len != 4 && len != 8) {  
            return 0;  
        }  
        if (len == 8) { 
            if (bit.charAt(0) == '0') {
                re = Integer.parseInt(bit, 2);  
            } else { 
                re = Integer.parseInt(bit, 2) - 256;  
            }  
        } else {  
            re = Integer.parseInt(bit, 2);  
        }  
        return (byte) re;  
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
