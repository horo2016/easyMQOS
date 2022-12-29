#  easymqOs_opticalFlow_gl9306 
光流模块通过串口读取无名创新公司的gl9306传感器的数据，其协议格式位：  
0xfe 0x04 d0 d1 d2 d3 sum qual 0xaa 
flowx = d0+d1*256;   
flowy = d2+d3*256;     
flowx,flowy数据为上一帧0时的相对值   


修改Makefile 中的 /dev/ttyS3 指定串口。  
 
  
