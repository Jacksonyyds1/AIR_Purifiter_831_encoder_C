LCD_RES_Set();
delay_ms(1); //delay_ms 1ms
LCD_RES_Clr();
delay_ms(10); //delay_ms 10ms
LCD_RES_Set();

delay_ms(120);                

WriteComm(0x11);     

delay_ms(120);   

WriteComm(0xB2);     
WriteData(0x1F);   
WriteData(0x1F);   
WriteData(0x00);   
WriteData(0x33);   
WriteData(0x33);   

WriteComm(0x35);     
WriteData(0x00);               

WriteComm(0x36);     
WriteData(0x00);   

WriteComm(0x3A);     
WriteData(0x05);   

WriteComm(0xB7);     
WriteData(0x00);   

WriteComm(0xBB);     
WriteData(0x36);   

WriteComm(0xC0);     
WriteData(0x2C);   

WriteComm(0xC2);     
WriteData(0x01);   

WriteComm(0xC3);     
WriteData(0x0F);   //GVDD=4.3V 

WriteComm(0xC4);     
WriteData(0x20);   //VDV, 0x20:0v

WriteComm(0xC6);     
WriteData(0x13);         

WriteComm(0xD6);     
WriteData(0xA1);   	

WriteComm(0xD0);     
WriteData(0xA4);   
WriteData(0xA1);  

WriteComm(0xD6);     
WriteData(0xA1);  

WriteComm(0xE0);
WriteData(0xF0);
WriteData(0x07);
WriteData(0x0D);
WriteData(0x0B);
WriteData(0x0C);
WriteData(0x28);
WriteData(0x2A);
WriteData(0x44);
WriteData(0x41);
WriteData(0x38);
WriteData(0x13);
WriteData(0x13);
WriteData(0x23);
WriteData(0x27);

WriteComm(0xE1);
WriteData(0xF0);
WriteData(0x08);
WriteData(0x0B);
WriteData(0x09);
WriteData(0x07);
WriteData(0x04);
WriteData(0x29);
WriteData(0x33);
WriteData(0x40);
WriteData(0x2A);
WriteData(0x17);
WriteData(0x17);
WriteData(0x24);
WriteData(0x29);

WriteComm(0xE4);    
WriteData(0x1D); //使用240根gate  (N+1)*8
WriteData(0x00); //设定gate起点位置
WriteData(0x00);

WriteComm(0x21);     

WriteComm(0x29);

WriteComm(0x2A);     
WriteData(0x00);   
WriteData(0x00);   
WriteData(0x00);   
WriteData(0xEF);  //240

WriteComm(0x2B);     
WriteData(0x00);   
WriteData(0x00);   
WriteData(0x00);   
WriteData(0xEF);  //240 

WriteComm(0x2C); 