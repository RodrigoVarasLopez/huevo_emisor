void RFIDsetup(){
   SPI.begin();        // Init SPI bus
    
    mfrc522.PCD_Init(); // Init MFRC522 card

    /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
    Serial.print("Ver: 0x");      
    byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    Serial.println(readReg, HEX);      

     /* setup the IRQ pin*/ 
    pinMode(IRQ_PIN, INPUT_PULLUP);
   
    /* 
     *  Allow the ... irq to be propagated to the IRQ pin
     *  For test purposes propagate the IdleIrq and loAlert
     */
    regVal = 0xA0; //rx irq
    mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg,regVal);
    
    //bNewInt = false; //interrupt flag

    /*Activate the interrupt*/
    //attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);
    

    Serial.println("End setup"); 
}

void activateRec(MFRC522 mfrc522){
    mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg,mfrc522.PICC_CMD_REQA);
    mfrc522.PCD_WriteRegister(mfrc522.CommandReg,mfrc522.PCD_Transceive);  
    mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);    
}
