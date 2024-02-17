#include "Arduino.h"
#include "iso-tp.h"
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

// #define ISO_TP_DEBUG3
// #define ISO_TP_DEBUG2
// #define ISO_TP_DEBUG

IsoTp::IsoTp(MCP_CAN* bus, uint8_t mcp_int)
{
  _mcp_int = mcp_int;
  _bus = bus;
}

void IsoTp::print_buffer(INT32U id, uint8_t *buffer, uint32_t len)
{
  uint16_t i=0;

  Serial2.print(F("Buffer: "));
  Serial2.print(id,HEX); Serial2.print(F(" ["));
  Serial2.print(len); Serial2.print(F("] "));
  for(i=0;i<len;i++)
  {
    if(buffer[i] < 0x10) Serial2.print(F("0"));
    Serial2.print(buffer[i],HEX);
    Serial2.print(F(" "));
  }
  Serial2.println();
}

uint8_t IsoTp::can_send(INT32U id, uint8_t len, uint8_t *data)
{
#ifdef ISO_TP_DEBUG
  Serial2.println(F("Send CAN RAW Data:"));
  print_buffer(id, data, len);
#endif
  return _bus->sendMsgBuf(id, 0, len, data);
}

uint8_t IsoTp::can_receive(void)
{
  bool msgReceived;

  if (_mcp_int)
    msgReceived = (!digitalRead(_mcp_int));                     // IRQ: if pin is low, read receive buffer
  else
    msgReceived = (_bus->checkReceive() == CAN_MSGAVAIL);       // No IRQ: poll receive buffer

  if (msgReceived)
  {
     memset(rxBuffer,0,sizeof(rxBuffer));       // Cleanup Buffer
     _bus->readMsgBuf(&rxId, &rxLen, rxBuffer); // Read data: buf = data byte(s)
#ifdef ISO_TP_DEBUG
     Serial2.println(F("Received CAN RAW Data:"));
     print_buffer(rxId, rxBuffer, rxLen);
#endif
    return true;
  }
  else return false;
}

uint8_t IsoTp::send_fc(struct Message_t *msg)
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // FC message high nibble = 0x3 , low nibble = FC Status
  TxBuf[0]=(N_PCI_FC | msg->fc_status);
  TxBuf[1]=msg->blocksize;
  /* fix wrong separation time values according spec */
  if ((msg->min_sep_time > 0x7F) && ((msg->min_sep_time < 0xF1)
      || (msg->min_sep_time > 0xF9))) msg->min_sep_time = 0x7F;
  TxBuf[2]=msg->min_sep_time;
  return can_send(msg->tx_id,8,TxBuf);
}

uint8_t IsoTp::send_sf(struct Message_t *msg) //Send SF Message
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // SF message high nibble = 0x0 , low nibble = Length
  TxBuf[0]=(N_PCI_SF | msg->len);
  memmove(TxBuf+1,msg->Buffer,msg->len);
//  return can_send(msg->tx_id,msg->len+1,TxBuf);// Add PCI length
  return can_send(msg->tx_id,8,TxBuf);// Always send full frame
}

uint8_t IsoTp::send_ff(struct Message_t *msg) // Send FF
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  msg->seq_id=1;

  if(msg->len <= 4095)
  {
    TxBuf[0]=(N_PCI_FF | ((msg->len&0x0F00) >> 8));
    TxBuf[1]=(msg->len&0x00FF);
    memmove(TxBuf+2,msg->Buffer,6);             // Skip 2 Bytes PCI
  } else
  {
    TxBuf[0] = N_PCI_FF;
    TxBuf[1] = 0;
    TxBuf[2] = (uint8_t)((msg->len >> 24) & 0xFF);
    TxBuf[3] = (uint8_t)((msg->len >> 16) & 0xFF);
    TxBuf[4] = (uint8_t)((msg->len >> 8) & 0xFF);
    TxBuf[5] = (uint8_t)(msg->len & 0xFF);
    TxBuf[6] = msg->Buffer[0];
    TxBuf[7] = msg->Buffer[1];
  }
  
  
  


  // /* get the FF_DL */
  // msg->len = (rxBuffer[0] & 0x0F) << 8;
  // msg->len += rxBuffer[1];

	// /* Check for FF_DL escape sequence supporting 32 bit PDU length */
	// if (msg->len != 0) {
	// 	rest=msg->len;
	// } else {
	// 	/* FF_DL = 0 => get real length from next 4 bytes */
	// 	msg->len = rxBuffer[2] << 24;
	// 	msg->len += rxBuffer[3] << 16;
	// 	msg->len += rxBuffer[4] << 8;
	// 	msg->len += rxBuffer[5];
	// 	rest=msg->len+4;
	// }


  
  return can_send(msg->tx_id,8,TxBuf);       // First Frame has full length
}

uint8_t IsoTp::send_cf(struct Message_t *msg) // Send SF Message
{
  uint8_t TxBuf[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint16_t len=7;

  TxBuf[0]=(N_PCI_CF | (msg->seq_id & 0x0F));
  if(msg->len>7) len=7; else len=msg->len;
  memmove(TxBuf+1,msg->Buffer,len);         // Skip 1 Byte PCI
  //return can_send(msg->tx_id,len+1,TxBuf); // Last frame is probably shorter
                                           // than 8 -> Signals last CF Frame
  return can_send(msg->tx_id,8,TxBuf);     // Last frame is probably shorter
                                           // than 8, pad with 00
}

void IsoTp::fc_delay(uint8_t sep_time)
{
  /*
  * 0x00 - 0x7F: 0 - 127ms
  * 0x80 - 0xF0: reserved
  * 0xF1 - 0xF9: 100us - 900us
  * 0xFA - 0xFF: reserved
  * default 0x7F, 127ms
  */
  if(sep_time <= 0x7F)
    delay(sep_time);
  else if ((sep_time >= 0xF1) && (sep_time <= 0xF9))
    delayMicroseconds((sep_time-0xF0)*100);
  else
    delay(0x7F);
}

uint8_t IsoTp::rcv_sf(struct Message_t* msg)
{
  /* get the SF_DL from the N_PCI byte */
  msg->len = rxBuffer[0] & 0x0F;
  /* copy the received data bytes */
  memmove(msg->Buffer,rxBuffer+1,msg->len); // Skip PCI, SF uses len bytes
  msg->tp_state=ISOTP_FINISHED;

  return 0;
}

uint8_t IsoTp::rcv_ff(struct Message_t* msg)
{
  msg->seq_id=1;

  /* get the FF_DL */
  msg->len = (rxBuffer[0] & 0x0F) << 8;
  msg->len += rxBuffer[1];

	/* Check for FF_DL escape sequence supporting 32 bit PDU length */
	if (msg->len != 0) {
		rest=msg->len;
	} else {
		/* FF_DL = 0 => get real length from next 4 bytes */
		msg->len = rxBuffer[2] << 24;
		msg->len += rxBuffer[3] << 16;
		msg->len += rxBuffer[4] << 8;
		msg->len += rxBuffer[5];
		rest=msg->len+4;
	}

  #ifdef ISO_TP_DEBUG2
  Serial2.print(F("First frame received with message length: "));
  Serial2.println(rest);
  Serial2.println(F("Send flow controll."));
  Serial2.print(F("ISO-TP state: ")); Serial2.println(msg->tp_state);
  print_buffer(5, rxBuffer, 7);
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  Serial2.print(F("#################################################"));
  #endif

  // free(msg->Buffer);
  // // msg->Buffer = (uint8_t*)malloc(150000);
  // msg->Buffer = (uint8_t*)malloc(msg->len);


  /* copy the first received data bytes */
  memmove(msg->Buffer,rxBuffer+2,6); // Skip 2 bytes PCI, FF must have 6 bytes!
  rest-=6; // Restlength

  msg->tp_state = ISOTP_WAIT_DATA;



  /* send our first FC frame with Target Address*/
  struct Message_t fc;
  fc.tx_id=msg->tx_id;
  fc.fc_status=ISOTP_FC_CTS;
  fc.blocksize=0;
  fc.min_sep_time=0;
  return send_fc(&fc);
}

uint8_t IsoTp::rcv_cf(struct Message_t* msg)
{
  //Handle Timeout
  //If no Frame within 250ms change State to ISOTP_IDLE
  INT32U delta=millis()-wait_cf;

  if((delta >= TIMEOUT_FC) && msg->seq_id>1)
  {
#ifdef ISO_TP_DEBUG2
    Serial2.println(F("CF frame timeout during receive wait_cf="));
    Serial2.print(wait_cf); Serial2.print(F(" delta="));
    Serial2.println(delta);
#endif
    msg->tp_state = ISOTP_IDLE;
    return 1;
  }
  wait_cf=millis();

#ifdef ISO_TP_DEBUG3
  Serial2.print(F("ISO-TP state: ")); Serial2.println(msg->tp_state);
  Serial2.print(F("CF received with message rest length: "));
  Serial2.println(rest);
#endif

  if (msg->tp_state != ISOTP_WAIT_DATA) return 0;


  if ((rxBuffer[0] & 0x0F) != (msg->seq_id & 0x0F))
  {
#ifdef ISO_TP_DEBUG2
    Serial2.print(F("Got sequence ID: ")); Serial2.print(rxBuffer[0] & 0x0F);
    Serial2.print(F(" Expected: ")); Serial2.println(msg->seq_id & 0x0F);
#endif
    msg->tp_state = ISOTP_IDLE;
    msg->seq_id = 1;
    return 1;
  }

  if(rest<=7) // Last Frame
  {
    memmove(msg->Buffer+6+7*(msg->seq_id-1),rxBuffer+1,rest);// 6 Bytes in FF +7
    msg->tp_state=ISOTP_FINISHED;                           // per CF skip PCI
#ifdef ISO_TP_DEBUG2
    Serial2.print(F("Last CF received with seq. ID: "));
    Serial2.println(msg->seq_id);
#endif
  }
  else
  {
#ifdef ISO_TP_DEBUG3
    Serial2.print(F("CF received with seq. ID: "));
    Serial2.println(msg->seq_id);
    Serial2.print(F("Buffer address: "));
    Serial2.println((unsigned long)(msg->Buffer));
    if(msg->seq_id==2340)
    {
      Serial2.println(msg->seq_id);
    }
#endif
    volatile uint32_t index = 6+7*(msg->seq_id-1);
    memmove(msg->Buffer+index,rxBuffer+1,7); // 6 Bytes in FF +7
                                                          // per CF
    rest-=7; // Got another 7 Bytes of Data;
      #ifdef ISO_TP_DEBUG
    Serial2.println("memmove ok");
#endif
  }
  



  msg->seq_id++;

  return 0;
}

uint8_t IsoTp::rcv_fc(struct Message_t* msg)
{
  uint8_t retval=0;

  if (msg->tp_state != ISOTP_WAIT_FC && msg->tp_state != ISOTP_WAIT_FIRST_FC)
    return 0;

  /* get communication parameters only from the first FC frame */
  if (msg->tp_state == ISOTP_WAIT_FIRST_FC)
  {
    msg->blocksize = rxBuffer[1];
    msg->min_sep_time = rxBuffer[2];

    /* fix wrong separation time values according spec */
    if ((msg->min_sep_time > 0x7F) && ((msg->min_sep_time < 0xF1)
	|| (msg->min_sep_time > 0xF9))) msg->min_sep_time = 0x7F;
  }

#ifdef ISO_TP_DEBUG
  Serial2.print(F("FC frame: FS "));
  Serial2.print(rxBuffer[0]&0x0F);
  Serial2.print(F(", Blocksize "));
  Serial2.print(msg->blocksize);
  Serial2.print(F(", Min. separation Time "));
  Serial2.println(msg->min_sep_time);
#endif

  switch (rxBuffer[0] & 0x0F)
  {
    case ISOTP_FC_CTS:
                         msg->tp_state = ISOTP_SEND_CF;
                         break;
    case ISOTP_FC_WT:
                         fc_wait_frames++;
			 if(fc_wait_frames >= MAX_FCWAIT_FRAME)
                         {
#ifdef ISO_TP_DEBUG2
                           Serial2.println(F("FC wait frames exceeded."));
#endif
                           fc_wait_frames=0;
                           msg->tp_state = ISOTP_IDLE;
                           retval=1;
                         }
#ifdef ISO_TP_DEBUG
                         Serial2.println(F("Start waiting for next FC"));
#endif
                         break;
    case ISOTP_FC_OVFLW:
#ifdef ISO_TP_DEBUG2
                         Serial2.println(F("Overflow in receiver side"));
#endif
    default:
                         msg->tp_state = ISOTP_IDLE;
                         retval=1;
  }
  return retval;
}

uint8_t IsoTp::send(Message_t* msg)
{
  uint8_t bs=false;
  INT32U delta=0;
  uint8_t retval=0;
  uint8_t *Buffer_start = msg->Buffer;

  msg->tp_state=ISOTP_SEND;

  while(msg->tp_state!=ISOTP_IDLE && msg->tp_state!=ISOTP_ERROR)
  {
    bs=false;

#ifdef ISO_TP_DEBUG
    Serial2.print(F("ISO-TP State: ")); Serial2.println(msg->tp_state);
    Serial2.print(F("Length      : ")); Serial2.println(msg->len);
#endif

    switch(msg->tp_state)
    {
      case ISOTP_IDLE         :  break;
      case ISOTP_SEND         :
                                 if(msg->len<=7)
                                 {
#ifdef ISO_TP_DEBUG
                                   Serial2.println(F("Send SF"));
#endif
                                   retval=send_sf(msg);
                                   msg->tp_state=ISOTP_IDLE;
                                 }
                                 else
                                 {
#ifdef ISO_TP_DEBUG
                                   Serial2.println(F("Send FF"));
#endif
                                   if(!(retval=send_ff(msg))) // FF complete
                                   {
                                    if(msg->len > 4095)
                                    { 
                                      msg->Buffer+=2;
                                      msg->len-=2;

                                    } else
                                    {
                                     msg->Buffer+=6;
                                     msg->len-=6;
                                    }

                                     msg->tp_state=ISOTP_WAIT_FIRST_FC;
                                     fc_wait_frames=0;
                                     wait_fc=millis();
                                   }
                                 }
                                 break;
      case ISOTP_WAIT_FIRST_FC:
#ifdef ISO_TP_DEBUG
                                 Serial2.println(F("Wait first FC"));
#endif
                                 delta=millis()-wait_fc;
                                 if(delta >= TIMEOUT_FC)
                                 {
#ifdef ISO_TP_DEBUG2
                                   Serial2.print(F("FC timeout during receive"));
                                   Serial2.print(F(" wait_fc="));
                                   Serial2.print(wait_fc);
                                   Serial2.print(F(" delta="));
                                   Serial2.println(delta);
#endif
                                   msg->tp_state = ISOTP_IDLE;
				   retval=1;
                                 }
                                 break;
      case ISOTP_WAIT_FC      :
#ifdef ISO_TP_DEBUG
                                 Serial2.println(F("Wait FC"));
#endif
                                 break;
      case ISOTP_SEND_CF      :
#ifdef ISO_TP_DEBUG
                                 Serial2.println(F("Send CF"));
#endif
                                 while(msg->len>7 && !bs)
                                 {
                                   fc_delay(msg->min_sep_time);
                                   if(!(retval=send_cf(msg)))
                                   {
#ifdef ISO_TP_DEBUG
                                     Serial2.print(F("Send Seq "));
                                     Serial2.println(msg->seq_id);
#endif
                                     if(msg->blocksize > 0)
                                     {
#ifdef ISO_TP_DEBUG
                                       Serial2.print(F("Blocksize trigger "));
                                       Serial2.print(msg->seq_id %
                                                    msg->blocksize);
#endif
                                       if(!(msg->seq_id % msg->blocksize))
                                       {
                                         bs=true;
                                         msg->tp_state=ISOTP_WAIT_FC;
#ifdef ISO_TP_DEBUG
                                         Serial2.println(F(" yes"));
#endif
                                       }
#ifdef ISO_TP_DEBUG
                                       else Serial2.println(F(" no"));
#endif
                                     }
                                     msg->seq_id++;
				     if (msg->blocksize < 16)
                                       msg->seq_id %= 16;
                                     else
                                       msg->seq_id %= msg->blocksize;
                                     msg->Buffer+=7;
                                     msg->len-=7;
#ifdef ISO_TP_DEBUG
                                     Serial2.print(F("Length      : "));
                                     Serial2.println(msg->len);
#endif
                                   }
                                 }
                                 if(!bs)
                                 {
                                   fc_delay(msg->min_sep_time);
#ifdef ISO_TP_DEBUG
                                   Serial2.print(F("Send last Seq "));
                                   Serial2.println(msg->seq_id);
#endif
                                   retval=send_cf(msg);
                                   msg->tp_state=ISOTP_IDLE;
                                 }
                                 break;
      default                 :  break;
    }


    if(msg->tp_state==ISOTP_WAIT_FIRST_FC ||
       msg->tp_state==ISOTP_WAIT_FC)
    {
      if(can_receive())
      {
#ifdef ISO_TP_DEBUG
        Serial2.println(F("Send branch:"));
#endif
        if(rxId==msg->rx_id)
        {
          retval=rcv_fc(msg);
          memset(rxBuffer,0,sizeof(rxBuffer));
#ifdef ISO_TP_DEBUG
          Serial2.println(F("rxId OK!"));
#endif
        }
      }
    }
  }
  // Code above actually changes the value of the buffer
  // pointer. A better fix than the following line would
  // be to fix that code so it does not change the buffer
  // value.
  msg->Buffer = Buffer_start;
  return retval;
}

uint8_t IsoTp::receive(Message_t* msg)
{
  uint8_t n_pci_type=0;
  INT32U delta=0;

  wait_session=millis();
  unsigned long wait_consecutive=millis();

#ifdef ISO_TP_DEBUG
  Serial2.println(F("Start receive..."));
#endif
  msg->tp_state=ISOTP_IDLE;

  while(msg->tp_state!=ISOTP_FINISHED && msg->tp_state!=ISOTP_ERROR)
  {
//     delta=millis()-wait_session;
//     if(delta >= TIMEOUT_SESSION)
//     {
// #ifdef ISO_TP_DEBUG2
//       Serial2.print(F("ISO-TP Session timeout wait_session="));
//       Serial2.print(wait_session); Serial2.print(F(" delta="));
//       Serial2.println(delta);
// #endif
//       return 1;
//     }

    delta=millis()-wait_consecutive;
    if(delta >= TIMEOUT_CF)
    {
      #ifdef ISO_TP_DEBUG2
      Serial2.print(F("ISO-TP Consecutive frame timeout wait_consecutive="));
      Serial2.print(wait_session); Serial2.print(F(" delta="));
      Serial2.println(delta);
      #endif
      return 1;
    }

    if(can_receive())
    {
      wait_consecutive=millis();
      if(rxId==msg->rx_id)
      {
#ifdef ISO_TP_DEBUG
        Serial2.println(F("rxId OK!"));
#endif
        n_pci_type=rxBuffer[0] & 0xF0;

        switch (n_pci_type)
        {
          case N_PCI_FC:
#ifdef ISO_TP_DEBUG
                      Serial2.println(F("FC"));
#endif
                      /* tx path: fc frame */
                      rcv_fc(msg);
                      break;

          case N_PCI_SF:
#ifdef ISO_TP_DEBUG
                      Serial2.println(F("SF"));
#endif
                      /* rx path: single frame */
                      rcv_sf(msg);
//		      msg->tp_state=ISOTP_FINISHED;
                      break;

          case N_PCI_FF:
#ifdef ISO_TP_DEBUG
                      Serial2.println(F("FF"));
#endif
                      /* rx path: first frame */
                      rcv_ff(msg);
//		      msg->tp_state=ISOTP_WAIT_DATA;
                      break;
                      break;

          case N_PCI_CF:
#ifdef ISO_TP_DEBUG
                      Serial2.println(F("CF"));
#endif
                      /* rx path: consecutive frame */
                      rcv_cf(msg);
                      break;
        }
        memset(rxBuffer,0,sizeof(rxBuffer));
      }
    }
  }
#ifdef ISO_TP_DEBUG
  Serial2.println(F("ISO-TP message received:"));
  // print_buffer(msg->rx_id, msg->Buffer, msg->len);
#endif

  return 0;
}
