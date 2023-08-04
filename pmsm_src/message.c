/*
 *     message.c
 *
 *     按照通讯协议，对收到的数据进行解析
 *
 *
 */
#include "DSP28x_Project.h"


void cltran(void)
{
    RCBUF[0]=0X00;//包头
    RCBUF[1]=0X00;//包头
    RCBUF[2]=0X00;//包头
    RCBUF[3]=0X00;//包头
    RCBUF[4]=0X00;//包长
    RCBUF[5]=0X00;//站点号
    RCBUF[6]=0X00;//序号
    RCBUF[7]=0X00;//命令码
    RCBUF[8]=0X00;//数据低字节
    RCBUF[9]=0X00;//数据高字节
    RCBUF[10]=0X00;//校验码
}

////发送缓存器初始化
//void ClrTxbuf(void)
//{
//    TXBUF[0]=0x00;//包头
//    TXBUF[1]=0x00;//包头
//    TXBUF[2]=0x00;//包头
//    TXBUF[3]=0x00;//包头
//    TXBUF[4]=0x00;//包长
//    TXBUF[5]=0x00;//站点号
//    TXBUF[6]=0x00;//序列号
//    TXBUF[7]=0x00;//命令码
//    TXBUF[8]=0x00;//数据低字节
//    TXBUF[9]=0x00;//数据高字节
//    TXBUF[10]=0x00;//校验码
//}

void InitSciParameter(void)
{
    unsigned int i;
    RunCommand = 0;
    for(i=0;i<ParameterNumber;i++)
    {
        Paramet[i] = 0x0101;
    }
}

//SCI接收处理函数
void SciRecieve(void)
{
    if(ScicRegs.SCIRXST.bit.FE == 1)//有帧错误//这段话的注释除了这句话，不要相信
    {
        InitSci();
    }
    else
    {
    //DSPd SCI模块数据校验格式，注意寄存器限制的原因，一次只能获取一个字节
        RCBUF[ReciveRCOUNT++] = ScicRegs.SCIRXBUF.bit.RXDT; //&0x00FF接收到的八位数据；按照8位8位来进行接收数据；
    //前4帧都是包头FE FE FE FE
        if((ReciveRCOUNT-1)<PackHeadLength)//包头长度总为4，与数据包长不同
        {
            //若小于包头长度，则现在仅在传送4位包头，所以对数据与既定的包头FE对比是否相同
            if(RCBUF[ReciveRCOUNT-1] != PackHead)//
            {
                //若不相同，则数据传送有误，485接收计数归零，停止接收跳出中断
                ReciveRCOUNT = 0;
            }
            //若相同，则数据传送无误，结束本次接收中断
        }
        else if((ReciveRCOUNT-1) == PackHeadLength) //包长
        {
            //若等于包头长度，则现在包头已经传送完毕
            PackLength = RCBUF[ReciveRCOUNT-1];//将第二表示的包长赋给给定包长
            RC_DataCount = 0;
        }
        else if((ReciveRCOUNT-1) > PackHeadLength)
        {
            //若大于包头长度，则现在开始传送序号
            if(PackLength==3||PackLength==5||PackLength==7||PackLength==19)//包长符合
            {
				if(RC_DataCount < PackLength)
				{
					//如果计数小于包长，则表示尚未将数据部分传送完毕，故将数据置于缓存中
					RC_DataBUF[RC_DataCount++] = RCBUF[ReciveRCOUNT-1];
				}
				if(RC_DataCount == PackLength)   //一包数据接收完成
				{
					//如果计数等于包长，则表示数据已经传送完毕，
					//故将缓存中的第0号站点号，第1号序列号，第2号命令码，以及最后一号校验码交给对应变量
					SerialNumber = RC_DataBUF[0];//序列号//序号只有8位0~255
					CommandCode = RC_DataBUF[1];//命令码
					CheckCode = RC_DataBUF[PackLength-1];//校验码
					//置发送标志
					flagRC = 1; //数据接收结束标志位，表明该包数据是所需数据；
					ReciveRCOUNT = 0;
					RC_DataCount = 0;
				}
            }
			else
			{
				ReciveRCOUNT = 0;
				RC_DataCount = 0;
			}
        }
        else
        {

        }
   }
}


//发送处理函数
void TXdeal(void)
{
	if(flagRC==1)   //数据接收完毕（数据接收标志位为1）
	{

	   //实时运行参数回发-浮点数传输-每个数4字节
		if( (SerialNumber < 44)&&(CommandCode!=0xB1))
		{
			TXBUF[0] = 0X00FE;//包头，放到发送缓存器中。
			TXBUF[1] = 0X00FE;//包头
			TXBUF[2] = 0X00FE;//包头
			TXBUF[3] = 0X00FE;//包头
			TXBUF[4] = 0X08;//包长--------------------
			TXBUF[5] = SerialNumber;//序列号
			TXBUF[6] = CommandCode;//命令码
			TXBUF[7] = ConfirmCode;//确认码
//方法1：手动移位合并数据
//        SendData = Paramet[SerialNumber];//
//        TXBUF[8] = SendData&0x00ff;//数据低字节
//        TXBUF[9] = (SendData&0xff00)>>8;//数据高字节
//方法2：结构体融合数据
//        Data_send.all=Paramet[SerialNumber];
//        TXBUF[8]=Data_send.bit.MEM1;
//        TXBUF[9]=Data_send.bit.MEM2;

			FData_send.all=Paramet[SerialNumber];
			TXBUF[8]=FData_send.bit.MEM1;
			TXBUF[9]=FData_send.bit.MEM2;
			TXBUF[10]=FData_send.bit.MEM3;
			TXBUF[11]=FData_send.bit.MEM4;

			//datasum=TXBUF[4]+TXBUF[5]+TXBUF[6]+TXBUF[7]+TXBUF[8]+TXBUF[9];
			datasum=TXBUF[4]+TXBUF[5]+TXBUF[6]+TXBUF[7]+TXBUF[8]+TXBUF[9]+TXBUF[10]+TXBUF[11];

			datasum=~datasum+1;//校验码：求和取反加1
			datasum &= 0X00FF;
			TXBUF[12]=datasum;//校验码

			TXCOUNT=0;//发送计数器
			flagSEND = 1;//发送数据标志位
			SendDataNumber = 13;
//        switch (SerialNumber)
//        {
//            case 00: break;
//        }
		}
		else if ((SerialNumber < 119)&&(SerialNumber > 43))//调试参数\修正系数\启停机命令的下发回应，加了确认码
		{
			datasum=0;
			TXBUF[0] = 0XFE;//包头
			TXBUF[1] = 0XFE;//包头
			TXBUF[2] = 0XFE;//包头
			TXBUF[3] = 0XFE;//包头
			TXBUF[4] = 0X04;//包长
			TXBUF[5] = SerialNumber; //序列号
			TXBUF[6] = CommandCode; //命令码
			TXBUF[7] = ConfirmCode;
			datasum = TXBUF[4]+TXBUF[5]+TXBUF[6]+TXBUF[7];
			datasum = (~datasum)+1; //校验码：求和取反加1
			datasum &= 0X00FF;
			TXBUF[8]=datasum; //校验码
			TXCOUNT=0;
			flagSEND = 1;
			SendDataNumber = 9;
		}
		else if (CommandCode == 0xB1)   //开关机
		{
			TXBUF[0] = 0XFE;//包头
			TXBUF[1] = 0XFE;//包头
			TXBUF[2] = 0XFE;//包头
			TXBUF[3] = 0XFE;//包头
			TXBUF[4] = 0X04;//包长
			TXBUF[5] = SerialNumber; //序列号
			TXBUF[6] = CommandCode; //命令码
			TXBUF[7] = ConfirmCode; //确认码
			datasum = TXBUF[4]+TXBUF[5]+TXBUF[6]+TXBUF[7];
			datasum = ~datasum+1; //校验码：求和取反加1
			TXBUF[8]=datasum; //校验码
			TXCOUNT=0;
			flagSEND = 1;
			SendDataNumber = 9;
		}
	}
}


void Checkdata(void)//数据判断
{
        if(TXCOUNT==0) //发送队列为空
        {
            if(flagRC==1)//接收队列为空,表示接受队列已完成，接收到的数据包完全正确
            {
                unsigned int i;
                ReciveRCOUNT=0;
				if(PackLength==19)//包长是否等于19，判断是否为粒子群反馈
				{
					int i;
					if(SerialNumber=200&&CheckCode==0xff)
					{
						for(i=0;i<4;i++)
						{
							PSO_get.bit.MEM1=RC_DataBUF[4*i+2];
							PSO_get.bit.MEM2=RC_DataBUF[4*i+3];
							PSO_get.bit.MEM3=RC_DataBUF[4*i+4];
							PSO_get.bit.MEM4=RC_DataBUF[4*i+5];
							PSO_g[i]=PSO_get.all;
						}
					}
					flagRC = 0;
				}
				else
				{
	                datasum = 0;//xyy注意，此变量最好改为局部变量
	                //验证接收数据是否正确，除包头和最后一个校验码外求和取反，然后与校验码比较
	                for(i=0;i<(PackLength-1);i++)
	                {
	                    datasum += RC_DataBUF[i];
	                }
	                datasum += PackLength;
	                datasum =~ datasum+1;
	                datasum &= 0X00FF;
	                datasum1 = datasum ;

				   if(datasum == CheckCode) //接收数据正确（datasum =校验码）
				   {
						if(PackLength==5&&CommandCode==0xB1)//包长是否等于5-判断开关机程序
						{
							//if(CommandCode==0xB1)
							//{
								//Switchsystem = RC_DataBUF[2]+((RC_DataBUF[3]<<8)&0xff00);
								Data_get.bit.MEM1=RC_DataBUF[2];
								Data_get.bit.MEM2=RC_DataBUF[3];
								Switchsystem=Data_get.all;
							//}
						}
						if(PackLength==7)//包长是否等于7//属于主机覆盖从机，表示主机有数据修改需要覆盖对应的从机
						{
							FData_get.bit.MEM1=RC_DataBUF[2];
							FData_get.bit.MEM2=RC_DataBUF[3];
							FData_get.bit.MEM3=RC_DataBUF[4];
							FData_get.bit.MEM4=RC_DataBUF[5];
							Paramet[SerialNumber]=FData_get.all;
						}
						TXdeal(); //命令码判定和发送程序
					}
					else //接收格式不符
					{
						flagRC = 0;
					}
				}
            }

        }
        else
        {
        }
}



//SCI发送处理函数
void SciSend(void)
{
    if( ScicRegs.SCICTL2.bit.TXRDY == 1)  //发送器的缓冲器和移位寄存器都为空，每次发送前都要确认一下这个寄存器的标志位为1
    {
        if(flagSEND == 1)
        {
            //一组数据还没发送完,接收和发送包长不同时，SendDataNumber在TXdeal()中给定
            if(TXCOUNT < SendDataNumber)
            {
                ScicRegs.SCITXBUF = TXBUF[TXCOUNT] ;//& 0X00FF;//发送，并再次确认为8位数据，这个寄存器就是用来发送字节的
                TXCOUNT++;
            }
            else//一组数据已经发送结束
            {
                flagRC = 0;
                flagSEND = 0;
                TXCOUNT = 0;
                //ClrTxbuf();
            }
        }
    }
}


//PSO发送程序
//定时主动发送给上位机
void PSOsend(float U[10])
{
	if( ScicRegs.SCICTL2.bit.TXRDY == 1)//发送前都要确认一下这个寄存器的标志位为1
	{
		if(PSOSENDF)//经过了0.05s后，发送一次数据
		{
			if(PSO_datainit_flag==0)
			{
				int i;
				for(i=0;i<4;i++)
				{
					PSOBUF[i]=0xFE;
				}
				PSOBUF[4]=43;
				PSOBUF[5]=200;
				PSOBUF[6]=0xFF;
				for(i=0;i<10;i++)
				{
					FData_get.all=U[i];
					PSOBUF[4*i+7]=FData_get.bit.MEM1;
					PSOBUF[4*i+8]=FData_get.bit.MEM2;
					PSOBUF[4*i+9]=FData_get.bit.MEM3;
					PSOBUF[4*i+10]=FData_get.bit.MEM4;
				}

				PSOBUF[PSONumber-1]=0XFF;//特殊校验符号
				PSO_datainit_flag=1;
			}
			else
			{
				if(PSOCOUNT < PSONumber)
				{
					ScicRegs.SCITXBUF = PSOBUF[PSOCOUNT] ;//& 0X00FF;//发送，并再次确认为8位数据，这个寄存器就是用来发送字节的
					PSOCOUNT++;
				}
				else
				{
					PSOCOUNT = 0;
					PSO_datainit_flag=0;
					PSOSENDF=0;
				}
			}
		}

	}
}





