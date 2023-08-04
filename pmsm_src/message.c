/*
 *     message.c
 *
 *     ����ͨѶЭ�飬���յ������ݽ��н���
 *
 *
 */
#include "DSP28x_Project.h"


void cltran(void)
{
    RCBUF[0]=0X00;//��ͷ
    RCBUF[1]=0X00;//��ͷ
    RCBUF[2]=0X00;//��ͷ
    RCBUF[3]=0X00;//��ͷ
    RCBUF[4]=0X00;//����
    RCBUF[5]=0X00;//վ���
    RCBUF[6]=0X00;//���
    RCBUF[7]=0X00;//������
    RCBUF[8]=0X00;//���ݵ��ֽ�
    RCBUF[9]=0X00;//���ݸ��ֽ�
    RCBUF[10]=0X00;//У����
}

////���ͻ�������ʼ��
//void ClrTxbuf(void)
//{
//    TXBUF[0]=0x00;//��ͷ
//    TXBUF[1]=0x00;//��ͷ
//    TXBUF[2]=0x00;//��ͷ
//    TXBUF[3]=0x00;//��ͷ
//    TXBUF[4]=0x00;//����
//    TXBUF[5]=0x00;//վ���
//    TXBUF[6]=0x00;//���к�
//    TXBUF[7]=0x00;//������
//    TXBUF[8]=0x00;//���ݵ��ֽ�
//    TXBUF[9]=0x00;//���ݸ��ֽ�
//    TXBUF[10]=0x00;//У����
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

//SCI���մ�����
void SciRecieve(void)
{
    if(ScicRegs.SCIRXST.bit.FE == 1)//��֡����//��λ���ע�ͳ�����仰����Ҫ����
    {
        InitSci();
    }
    else
    {
    //DSPd SCIģ������У���ʽ��ע��Ĵ������Ƶ�ԭ��һ��ֻ�ܻ�ȡһ���ֽ�
        RCBUF[ReciveRCOUNT++] = ScicRegs.SCIRXBUF.bit.RXDT; //&0x00FF���յ��İ�λ���ݣ�����8λ8λ�����н������ݣ�
    //ǰ4֡���ǰ�ͷFE FE FE FE
        if((ReciveRCOUNT-1)<PackHeadLength)//��ͷ������Ϊ4�������ݰ�����ͬ
        {
            //��С�ڰ�ͷ���ȣ������ڽ��ڴ���4λ��ͷ�����Զ�������ȶ��İ�ͷFE�Ա��Ƿ���ͬ
            if(RCBUF[ReciveRCOUNT-1] != PackHead)//
            {
                //������ͬ�������ݴ�������485���ռ������㣬ֹͣ���������ж�
                ReciveRCOUNT = 0;
            }
            //����ͬ�������ݴ������󣬽������ν����ж�
        }
        else if((ReciveRCOUNT-1) == PackHeadLength) //����
        {
            //�����ڰ�ͷ���ȣ������ڰ�ͷ�Ѿ��������
            PackLength = RCBUF[ReciveRCOUNT-1];//���ڶ���ʾ�İ���������������
            RC_DataCount = 0;
        }
        else if((ReciveRCOUNT-1) > PackHeadLength)
        {
            //�����ڰ�ͷ���ȣ������ڿ�ʼ�������
            if(PackLength==3||PackLength==5||PackLength==7||PackLength==19)//��������
            {
				if(RC_DataCount < PackLength)
				{
					//�������С�ڰ��������ʾ��δ�����ݲ��ִ�����ϣ��ʽ��������ڻ�����
					RC_DataBUF[RC_DataCount++] = RCBUF[ReciveRCOUNT-1];
				}
				if(RC_DataCount == PackLength)   //һ�����ݽ������
				{
					//����������ڰ��������ʾ�����Ѿ�������ϣ�
					//�ʽ������еĵ�0��վ��ţ���1�����кţ���2�������룬�Լ����һ��У���뽻����Ӧ����
					SerialNumber = RC_DataBUF[0];//���к�//���ֻ��8λ0~255
					CommandCode = RC_DataBUF[1];//������
					CheckCode = RC_DataBUF[PackLength-1];//У����
					//�÷��ͱ�־
					flagRC = 1; //���ݽ��ս�����־λ�������ð��������������ݣ�
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


//���ʹ�����
void TXdeal(void)
{
	if(flagRC==1)   //���ݽ�����ϣ����ݽ��ձ�־λΪ1��
	{

	   //ʵʱ���в����ط�-����������-ÿ����4�ֽ�
		if( (SerialNumber < 44)&&(CommandCode!=0xB1))
		{
			TXBUF[0] = 0X00FE;//��ͷ���ŵ����ͻ������С�
			TXBUF[1] = 0X00FE;//��ͷ
			TXBUF[2] = 0X00FE;//��ͷ
			TXBUF[3] = 0X00FE;//��ͷ
			TXBUF[4] = 0X08;//����--------------------
			TXBUF[5] = SerialNumber;//���к�
			TXBUF[6] = CommandCode;//������
			TXBUF[7] = ConfirmCode;//ȷ����
//����1���ֶ���λ�ϲ�����
//        SendData = Paramet[SerialNumber];//
//        TXBUF[8] = SendData&0x00ff;//���ݵ��ֽ�
//        TXBUF[9] = (SendData&0xff00)>>8;//���ݸ��ֽ�
//����2���ṹ���ں�����
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

			datasum=~datasum+1;//У���룺���ȡ����1
			datasum &= 0X00FF;
			TXBUF[12]=datasum;//У����

			TXCOUNT=0;//���ͼ�����
			flagSEND = 1;//�������ݱ�־λ
			SendDataNumber = 13;
//        switch (SerialNumber)
//        {
//            case 00: break;
//        }
		}
		else if ((SerialNumber < 119)&&(SerialNumber > 43))//���Բ���\����ϵ��\��ͣ��������·���Ӧ������ȷ����
		{
			datasum=0;
			TXBUF[0] = 0XFE;//��ͷ
			TXBUF[1] = 0XFE;//��ͷ
			TXBUF[2] = 0XFE;//��ͷ
			TXBUF[3] = 0XFE;//��ͷ
			TXBUF[4] = 0X04;//����
			TXBUF[5] = SerialNumber; //���к�
			TXBUF[6] = CommandCode; //������
			TXBUF[7] = ConfirmCode;
			datasum = TXBUF[4]+TXBUF[5]+TXBUF[6]+TXBUF[7];
			datasum = (~datasum)+1; //У���룺���ȡ����1
			datasum &= 0X00FF;
			TXBUF[8]=datasum; //У����
			TXCOUNT=0;
			flagSEND = 1;
			SendDataNumber = 9;
		}
		else if (CommandCode == 0xB1)   //���ػ�
		{
			TXBUF[0] = 0XFE;//��ͷ
			TXBUF[1] = 0XFE;//��ͷ
			TXBUF[2] = 0XFE;//��ͷ
			TXBUF[3] = 0XFE;//��ͷ
			TXBUF[4] = 0X04;//����
			TXBUF[5] = SerialNumber; //���к�
			TXBUF[6] = CommandCode; //������
			TXBUF[7] = ConfirmCode; //ȷ����
			datasum = TXBUF[4]+TXBUF[5]+TXBUF[6]+TXBUF[7];
			datasum = ~datasum+1; //У���룺���ȡ����1
			TXBUF[8]=datasum; //У����
			TXCOUNT=0;
			flagSEND = 1;
			SendDataNumber = 9;
		}
	}
}


void Checkdata(void)//�����ж�
{
        if(TXCOUNT==0) //���Ͷ���Ϊ��
        {
            if(flagRC==1)//���ն���Ϊ��,��ʾ���ܶ�������ɣ����յ������ݰ���ȫ��ȷ
            {
                unsigned int i;
                ReciveRCOUNT=0;
				if(PackLength==19)//�����Ƿ����19���ж��Ƿ�Ϊ����Ⱥ����
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
	                datasum = 0;//xyyע�⣬�˱�����ø�Ϊ�ֲ�����
	                //��֤���������Ƿ���ȷ������ͷ�����һ��У���������ȡ����Ȼ����У����Ƚ�
	                for(i=0;i<(PackLength-1);i++)
	                {
	                    datasum += RC_DataBUF[i];
	                }
	                datasum += PackLength;
	                datasum =~ datasum+1;
	                datasum &= 0X00FF;
	                datasum1 = datasum ;

				   if(datasum == CheckCode) //����������ȷ��datasum =У���룩
				   {
						if(PackLength==5&&CommandCode==0xB1)//�����Ƿ����5-�жϿ��ػ�����
						{
							//if(CommandCode==0xB1)
							//{
								//Switchsystem = RC_DataBUF[2]+((RC_DataBUF[3]<<8)&0xff00);
								Data_get.bit.MEM1=RC_DataBUF[2];
								Data_get.bit.MEM2=RC_DataBUF[3];
								Switchsystem=Data_get.all;
							//}
						}
						if(PackLength==7)//�����Ƿ����7//�����������Ǵӻ�����ʾ�����������޸���Ҫ���Ƕ�Ӧ�Ĵӻ�
						{
							FData_get.bit.MEM1=RC_DataBUF[2];
							FData_get.bit.MEM2=RC_DataBUF[3];
							FData_get.bit.MEM3=RC_DataBUF[4];
							FData_get.bit.MEM4=RC_DataBUF[5];
							Paramet[SerialNumber]=FData_get.all;
						}
						TXdeal(); //�������ж��ͷ��ͳ���
					}
					else //���ո�ʽ����
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



//SCI���ʹ�����
void SciSend(void)
{
    if( ScicRegs.SCICTL2.bit.TXRDY == 1)  //�������Ļ���������λ�Ĵ�����Ϊ�գ�ÿ�η���ǰ��Ҫȷ��һ������Ĵ����ı�־λΪ1
    {
        if(flagSEND == 1)
        {
            //һ�����ݻ�û������,���պͷ��Ͱ�����ͬʱ��SendDataNumber��TXdeal()�и���
            if(TXCOUNT < SendDataNumber)
            {
                ScicRegs.SCITXBUF = TXBUF[TXCOUNT] ;//& 0X00FF;//���ͣ����ٴ�ȷ��Ϊ8λ���ݣ�����Ĵ����������������ֽڵ�
                TXCOUNT++;
            }
            else//һ�������Ѿ����ͽ���
            {
                flagRC = 0;
                flagSEND = 0;
                TXCOUNT = 0;
                //ClrTxbuf();
            }
        }
    }
}


//PSO���ͳ���
//��ʱ�������͸���λ��
void PSOsend(float U[10])
{
	if( ScicRegs.SCICTL2.bit.TXRDY == 1)//����ǰ��Ҫȷ��һ������Ĵ����ı�־λΪ1
	{
		if(PSOSENDF)//������0.05s�󣬷���һ������
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

				PSOBUF[PSONumber-1]=0XFF;//����У�����
				PSO_datainit_flag=1;
			}
			else
			{
				if(PSOCOUNT < PSONumber)
				{
					ScicRegs.SCITXBUF = PSOBUF[PSOCOUNT] ;//& 0X00FF;//���ͣ����ٴ�ȷ��Ϊ8λ���ݣ�����Ĵ����������������ֽڵ�
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





