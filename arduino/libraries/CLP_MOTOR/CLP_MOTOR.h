//�{���W��:���j���B�i���F �������
//�����GV1.4
//�t�X�w��:HB860H�X�ʾ�
//�t�X�w��:���j���B�i���F
//�t�X�w��:TIP120 �q���鱱��q��
//����Y/N:N
//�Ƶ��G
//�ϥ�Atmel ATmega 2560 TIMER���gPULSE
//�]�wpulse 15(15us)�t�׳̧֡A���i�C��15
//�]�w65531��20us���_�@��
//�]�w���U1�����s ���F�]CLP_StepSet�ӨB��
//�X�ʾ��]�w��1600��pulse���@��
//�s�WENABLE����
//�ק���~Pulse HI LOW����BJT�쥻�ۤ�
//�{���s�g����G2017/10/29(��)
#ifndef CLP_MOTOR_h
#define CLP_MOTOR_h
#include <inttypes.h>

class CLPMTR
{
private:  
  uint8_t CLPMTR_PUL;
  uint8_t CLPMTR_DIR;  
  uint8_t CLPMTR_ENA;  
public:    
  CLPMTR();    //�غc
  ~CLPMTR();       //�Ѻc
  //boolean cw_ccw;
  uint8_t DIRST;
  enum DIR{cw=1,ccw};
  void CLP_MOTOR_Initial(uint8_t PUL,uint8_t DIR);
  void CLP_MOTOR_Initial_all(uint8_t PUL,uint8_t DIR,uint8_t ENA);
  void setCLPMTR_Forward();
  void setCLPMTR_Reverse();  
  boolean setCLPMTR_CW();
  boolean setCLPMTR_CCW();
  void setCLPMTR_HIGH();
  void setCLPMTR_LOW();
  void setCLPMTR_Enable();
  void setCLPMTR_Disable();	
};

#endif
