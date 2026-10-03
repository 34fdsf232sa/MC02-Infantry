// 黄金向量生成器：把【原版 F4 固件】的 mi_motor.c 原样编译到主机上，用它的真实输出做回归基准。
//
// 重新生成：
//   F4=/path/to/Infantry-Robot          # github.com/34fdsf232sa/Infantry-Robot @ 1dfdc2f
//   L=$F4/Libraries; INC="-Itests/golden -I$L/Device/Motor/Inc -I$L/Algorithm/Math/Inc"
//   gcc -std=gnu99 -w $INC -c $L/Device/Motor/Src/mi_motor.c -o /tmp/mi.o
//   gcc -std=gnu99 -w $INC -c $L/Algorithm/Math/Src/alg_math.c -o /tmp/am.o
//   gcc -std=gnu99 -w $INC tests/golden/gen_f4_vectors.c /tmp/mi.o /tmp/am.o -lm -o /tmp/gen
//   /tmp/gen > /tmp/golden.txt && python3 tests/golden/make_inc.py /tmp/golden.txt > tests/golden/mit_motor_f4_vectors.inc
// 用【原版 F4 mi_motor.c】生成黄金向量：编码（Output）与解码（Data_Process）
#include "headfile.h"
CAN_Manage_Object CAN1_Manage_Object, CAN2_Manage_Object;
static uint8_t last_tx[8];
uint8_t CAN_Send_Data(CAN_HandleTypeDef *h, uint16_t id, uint8_t *d, uint16_t n){ memcpy(last_tx,d,8); return 0; }

static void enc(float pmax,float vmax,float tmax,float ang,float om,float kp,float kd,float tq){
  Class_MI_Motor m; memset(&m,0,sizeof m); CAN_HandleTypeDef h={CAN1}; MI_Motor_Structure_Init(&m);
  MI_Motor_Init(&m,&h,0x06,0x05,pmax,vmax,tmax);
  MI_Motor_Set_Control_Angle(&m,ang); MI_Motor_Set_Control_Omega(&m,om);
  MI_Motor_Set_Control_Torque(&m,tq); MI_Motor_Set_K_P(&m,kp); MI_Motor_Set_K_D(&m,kd);
  MI_Motor_Send_PeriodElapsedCallback(&m);     // 含限幅，和实车调用路径一致
  printf("ENC %.9g %.9g %.9g  %.9g %.9g %.9g %.9g %.9g  ",pmax,vmax,tmax,ang,om,kp,kd,tq);
  for(int i=0;i<8;i++) printf("%02x",last_tx[i]); printf("\n");
}
static void dec(float pmax,float vmax,float tmax,const uint8_t*rx){
  Class_MI_Motor m; memset(&m,0,sizeof m); CAN_HandleTypeDef h={CAN1}; MI_Motor_Structure_Init(&m);
  MI_Motor_Init(&m,&h,0x06,0x05,pmax,vmax,tmax);
  memcpy(CAN1_Manage_Object.Rx_Buffer.Data,rx,8);
  MI_Motor_CAN_RxCpltCallback(&m,(uint8_t*)rx);
  printf("DEC %.9g %.9g %.9g  ",pmax,vmax,tmax); for(int i=0;i<8;i++) printf("%02x",rx[i]);
  printf("  %.6f %.6f %.6f %g %g %d\n",m.Rx_Data.Now_Angle,m.Rx_Data.Now_Omega,m.Rx_Data.Now_Torque,
         m.Rx_Data.Now_MOS_Temperature,m.Rx_Data.Now_Rotor_Temperature,(int)m.Rx_Data.Control_Status);
}
int main(void){
  const float P=95.5f,V=45.0f,T=18.0f;   // 实车 chassis.c 使用的量程
  float ang[]={0,1.0f,-1.0f,0.5f,-1.9635f,-13.46f,30.0f,95.5f,-95.5f};
  for(unsigned i=0;i<sizeof ang/sizeof*ang;i++) enc(P,V,T,ang[i],0,45.0f,2.5f,0);     // 位置保持
  float om[]={0,5.0f,-5.0f,44.9f,-44.9f,45.0f,-45.0f}; for(unsigned i=0;i<7;i++) enc(P,V,T,0,om[i],0,1.0f,0);
  float tq[]={0,1.0f,-1.0f,7.0f,-7.0f,17.9f,-17.9f,18.0f,-18.0f}; for(unsigned i=0;i<9;i++) enc(P,V,T,0,0,0,0,tq[i]);
  enc(P,V,T,0,0,0,0,0); enc(P,V,T,0,0,500.0f,5.0f,0); enc(P,V,T,0,0,600.0f,9.0f,0); enc(P,V,T,0,0,-3.0f,-1.0f,0);
  enc(12.5f,30.0f,10.0f,1.0f,2.0f,20.0f,1.0f,3.0f);                                   // DM4310 量程
  // 解码：byte0 = status<<4 | can_id(低4位=Tx_ID&0x0f=5)，其余为 位置/速度/力矩/温度
  uint8_t r1[8]={0x15,0x80,0x00,0x80,0x08,0x00,0x28,0x2d};   // 中位附近
  uint8_t r2[8]={0x15,0xc0,0x00,0x90,0x78,0x90,0x30,0x32};
  uint8_t r3[8]={0x85,0x40,0x00,0x70,0x87,0x70,0x1e,0x1f};   // status=8 过压
  uint8_t r4[8]={0x16,0x80,0x00,0x80,0x08,0x00,0x28,0x2d};   // can_id=6 ≠ 5 -> 应被忽略
  uint8_t r5[8]={0x15,0x00,0x00,0x00,0x00,0x00,0xf6,0xfb};   // 全最小，负温度
  uint8_t r6[8]={0x15,0xff,0xff,0xff,0xff,0xff,0x7f,0x80};   // 全最大
  uint8_t r7[8]={0x15,0x80,0x80,0x90,0x78,0x90,0x30,0x32};   // b1==b2：原版位置解码恰好正确
  uint8_t r8[8]={0x25,0x12,0x34,0x7f,0xf7,0xff,0x19,0x1a};   // b1!=b2：高低字节都有信息
  uint8_t r9[8]={0x15,0x7f,0xff,0x7f,0xf7,0xff,0x19,0x1a};   // 刚好过零点
  const uint8_t*rs[]={r1,r2,r3,r4,r5,r6,r7,r8,r9}; for(int i=0;i<9;i++){ dec(P,V,T,rs[i]); dec(12.5f,45.0f,18.0f,rs[i]); }
  return 0;
}
void osDelay(unsigned ms){(void)ms;}
