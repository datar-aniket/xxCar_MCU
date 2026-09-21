/* SPDX-License-Identifier: Apache-2.0
 * Execute production S1-S8 driver against three independent timer models.
 * Register sequencing/phase tests are NOT an electrical jitter measurement.
 */
#include <nuttx/config.h>
#include <errno.h>
#include <string.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_rcc.h"
#include "hardware/stm32_tim.h"
extern int printf(const char *, ...);
extern int puts(const char *);
extern void exit(int);
#define CHECK(x) do { if (!(x)) {printf("FAIL %d: %s\n", __LINE__, #x); exit(1);} } while (0)
#define REG(b,o) model.tim[b][(o)/4]
static const uint32_t bases[] = {STM32_TIM8_BASE, STM32_TIM2_BASE, STM32_TIM4_BASE};
static const unsigned banks[] = {0,0,1,1,1,1,2,2};
static const unsigned channels[] = {1,2,0,1,2,3,0,1};
static const unsigned ports[] = {1,1,0,0,0,0,3,3};
static const unsigned pins[] = {0,1,0,1,2,3,12,13};
static struct
{
  uint32_t tim[3][32], gpio[4][16], en[2], rst[2], d2;
  uint32_t arr[3], cc[3][4], frames[3][32][4];
  unsigned nf[3], pulses[8], depth, gpio_calls, fail_gpio, writes;
  uint64_t us, rise[8], previous[8];
  bool level[8], wave;
  int wd_result;
  wdentry_t timeout;
  wdparm_t timeout_arg;
} model;

static void sample(void)
{
  for (unsigned i=0;i<8;i++)
    {
      unsigned b=banks[i], c=channels[i], p=pins[i];
      uint32_t e=REG(b,STM32_GTIM_CCER_OFFSET)>>(c*4);
      bool alt=((model.gpio[ports[i]][STM32_GPIO_MODER_OFFSET/4]>>(p*2))&3)==GPIO_MODER_ALT;
      bool ref=REG(b,STM32_GTIM_CNT_OFFSET)<model.cc[b][c];
      bool enabled;
      if (b==0)
        {
          enabled=(e&4) && (REG(b,STM32_ATIM_BDTR_OFFSET)&ATIM_BDTR_MOE);
          /* ST N-only table: enabling the main output would invert N. */
          ref ^= (e&1)!=0;
          ref ^= (e&8)!=0;
        }
      else {enabled=e&1; ref^=(e&2)!=0;}
      bool high=alt && enabled && ref;
      if (high && !model.level[i])
        {
          CHECK(REG(b,STM32_GTIM_CR1_OFFSET)&GTIM_CR1_CEN);
          if(model.wave && model.previous[i]) CHECK(model.us-model.previous[i]==model.arr[b]+1);
          model.rise[i]=model.previous[i]=model.us;
        }
      if (!high && model.level[i])
        {
          if(model.wave)
            {
              CHECK(model.us-model.rise[i]==model.cc[b][c]);
              CHECK(model.cc[b][c]>=900 && model.cc[b][c]<=2100);
            }
          model.pulses[i]++;
        }
      model.level[i]=high;
    }
}
static void transfer(unsigned b)
{
  model.arr[b]=REG(b,STM32_GTIM_ARR_OFFSET);
  for(unsigned c=0;c<4;c++) model.cc[b][c]=REG(b,STM32_GTIM_CCR1_OFFSET+c*4);
}
static void tick(void)
{
  model.us++;
  for(unsigned b=0;b<3;b++)
    if(REG(b,STM32_GTIM_CR1_OFFSET)&GTIM_CR1_CEN)
      {
        if(REG(b,STM32_GTIM_CNT_OFFSET)>=model.arr[b])
          {
            REG(b,STM32_GTIM_CNT_OFFSET)=0;
            if(!(REG(b,STM32_GTIM_CR1_OFFSET)&GTIM_CR1_UDIS)) transfer(b);
            if(model.nf[b]<32) memcpy(model.frames[b][model.nf[b]++],model.cc[b],sizeof(model.cc[b]));
          }
        else REG(b,STM32_GTIM_CNT_OFFSET)++;
      }
  sample();
}
static void advance(unsigned n) {while(n--) tick();}
static uint32_t *ptr(uint32_t a)
{
  for(unsigned b=0;b<3;b++) if(a>=bases[b] && a<bases[b]+128) return &model.tim[b][(a-bases[b])/4];
  for(unsigned p=0;p<4;p++)
    if(a>=STM32_GPIOA_BASE+p*0x400 && a<STM32_GPIOA_BASE+p*0x400+64)
      return &model.gpio[p][(a-STM32_GPIOA_BASE-p*0x400)/4];
  switch(a)
    {
      case STM32_RCC_APB1LENR:return &model.en[0];
      case STM32_RCC_APB2ENR:return &model.en[1];
      case STM32_RCC_APB1LRSTR:return &model.rst[0];
      case STM32_RCC_APB2RSTR:return &model.rst[1];
      case STM32_RCC_D2CFGR:return &model.d2;
      default:CHECK(false);return 0;
    }
}
static uint32_t rd(uint32_t a) {return *ptr(a);}
static void wr(uint32_t v,uint32_t a)
{
  CHECK(model.depth);
  model.writes++;
  for(unsigned b=0;b<3;b++)
    {
#ifdef TEST_WITHOUT_UDIS
      if(a==bases[b]+STM32_GTIM_CR1_OFFSET) v&=~GTIM_CR1_UDIS;
#endif
      if ((b==0 && a==STM32_RCC_APB2RSTR && (v&RCC_APB2RSTR_TIM8RST)) ||
          (b==1 && a==STM32_RCC_APB1LRSTR && (v&RCC_APB1LRSTR_TIM2RST)) ||
          (b==2 && a==STM32_RCC_APB1LRSTR && (v&RCC_APB1LRSTR_TIM4RST)))
        {memset(model.tim[b],0,sizeof(model.tim[b]));memset(model.cc[b],0,sizeof(model.cc[b]));model.arr[b]=UINT32_MAX;}
      if(a==bases[b]+STM32_GTIM_EGR_OFFSET)
        {
          CHECK(!(REG(b,STM32_GTIM_CR1_OFFSET)&GTIM_CR1_CEN));
          CHECK(REG(b,STM32_GTIM_CCER_OFFSET)==0);
          transfer(b);REG(b,STM32_GTIM_CNT_OFFSET)=0;
        }
    }
  *ptr(a)=v;
  sample();tick();
}
static void mod(uint32_t a,uint32_t c,uint32_t s) {wr((rd(a)&~c)|s,a);}
static int gpio(uint32_t g)
{
  unsigned p=(g&GPIO_PORT_MASK)>>GPIO_PORT_SHIFT, n=g&GPIO_PIN_MASK;
  CHECK(p<4 && model.depth);
  if(++model.gpio_calls==model.fail_gpio) return -EIO;
  uint32_t *m=&model.gpio[p][STM32_GPIO_MODER_OFFSET/4];
  uint32_t *a=&model.gpio[p][(n<8?STM32_GPIO_AFRL_OFFSET:STM32_GPIO_AFRH_OFFSET)/4];
  *m=(*m&~(3u<<(n*2)))|(((g&GPIO_MODE_MASK)>>GPIO_MODE_SHIFT)<<(n*2));
  *a=(*a&~(15u<<((n%8)*4)))|(((g&GPIO_AF_MASK)>>GPIO_AF_SHIFT)<<((n%8)*4));
  sample();tick();return 0;
}
static irqstate_t enter(void) {return model.depth++;}
static void leave(irqstate_t f) {CHECK(model.depth==f+1);model.depth=f;}
static int wd(struct wdog_s *w,clock_t d,wdentry_t e,wdparm_t a)
{(void)w;CHECK(model.depth && d==MSEC2TICK(200));if(!model.wd_result){model.timeout=e;model.timeout_arg=a;}return model.wd_result;}
static int cancel(struct wdog_s *w) {(void)w;model.timeout=0;return 0;}
#undef getreg16
#undef getreg32
#undef putreg16
#undef putreg32
#undef enter_critical_section
#undef leave_critical_section
#define getreg16(a) ((uint16_t)rd(a))
#define getreg32(a) rd(a)
#define putreg16(v,a) wr(v,a)
#define putreg32(v,a) wr(v,a)
#define modifyreg32(a,c,s) mod(a,c,s)
#define stm32_configgpio(g) gpio(g)
#define enter_critical_section() enter()
#define leave_critical_section(f) leave(f)
#define wd_start(w,d,e,a) wd(w,d,e,a)
#define wd_cancel(w) cancel(w)
#include "../boards/fmuv6c/src/matekh743_pwm.c"
static void reset(void)
{
  memset(&model,0,sizeof(model));memset(&g_status,0,sizeof(g_status));
  memset(g_ccer,0,sizeof(g_ccer));memset(g_ccmr,0,sizeof(g_ccmr));
  model.wave=true;model.d2=STM32_RCC_D2CFGR_D2PPRE1|STM32_RCC_D2CFGR_D2PPRE2;
}
static void frames(const uint16_t *old,const uint16_t *next)
{
  for(unsigned b=0;b<3;b++) for(unsigned f=0;f<model.nf[b];f++)
    {
      bool was=true,now=true;
      for(unsigned i=0;i<8;i++) if(banks[i]==b)
        {was &= model.frames[b][f][channels[i]]==old[i];now &= model.frames[b][f][channels[i]]==next[i];}
      CHECK(was || now); /* model.frames must never contain a torn bank. */
    }
}
int main(void)
{
  uint16_t old[8]={1100,1200,1300,1400,1500,1600,1700,1800};
  uint16_t next[8]={1900,1800,1700,1600,1400,1300,1200,1100};
  uint16_t safe[8]={1500,1500,1500,1500,1500,1500,1500,1500};
  reset();CHECK(board_matek_pwm_set(old,safe)==-ENODEV);
  CHECK(board_matek_pwm_start(401,255,old)==-EINVAL);
  CHECK(board_matek_pwm_start(50,0,old)==-EINVAL);
  model.d2=0;CHECK(board_matek_pwm_start(50,255,old)==-EIO);CHECK(!model.writes);
  for(unsigned f=1;f<=16;f++)
    {reset();model.fail_gpio=f;CHECK(board_matek_pwm_start(50,255,old)==-EIO);CHECK(!g_status.mask);for(unsigned i=0;i<8;i++)CHECK(!model.level[i]);}
  /* Every nonempty channel subset, including every ordered steering pair. */
  for(unsigned mask=1;mask<=255;mask++)
    {
      reset();CHECK(board_matek_pwm_start(50,mask,old)==0);
      CHECK(board_matek_steering_pwm_healthy());advance(42000);
      for(unsigned i=0;i<8;i++) CHECK((model.pulses[i]>0)==((mask&(1u<<i))!=0));
      model.wave=false;board_matek_pwm_stop();CHECK(!g_status.mask);
      for(unsigned i=0;i<8;i++)CHECK(!model.level[i]);
    }
  for(unsigned b=0;b<3;b++) for(unsigned phase=0;phase<20;phase++)
    {
      reset();CHECK(board_matek_pwm_start(50,255,old)==0);advance(40000);
      advance((model.arr[b]-REG(b,STM32_GTIM_CNT_OFFSET)+20000-phase)%20000);
      memset(model.nf,0,sizeof(model.nf));
      CHECK(board_matek_pwm_set(next,safe)==0);advance(45000);frames(old,next);
      memset(model.nf,0,sizeof(model.nf));CHECK(model.timeout);model.timeout(model.timeout_arg);
      advance(45000);frames(next,safe);
      for(unsigned i=0;i<8;i++)for(unsigned j=0;j<8;j++)
        if(banks[i]==banks[j])CHECK(model.previous[i]==model.previous[j]);
    }
  reset();CHECK(board_matek_pwm_start(400,255,old)==0);
  unsigned writes=model.writes;
  CHECK(board_matek_pwm_start(50,255,old)==-EBUSY);
  CHECK(board_matek_pwm_start(400,3,old)==-EBUSY);CHECK(writes==model.writes);
  model.wd_result=-EIO;CHECK(board_matek_pwm_set(next,safe)==-EIO);advance(10000);frames(old,safe);
  REG(0,STM32_ATIM_BDTR_OFFSET)=0;CHECK(!board_matek_steering_pwm_healthy());
  model.wave=false;CHECK(board_matek_pwm_set(next,safe)==-EIO);CHECK(!g_status.mask);
  CHECK(model.depth==0);
  {
    const unsigned rates[]={25,50,333,400};
    uint32_t random=0xabc123;
    for(unsigned r=0;r<4;r++)
      {
        reset();CHECK(board_matek_pwm_start(rates[r],255,safe)==0);
        for(unsigned j=0;j<150;j++)
          {
            random=random*1664525u+1013904223u;
            advance(1+random%11000);
            for(unsigned i=0;i<8;i++)next[i]=900+((random>>(i*3))%1201);
            CHECK(board_matek_pwm_set(next,safe)==0);
          }
        advance(85000);
        for(unsigned i=0;i<8;i++) CHECK(model.pulses[i]>15);
      }
  }
  puts("PASS: S1-S8 all masks, N-only polarity, aligned bank edges, rollover atomicity, timeout, rollback");
  return 0;
}
