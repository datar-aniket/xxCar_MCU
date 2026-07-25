#include <stdio.h>
#include <stdint.h>
#include <string.h>
static FILE*f; static void put(const void*d,size_t n){fwrite(d,1,n,f);}
static void msg(uint8_t t,const void*p,uint16_t l){uint8_t h[3]={l&0xff,l>>8,t};put(h,3);put(p,l);}
static void dat(uint16_t id,const void*rec,uint16_t rs){uint8_t h[3]={(2+rs)&0xff,(2+rs)>>8,'D'};put(h,3);uint8_t i[2]={id&0xff,id>>8};put(i,2);put(rec,rs);}
#pragma pack(push,1)
struct accel { uint64_t t; float x,y,z,temp; };
struct baro  { uint64_t t; float p,temp; };
struct mag   { uint64_t t; float x,y,z,temp; int32_t st; uint8_t pad[4]; };
struct rc    { uint64_t t; uint16_t ch[18]; uint16_t fr,lost; uint8_t cnt,rssi,ok,fs,src,pad[3]; };
#pragma pack(pop)
int main(void){
  f=fopen("all.ulg","wb");
  uint8_t magic[8]={0x55,0x4c,0x6f,0x67,0x01,0x12,0x35,0x01}; uint64_t ts=1000; put(magic,8); put(&ts,8);
  uint8_t flag[40]; memset(flag,0,40); msg('B',flag,40);
  const char*F[]={
    "sensor_accel:uint64_t timestamp;float x;float y;float z;float temperature;",
    "sensor_baro:uint64_t timestamp;float pressure;float temperature;",
    "sensor_mag:uint64_t timestamp;float x;float y;float z;float temperature;int32_t status;",
    "rc_input:uint64_t timestamp;uint16_t[18] channel;uint16_t frames;uint16_t lost_frames;uint8_t count;uint8_t rssi;uint8_t ok;uint8_t failsafe;uint8_t source;"};
  for(int i=0;i<4;i++) msg('F',F[i],strlen(F[i]));
  const char*N[]={"sensor_accel","sensor_baro","sensor_mag","rc_input"};
  for(uint16_t id=0;id<4;id++){uint8_t a[40];size_t l=0;a[l++]=0;a[l++]=id&0xff;a[l++]=id>>8;memcpy(a+l,N[id],strlen(N[id]));l+=strlen(N[id]);msg('A',a,l);}
  for(int k=0;k<4;k++){
    struct accel a={.t=1000+k*500,.x=0.10f*k,.y=-0.20f,.z=9.81f,.temp=34.5f}; dat(0,&a,24);
    struct baro  b={.t=1000+k*500,.p=1013.25f,.temp=27.9f};                   dat(1,&b,16);
    struct mag   m={.t=1000+k*500,.x=0.21f,.y=0.11f,.z=0.41f,.temp=33.0f,.st=1}; dat(2,&m,28);
    struct rc    r={.t=1000+k*500,.cnt=16,.rssi=200,.ok=1}; r.ch[0]=1500; r.ch[1]=1000; dat(3,&r,53);
  }
  fclose(f); return 0;
}
