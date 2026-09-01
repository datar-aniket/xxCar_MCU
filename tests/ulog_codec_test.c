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
struct flow  { uint64_t t; uint32_t it,tdd; float ix,iy,ixg,iyg,izg,dist; int16_t temp; uint8_t q,sid,pad[4]; };
struct dist  { uint64_t t; float cur,mn,mx; uint8_t type,ori,cov,sq; };
struct vimu  { uint64_t t,ts,tf; float da[3],dv[3],adt,vdt; uint16_t n,rst; uint8_t inst,clip,acal,gcal; };
struct estate { uint64_t t,ts; float q[4],v[3],p[3],gb[3],ab[3],av[3],vv[3],pv[3]; uint32_t pred,cov; uint16_t rst; uint8_t status,inst; };
struct extpose { uint64_t t,ts; float x,y,yaw,cov[6]; uint8_t flags,rst,pad[2]; };
struct ediag { uint64_t t,ts,ets; float sf[3],cf[3],grav[3],res[3],na[3],q[4],v[3],p[3],gb[3],ab[3],ei[2],enis[2],em[3],znis[3],gnis,anorm,avar,gdev,eratio,wheel; uint32_t ea,er,za,zr,ga,gr; uint16_t rst,flags; uint8_t inst,pad[7]; };
struct vaccel { uint64_t t,ts; float x,y,z; uint8_t inst,cal,pad[2]; };
struct vst { uint64_t t,ts,ats,wts; float p[3],q[4],v[3],w[3],slip,a[3],torque,steer,speed; uint32_t rc; uint8_t status,rst,src,pad; };
#pragma pack(pop)
int main(void){
  f=fopen("all.ulg","wb");
  uint8_t magic[8]={0x55,0x4c,0x6f,0x67,0x01,0x12,0x35,0x01}; uint64_t ts=1000; put(magic,8); put(&ts,8);
  uint8_t flag[40]; memset(flag,0,40); msg('B',flag,40);
  const char*F[]={
    "sensor_accel:uint64_t timestamp;float x;float y;float z;float temperature;",
    "sensor_baro:uint64_t timestamp;float pressure;float temperature;",
    "sensor_mag:uint64_t timestamp;float x;float y;float z;float temperature;int32_t status;",
    "rc_input:uint64_t timestamp;uint16_t[18] channel;uint16_t frames;uint16_t lost_frames;uint8_t count;uint8_t rssi;uint8_t ok;uint8_t failsafe;uint8_t source;",
    "optical_flow:uint64_t timestamp;uint32_t integration_time_us;uint32_t time_delta_distance_us;float integrated_x;float integrated_y;float integrated_xgyro;float integrated_ygyro;float integrated_zgyro;float distance;int16_t temperature;uint8_t quality;uint8_t sensor_id;",
    "distance_sensor:uint64_t timestamp;float current_distance;float min_distance;float max_distance;uint8_t type;uint8_t orientation;uint8_t covariance;uint8_t signal_quality;",
    "vehicle_imu:uint64_t timestamp;uint64_t timestamp_sample;uint64_t timestamp_first;float[3] delta_angle;float[3] delta_velocity;float delta_angle_dt;float delta_velocity_dt;uint16_t samples;uint16_t reset_counter;uint8_t instance;uint8_t clipping;uint8_t accel_calibrated;uint8_t gyro_calibrated;",
    "estimator_state:uint64_t timestamp;uint64_t timestamp_sample;float[4] quaternion;float[3] velocity;float[3] position;float[3] gyro_bias;float[3] accel_bias;float[3] angle_variance;float[3] velocity_variance;float[3] position_variance;uint32_t predict_count;uint32_t covariance_count;uint16_t reset_counter;uint8_t solution_status;uint8_t instance;",
    "external_pose:uint64_t timestamp;uint64_t timestamp_sample;float x;float y;float yaw;float[6] cov;uint8_t flags;uint8_t reset_counter;",
    "estimator_diag:uint64_t timestamp;uint64_t timestamp_sample;uint64_t extnav_timestamp;float[3] specific_force;float[3] corrected_force;float[3] gravity_body;float[3] residual_accel_body;float[3] nav_accel;float[4] quaternion;float[3] velocity;float[3] position;float[3] gyro_bias;float[3] accel_bias;float[2] extnav_innov;float[2] extnav_nis;float[3] extnav_measurement;float[3] zupt_nis;float gravity_nis;float accel_norm;float accel_variance;float gravity_deviation;float extnav_test_ratio;float wheel_speed_cps;uint32_t extnav_accept_count;uint32_t extnav_reject_count;uint32_t zupt_accept_count;uint32_t zupt_reject_count;uint32_t gravity_accept_count;uint32_t gravity_reject_count;uint16_t reset_counter;uint16_t flags;uint8_t instance;",
    "vehicle_accel:uint64_t timestamp;uint64_t timestamp_sample;float x;float y;float z;uint8_t instance;uint8_t calibrated;",
    "vehicle_state_tx:uint64_t timestamp;uint64_t timestamp_sample;uint64_t accel_timestamp_sample;uint64_t wire_timestamp_us;float[3] position;float[4] quaternion;float[3] velocity;float[3] angular_velocity;float side_slip_rad;float[3] accel;float wheel_torque_nm;float steering_angle;float motor_speed_ms;uint32_t rc_status;uint8_t solution_status;uint8_t reset_counter;uint8_t source_valid;"};
  if(strlen(F[4])<=256){fprintf(stderr,"optical_flow regression fixture must exceed 256 bytes\n");return 2;}
  for(int i=0;i<12;i++) msg('F',F[i],strlen(F[i]));
  const char*N[]={"sensor_accel","sensor_baro","sensor_mag","rc_input","optical_flow","distance_sensor","vehicle_imu","estimator_state","external_pose","estimator_diag","vehicle_accel","vehicle_state_tx"};
  for(uint16_t id=0;id<12;id++){uint8_t a[40];size_t l=0;a[l++]=0;a[l++]=id&0xff;a[l++]=id>>8;memcpy(a+l,N[id],strlen(N[id]));l+=strlen(N[id]);msg('A',a,l);}
  for(int k=0;k<4;k++){
    struct accel a={.t=1000+k*500,.x=0.10f*k,.y=-0.20f,.z=9.81f,.temp=34.5f}; dat(0,&a,24);
    struct baro  b={.t=1000+k*500,.p=1013.25f,.temp=27.9f};                   dat(1,&b,16);
    struct mag   m={.t=1000+k*500,.x=0.21f,.y=0.11f,.z=0.41f,.temp=33.0f,.st=1}; dat(2,&m,28);
    struct rc    r={.t=1000+k*500,.cnt=16,.rssi=200,.ok=1}; r.ch[0]=1500; r.ch[1]=1000; dat(3,&r,53);
    struct flow  fl={.t=1000+k*500,.it=10000,.ix=0.05f,.iy=-0.03f,.dist=1.25f,.q=200}; dat(4,&fl,44);
    struct dist  ds={.t=1000+k*500,.cur=1.30f,.mn=0.1f,.mx=8.0f,.sq=90}; dat(5,&ds,24);
    struct vimu vi={.t=1000+k*500,.ts=900+k*500,.tf=400+k*500,.dv={0.01f,0.02f,0.0245f},.adt=0.0025f,.vdt=0.0025f,.n=5,.acal=1,.gcal=1}; dat(6,&vi,64);
    struct estate es={.t=1000+k*500,.ts=900+k*500,.q={1.0f},.v={0.1f,0.2f,0.3f},.p={1.0f,2.0f,3.0f},.pred=42,.cov=10,.status=0x61}; dat(7,&es,128);
    struct extpose ep={.t=1000+k*500,.ts=850+k*500,.x=1.2f,.y=-0.4f,.yaw=0.3f,.flags=1}; dat(8,&ep,54);
    struct ediag ed={.t=900+k*500,.ts=900+k*500,.ets=850+k*500,.sf={0.04f,0.08f,9.80f},.cf={0.03f,0.07f,9.79f},.grav={0.0f,0.0f,9.80665f},.res={0.03f,0.07f,-0.01665f},.na={0.03f,0.07f,-0.01665f},.q={1.0f},.v={0.1f,0.2f,0.3f},.p={1.0f,2.0f,3.0f},.em={1.2f,-0.4f,0.3f},.anorm=9.79f,.ea=3,.za=5,.ga=7,.flags=0x1143}; dat(9,&ed,241);
    struct vaccel va={.t=1000+k*500,.ts=990+k*500,.x=0.04f,.y=0.08f,.z=9.80f,.cal=1}; dat(10,&va,30);
    struct vst vs={.t=1010+k*500,.ts=900+k*500,.ats=990+k*500,.wts=1700000000000000ull+k*500,.p={1.0f,2.0f,3.0f},.q={1.0f},.a={0.03f,0.08f,-0.01f},.rc=0x075c85f0,.status=0x61,.src=0x1f}; dat(11,&vs,119);
  }
  fclose(f); return 0;
}
