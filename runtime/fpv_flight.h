#ifndef NFS_FPV_FLIGHT_H
#define NFS_FPV_FLIGHT_H
/* Assisted / quaternion Acro prototypes, metres / seconds, Z up.
 * Not a calibrated airframe or Betaflight simulation. */
typedef struct { float p[3], v[3], origin[3], yaw, pitch, roll; int limited;
    float q[4],omega[3],motor; int acro; float mass_kg; } fpv_flight;
typedef struct { float roll, pitch, yaw, lift; } fpv_controls;
void fpv_flight_reset(fpv_flight *s, const float origin[3], float yaw);
void fpv_flight_step(fpv_flight *s, const fpv_controls *in, float dt);
void fpv_flight_basis(const fpv_flight *s, float forward[3], float up[3]);
void fpv_acro_reset(fpv_flight *s,const float origin[3],float yaw);
void fpv_acro_step(fpv_flight *s,const fpv_controls *in,float dt);
void fpv_acro_set_mass(fpv_flight *s,float grams);
#endif
