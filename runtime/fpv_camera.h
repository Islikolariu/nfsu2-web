#ifndef NFSU2_FPV_CAMERA_H
#define NFSU2_FPV_CAMERA_H

#include "fpv_flight.h"
#include <math.h>

/* Free Roam still streams around the car. Keep this experimental camera in
 * the same 100 m envelope as Photo Mode until it has its own streaming. */
static inline void fpv_camera_limit(fpv_flight *f, const float anchor[3]){
    if (f->p[2] < anchor[2]){
        f->p[2] = anchor[2];
        f->v[2] = fmaxf(f->v[2], 0.f);
    }
    float d[3] = { f->p[0] - anchor[0], f->p[1] - anchor[1], f->p[2] - anchor[2] };
    float dist = sqrtf(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    if (dist > 100.f){
        float outward = (d[0]*f->v[0] + d[1]*f->v[1] + d[2]*f->v[2]) / dist;
        for (int i = 0; i < 3; ++i){
            f->p[i] = anchor[i] + d[i] * (100.f / dist);
            f->v[i] -= d[i] / dist * fmaxf(outward, 0.f);
        }
    }
}

/* Tilt the camera up by 46 degrees relative to the airframe, then convert
 * its full basis to the debug mover's target and the view's roll. */
static inline float fpv_camera_view(const fpv_flight *f, float look[3], float *yaw){
    const float mount_cos = 0.69465837f, mount_sin = 0.71933980f;
    float forward[3], up[3];
    fpv_flight_basis(f, forward, up);
    for (int i = 0; i < 3; ++i){
        float body_forward = forward[i];
        forward[i] = body_forward * mount_cos + up[i] * mount_sin;
        up[i] = up[i] * mount_cos - body_forward * mount_sin;
    }
    float flat = hypotf(forward[0], forward[1]);
    if (flat > 0.001f) *yaw = atan2f(forward[1], forward[0]);
    float cx = cosf(*yaw), sy = sinf(*yaw);
    float right[3] = { sy, -cx, 0.f };
    float level_up[3] = { -forward[2] * cx, -forward[2] * sy, flat };
    look[0] = f->p[0] + 10.f * fmaxf(flat, 0.001f) * cx;
    look[1] = f->p[1] + 10.f * fmaxf(flat, 0.001f) * sy;
    look[2] = f->p[2] + 10.f * forward[2];
    float sine = up[0]*right[0] + up[1]*right[1];
    float cosine = up[0]*level_up[0] + up[1]*level_up[1] + up[2]*level_up[2];
    return atan2f(-sine, cosine);
}

#endif
