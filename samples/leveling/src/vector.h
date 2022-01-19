#ifndef VECTOR_H
#define VECTOR_H

#include <math.h>

#define PI         (3.1415926)
#define RAD_TO_DEG (180.0 / PI)

struct accel_vec {
    double x;
    double y;
    double z;
};

static inline void normalize(struct accel_vec *v) {
    double len = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
    v->x /= len; v->y /= len; v->z /= len;
}

static inline double dot(struct accel_vec *v1, struct accel_vec *v2) {
    return v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;
}

static inline double angle(struct accel_vec *v1, struct accel_vec *v2) {
    return acos(dot(v1, v2)) * RAD_TO_DEG;
}

#endif // VECTOR_H
