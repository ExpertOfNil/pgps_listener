#ifndef CIMPL_GLM_H
#define CIMPL_GLM_H

#include <fcntl.h>
#include <math.h>
#include <unistd.h>

#include "cimpl_core.h"

#define POSE_PRINT_FORMAT \
    "[%010d] %9.3f, %9.3f, %9.3f,%9.3f, %9.3f, %9.3f, %9.3f\n"
#define QUAT_IDENTITY {0.0f, 0.0f, 0.0f, 1.0f};
// clang-format off
#define MAT3_IDENTITY { \
    1.0f, 0.0f, 0.0f, \
    0.0f, 1.0f, 0.0f, \
    0.0f, 0.0f, 1.0f, \
}
// clang-format on
// clang-format off
#define MAT4_IDENTITY { \
    1.0f, 0.0f, 0.0f, 0.0f, \
    0.0f, 1.0f, 0.0f, 0.0f, \
    0.0f, 0.0f, 1.0f, 0.0f, \
    0.0f, 0.0f, 0.0f, 1.0f, \
}
// clang-format on

typedef enum {
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2,
} Axis;

typedef struct Vec3 {
    f32 x, y, z;
} Vec3;

DEFINE_DYNAMIC_ARRAY(Vec3, Vec3Array)

typedef struct Vec3Node Vec3Node;
struct Vec3Node {
    u32 index;
    Vec3* item;
    Vec3Node* left;
    Vec3Node* right;
};

DEFINE_DYNAMIC_ARRAY(Vec3Node, Vec3Tree)

typedef struct StlTriangle {
    Vec3 normal;
    Vec3 vertices[3];
    u16 attributes;
} StlTriangle;

DEFINE_DYNAMIC_ARRAY(StlTriangle, StlTriangleArray)

typedef struct Vec4 {
    f32 x, y, z, w;
} Vec4;

typedef struct Vec4 Quat;

typedef struct Mat3 {
    f32 xi, xj, xk;
    f32 yi, yj, yk;
    f32 zi, zj, zk;
} Mat3;

typedef struct Mat4 {
    f32 xi, xj, xk, xw;
    f32 yi, yj, yk, yw;
    f32 zi, zj, zk, zw;
    f32 ti, tj, tk, tw;
} Mat4;

/*** FUNCTION DECLARATIONS ***/

f32 Vec3_length(Vec3 v);
Vec3 Vec3_normalize(Vec3 v);
Vec3 Vec3_cross(Vec3 a, Vec3 b);
Vec3 Vec3_translation_from_mat4(Mat4 m);

void Vec3Tree_print(Vec3Tree*);
i32 Vec3Tree_partition(Vec3Tree*, Axis, i32, i32);
void Vec3Tree_quicksort(Vec3Tree*, Axis, i32, i32);
CimplReturn Vec3Tree_sort(Vec3Tree*, Axis, const Vec3Array*);

f32 Quat_length(Quat q);
Quat Quat_normalize(Quat q);

Mat3 Mat3_from_quat(Quat q);
Mat3 Mat3_rotation_from_mat4(Mat4 m);

Mat4 Mat4_with_rotation(Mat4 dst, Mat3 src);
void Mat4_print_with_id(Mat4 m, const char* id);
Mat4 Mat4_from_translation_quat(Vec3 t, Quat q);
Mat4 Mat4_inverse_rigid(Mat4 src);
Mat4 Mat4_mul(Mat4 a, Mat4 b);
Mat4 Mat4_orthonormalize(Mat4 m);

CimplReturn StlTriangleArray_from_binary(const char*, StlTriangleArray*);

void Vec3Tree_print(Vec3Tree* arr);
i32 Vec3Tree_partition(Vec3Tree* node_arr, Axis axis, i32 start, i32 end);
void Vec3Tree_quicksort(Vec3Tree* node_arr, Axis axis, i32 start, i32 end);
CimplReturn Vec3Tree_sort(
    Vec3Tree* node_arr, Axis axis, const Vec3Array* pt_arr
);

/*** FUNCTION DEFINITIONS ***/

#ifdef CIMPL_IMPLEMENTATION

/* Vec3 */
f32 Vec3_length(Vec3 v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

Vec3 Vec3_normalize(Vec3 v) {
    f32 mag = Vec3_length(v);
    v.x = v.x / mag;
    v.y = v.y / mag;
    v.z = v.z / mag;
    return v;
}

Vec3 Vec3_cross(Vec3 a, Vec3 b) {
    Vec3 c = {0};
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

Vec3 Vec3_translation_from_mat4(Mat4 m) { return (Vec3){m.ti, m.tj, m.tk}; }

/* Quat */
f32 Quat_length(Quat q) {
    return sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}
Quat Quat_normalize(Quat q) {
    f32 mag = Quat_length(q);
    q.x = q.x / mag;
    q.y = q.y / mag;
    q.z = q.z / mag;
    q.w = q.w / mag;
    return q;
}

/* Mat3 */
Mat3 Mat3_from_quat(Quat q) {
    // Ensure it is normalized
    q = Quat_normalize(q);
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    Mat3 dst;

    dst.xi = 1.0f - 2.0f * (yy + zz);
    dst.xj = 2.0f * (xy + wz);
    dst.xk = 2.0f * (xz - wy);

    dst.yi = 2.0f * (xy - wz);
    dst.yj = 1.0f - 2.0f * (xx + zz);
    dst.yk = 2.0f * (yz + wx);

    dst.zi = 2.0f * (xz + wy);
    dst.zj = 2.0f * (yz - wx);
    dst.zk = 1.0f - 2.0f * (xx + yy);

    return dst;
}

Mat3 Mat3_rotation_from_mat4(Mat4 m) {
    return (Mat3){
        // clang-format off
        m.xi, m.xj, m.xk,
        m.yi, m.yj, m.yk,
        m.zi, m.zj, m.zk,
        // clang-format on
    };
}

/* Mat4 */
Mat4 Mat4_with_rotation(Mat4 dst, Mat3 src) {
    dst.xi = src.xi;
    dst.xj = src.xj;
    dst.xk = src.xk;
    dst.yi = src.yi;
    dst.yj = src.yj;
    dst.yk = src.yk;
    dst.zi = src.zi;
    dst.zj = src.zj;
    dst.zk = src.zk;
    return dst;
}

void Mat4_print_with_id(Mat4 m, const char* id) {
    printf(
        "%s: [\n"
        "  %9.3f, %9.3f, %9.3f, %9.3f\n"
        "  %9.3f, %9.3f, %9.3f, %9.3f\n"
        "  %9.3f, %9.3f, %9.3f, %9.3f\n"
        "  %9.3f, %9.3f, %9.3f, %9.3f\n"
        "]\n",
        id,
        // clang-format off
        m.xi, m.yi, m.zi, m.ti,
        m.xj, m.yj, m.zj, m.tj,
        m.xk, m.yk, m.zk, m.tk,
        m.xw, m.yw, m.zw, m.tw
        // clang-format on
    );
}

Mat4 Mat4_from_translation_quat(Vec3 t, Quat q) {
    Mat4 m = MAT4_IDENTITY;
    m.ti = t.x;
    m.tj = t.y;
    m.tk = t.z;
    Mat3 R = Mat3_from_quat(q);
    return Mat4_with_rotation(m, R);
}

Mat4 Mat4_inverse_rigid(Mat4 src) {
    Mat4 dst = MAT4_IDENTITY;
    Vec3 x = Vec3_normalize((Vec3){src.xi, src.xj, src.xk});
    Vec3 y = Vec3_normalize((Vec3){src.yi, src.yj, src.yk});
    Vec3 z = Vec3_normalize((Vec3){src.zi, src.zj, src.zk});
    dst.xi = x.x;
    dst.xj = y.x;
    dst.xk = z.x;

    dst.yi = x.y;
    dst.yj = y.y;
    dst.yk = z.y;

    dst.zi = x.z;
    dst.zj = y.z;
    dst.zk = z.z;

    dst.ti = -(x.x * src.ti + x.y * src.tj + x.z * src.tk);
    dst.tj = -(y.x * src.ti + y.y * src.tj + y.z * src.tk);
    dst.tk = -(z.x * src.ti + z.y * src.tj + z.z * src.tk);

    return dst;
}

Mat4 Mat4_mul(Mat4 a, Mat4 b) {
    Mat4 dst = MAT4_IDENTITY;
    dst.xi = a.xi * b.xi + a.yi * b.xj + a.zi * b.xk + a.ti * b.xw;
    dst.xj = a.xj * b.xi + a.yj * b.xj + a.zj * b.xk + a.tj * b.xw;
    dst.xk = a.xk * b.xi + a.yk * b.xj + a.zk * b.xk + a.tk * b.xw;
    dst.xw = a.xw * b.xi + a.yw * b.xj + a.zw * b.xk + a.tw * b.xw;

    dst.yi = a.xi * b.yi + a.yi * b.yj + a.zi * b.yk + a.ti * b.yw;
    dst.yj = a.xj * b.yi + a.yj * b.yj + a.zj * b.yk + a.tj * b.yw;
    dst.yk = a.xk * b.yi + a.yk * b.yj + a.zk * b.yk + a.tk * b.yw;
    dst.yw = a.xw * b.yi + a.yw * b.yj + a.zw * b.yk + a.tw * b.yw;

    dst.zi = a.xi * b.zi + a.yi * b.zj + a.zi * b.zk + a.ti * b.zw;
    dst.zj = a.xj * b.zi + a.yj * b.zj + a.zj * b.zk + a.tj * b.zw;
    dst.zk = a.xk * b.zi + a.yk * b.zj + a.zk * b.zk + a.tk * b.zw;
    dst.zw = a.xw * b.zi + a.yw * b.zj + a.zw * b.zk + a.tw * b.zw;

    dst.ti = a.xi * b.ti + a.yi * b.tj + a.zi * b.tk + a.ti * b.tw;
    dst.tj = a.xj * b.ti + a.yj * b.tj + a.zj * b.tk + a.tj * b.tw;
    dst.tk = a.xk * b.ti + a.yk * b.tj + a.zk * b.tk + a.tk * b.tw;
    dst.tw = a.xw * b.ti + a.yw * b.tj + a.zw * b.tk + a.tw * b.tw;

    return dst;
}

Mat4 Mat4_orthonormalize(Mat4 m) {
    Mat4 dst = MAT4_IDENTITY;
    // Ensure x-axis is orthogonal to the yz-plane
    Vec3 x_axis =
        Vec3_cross((Vec3){m.yi, m.yj, m.yk}, (Vec3){m.zi, m.zj, m.zk});
    x_axis = Vec3_normalize(x_axis);
    dst.xi = x_axis.x;
    dst.xj = x_axis.y;
    dst.xk = x_axis.z;
    // Ensure y-axis is orthogonal to the xz-plane
    Vec3 y_axis =
        Vec3_cross((Vec3){m.zi, m.zj, m.zk}, (Vec3){dst.xi, dst.xj, dst.xk});
    y_axis = Vec3_normalize(y_axis);
    dst.yi = y_axis.x;
    dst.yj = y_axis.y;
    dst.yk = y_axis.z;
    // Ensure z-axis is normalized
    Vec3 z_axis = Vec3_normalize((Vec3){m.zi, m.zj, m.zk});
    dst.zi = z_axis.x;
    dst.zj = z_axis.y;
    dst.zk = z_axis.z;

    dst.ti = m.ti;
    dst.tj = m.tj;
    dst.tk = m.tk;
    return dst;
}

CimplReturn StlTriangleArray_from_binary(
    const char* fpath, StlTriangleArray* triangles
) {
    i32 fd = open(fpath, O_RDONLY);
    if (fd < 0) {
        log_error("Failed to open %s", fpath);
        return RETURN_ERR;
    }
    u8 header[80] = {0};
    isize read_bytes = read(fd, &header, 80);
    if (read_bytes != 80) {
        log_error("Failed to read header when parsing %s", fpath);
        goto error;
    }
    u32 triangle_ct = 0;
    read_bytes = read(fd, &triangle_ct, sizeof(triangle_ct));
    if (read_bytes != sizeof(triangle_ct)) {
        log_error("Failed to read number of triangles when parsing %s", fpath);
        goto error;
    }

    StlTriangle triangle = {0};
    for (u32 i = 0; i < triangle_ct; ++i) {
        read_bytes = read(fd, &triangle, sizeof(triangle));
        if (read_bytes < (u32)sizeof(triangle)) {
            log_error("Failed to read triangle %d from %s", i, fpath);
            close(fd);
            return RETURN_ERR;
        }
        StlTriangleArray_push(triangles, triangle);
    }
    close(fd);
    return RETURN_OK;
error:
    close(fd);
    return RETURN_ERR;
}

void Vec3Tree_print(Vec3Tree* arr) {
    for (u32 i = 0; i < arr->count; ++i) {
        Vec3* pt = arr->items[i].item;
        printf(
            "%2d: [%2d][%9.3f, %9.3f, %9.3f]\n",
            i,
            arr->items[i].index,
            pt->x,
            pt->y,
            pt->z
        );
    }
}

i32 Vec3Tree_partition(Vec3Tree* node_arr, Axis axis, i32 start, i32 end) {
    f32 pivot_value = ((f32*)(node_arr->items[end].item))[axis];
    i32 i = start - 1;
    for (i32 j = start; j < end; ++j) {
        f32 value = ((f32*)(node_arr->items[j].item))[axis];
        if (value <= pivot_value) {
            i++;
            Vec3Node tmp = node_arr->items[j];
            memcpy(&node_arr->items[j], &node_arr->items[i], sizeof(Vec3Node));
            memcpy(&node_arr->items[i], &tmp, sizeof(Vec3Node));
        }
    }
    i++;
    Vec3Node tmp = node_arr->items[end];
    memcpy(&node_arr->items[end], &node_arr->items[i], sizeof(Vec3Node));
    memcpy(&node_arr->items[i], &tmp, sizeof(Vec3Node));
    return i;
}

void Vec3Tree_quicksort(Vec3Tree* node_arr, Axis axis, i32 start, i32 end) {
    if (end <= start) return;
    i32 pivot = Vec3Tree_partition(node_arr, axis, start, end);
    Vec3Tree_quicksort(node_arr, axis, start, pivot - 1);
    Vec3Tree_quicksort(node_arr, axis, pivot + 1, end);
}

CimplReturn Vec3Tree_sort(
    Vec3Tree* node_arr, Axis axis, const Vec3Array* pt_arr
) {
    // Ensure we have enough capacity
    // TODO (mmckenna): if not, reserve
    if (node_arr->capacity < pt_arr->count) {
        log_error(
            "Node capacity %d must match point count %d",
            node_arr->capacity,
            pt_arr->count
        );
        return RETURN_ERR;
    }

    // TODO (mmckenna): ensure node_arr is cleared out and count = 0
    // Create references
    for (u32 i = 0; i < pt_arr->count; ++i) {
        Vec3Node node;
        node.index = i;
        node.item = &pt_arr->items[i];
        node.left = NULL;
        node.right = NULL;
        Vec3Tree_push(node_arr, node);
    }

    Vec3Tree_quicksort(node_arr, axis, 0, pt_arr->count - 1);

    return RETURN_OK;
}

#endif /* CIMPL_IMPLEMENTATION */

#endif /* CIMPL_GLM_H */
