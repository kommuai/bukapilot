#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rkutil.h"

uint as_uint(const float x) {
    return *(uint*)&x;
}
float as_float(const uint x) {
    return *(float*)&x;
}

float half_to_float(half x) { // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
    const uint e = (x&0x7C00)>>10; // exponent
    const uint m = (x&0x03FF)<<13; // mantissa
    const uint v = as_uint((float)m)>>23; // evil log2 bit hack to count leading zeros in denormalized format
    return as_float((x&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23|((m<<(150-v))&0x007FE000))); // sign : normalized : denormalized
}

half float_to_half(float x)
{
    suf32 in;
    in.f          = x;
    unsigned sign = in.u & 0x80000000;
    in.u ^= sign;
    ushort w;

    if (in.u >= 0x47800000)
      w = (ushort)(in.u > 0x7f800000 ? 0x7e00 : 0x7c00);
    else {
      if (in.u < 0x38800000) {
        in.f += 0.5f;
        w = (ushort)(in.u - 0x3f000000);
      } else {
        unsigned t = in.u + 0xc8000fff;
        w          = (ushort)((t + ((in.u >> 13) & 1)) >> 13);
      }
    }

    w = (ushort)(w | (sign >> 16));

    return w;
}

void float_to_half_array(float *src, half *dst, int size)
{
    for (int i = 0; i < size; i++)
    {
        dst[i] = float_to_half(src[i]);
    }
}

void half_to_float_array(half *src, float *dst, int size)
{
    for (int i = 0; i < size; i++)
    {
        dst[i] = half_to_float(src[i]);
    }
}

int _rknn_app_nchw_2_nhwc(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size){
    // transpose nchw to nhwc, consider w align.
    int ret = 0;
    int N = src->dims[0];
    int C = src->dims[1];
    int H = src->dims[2];
    int W = src->dims[3];

    int dst_W;
    if (dst->size != dst->size_with_stride){
        int attr_dtype_size = dst->size / dst->n_elems;
        dst_W = dst->size_with_stride / (attr_dtype_size * N * C * H);
    }
    else{
        dst_W = W;
    }

    int src_HW = H * W;
    int src_CHW = C * src_HW;
    int dst_WC = dst_W * C;
    int dst_HWC = H * dst_WC;

    for (int n = 0; n < N; n++){
        for (int h = 0; h < H; h++){
            for (int w = 0; w < W; w++){
                for (int c = 0; c < C; c++){
                    int src_idx = n * src_CHW + c * src_HW + h * W + w;
                    int dst_idx = n * dst_HWC + h * dst_WC + w * C + c;
                    memcpy(dst_ptr + dst_idx * type_size, src_ptr + src_idx * type_size, type_size);
                }
            }
        }
    }
    return ret;
}


int _rknn_app_nhwc_2_nchw(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size){
    // transpose nhwc to nchw, consider w align.
    int ret = 0;
    int N = src->dims[0];
    int H = src->dims[1];
    int W = src->dims[2];
    int C = src->dims[3];

    int dst_W = dst->dims[3];

    int src_WC = W * C;
    int src_HWC = H * src_WC;
    int dst_HW = H * dst_W;
    int dst_CHW = C * dst_HW;

    for (int n = 0; n < N; n++){
        for (int c = 0; c < C; c++){
            for (int h = 0; h < H; h++){
                for (int w = 0; w < dst_W; w++){
                    int src_idx = n * src_HWC + h * src_WC + w * C + c;
                    int dst_idx = n * dst_CHW + c * dst_HW + h * dst_W + w;
                    memcpy(dst_ptr + dst_idx * type_size, src_ptr + src_idx * type_size, type_size);
                }
            }
        }
    }
    return ret;
}


int _rknn_app_nchw_2_nc1hwc2(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size){
    // transpose nchw to nc1hwc2, consider c2 align
    int ret = 0;
    int N = src->dims[0];
    int C = src->dims[1];
    int H = src->dims[2];
    int W = src->dims[3];

    int C1 = dst->dims[1];
    int C2 = dst->dims[4];

    int src_HW = H * W;
    int src_CHW = C * src_HW;
    int dst_WC2 = W * C2;
    int dst_HWC2 = H * dst_WC2;
    int dst_C1HWC2 = C1 * dst_HWC2;

    for (int n = 0; n < N; n++){
        for (int c = 0; c < C; c++){
            for (int h = 0; h < H; h++){
                for (int w = 0; w < W; w++){
                    int c1 = c / C2;
                    int c2 = c % C2;
                    int src_idx = n * src_CHW + c * src_HW + h * W + w;
                    int dst_idx = n * dst_C1HWC2 + c1 * dst_HWC2 + h * dst_WC2 + w * C2 + c2;
                    memcpy(dst_ptr + dst_idx * type_size, src_ptr + src_idx * type_size, type_size);
                }
            }
        }
    }
    return ret;
}


int _rknn_app_nc1hwc2_2_nchw(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size){
    int ret = 0;

    int N = src->dims[0];
    int C1 = src->dims[1];
    int H = src->dims[2];
    int W = src->dims[3];
    int C2 = src->dims[4];

    int C = dst->dims[1];

    int src_WC2 = W * C2;
    int src_HWC2 = H * src_WC2;
    int src_C1HWC2 = C1 * src_HWC2;
    int dst_HW = H * W;
    int dst_CHW = C * dst_HW;

    for (int n = 0; n < N; n++){
        for (int c = 0; c < C; c++){
            for (int h = 0; h < H; h++){
                for (int w = 0; w < W; w++){
                    int c1 = c / C2;
                    int c2 = c % C2;
                    int src_idx = n * src_C1HWC2 + c1 * src_HWC2 + h * src_WC2 + w * C2 + c2;
                    int dst_idx = n * dst_CHW + c * dst_HW + h * W + w;
                    memcpy(dst_ptr + dst_idx * type_size, src_ptr + src_idx * type_size, type_size);
                }
            }
        }
    }

    return ret;
}

int _rknn_app_nhwc_2_nc1hwc2(rknn_tensor_attr *src, unsigned char* src_ptr, rknn_tensor_attr *dst, unsigned char* dst_ptr, int type_size){
    int ret = 0;
    int N = src->dims[0];
    int H = src->dims[1];
    int W = src->dims[2];
    int C = src->dims[3];

    int C1 = dst->dims[1];
    int C2 = dst->dims[4];

    int src_WC = W * C;
    int src_HWC = H * src_WC;
    int dst_WC2 = W * C2;
    int dst_HWC2 = H * dst_WC2;
    int dst_C1HWC2 = C1 * dst_HWC2;

    for (int n = 0; n < N; n++){
        for (int c = 0; c < C; c++){
            for (int h = 0; h < H; h++){
                for (int w = 0; w < W; w++){
                    int c1 = c / C2;
                    int c2 = c % C2;
                    int src_idx = n * src_HWC + h * src_WC + w * C + c;
                    int dst_idx = n * dst_C1HWC2 + c1 * dst_HWC2 + h * dst_WC2 + w * C2 + c2;
                    memcpy(dst_ptr + dst_idx * type_size, src_ptr + src_idx * type_size, type_size);
                }
            }
        }
    }

    return ret;
}


// data_type may not match attr.dtype, type_size is needed
int rknn_app_layout_convert(unsigned char* src_ptr,
                            rknn_tensor_attr *src,
                            unsigned char* dst_ptr,
                            rknn_tensor_attr *dst,
                            int type_size,
                            bool verbose)
{
    char src_dims_str[100];
    memset(src_dims_str, 0, sizeof(src_dims_str));
    for (int i = 0; i < src->n_dims; i++) {
        sprintf(src_dims_str, "%s%d,", src_dims_str, src->dims[i]);
    }

    char dst_dims_str[100];
    memset(dst_dims_str, 0, sizeof(dst_dims_str));
    for (int i = 0; i < dst->n_dims; i++) {
        sprintf(dst_dims_str, "%s%d,", dst_dims_str, dst->dims[i]);
    }
    if (verbose){
        LOGD("    rknn_layout_convert: src->fmt=%s(%s), dst->fmt=%s(%s)\n", get_format_string(src->fmt), src_dims_str, get_format_string(dst->fmt), dst_dims_str);
    }

    if (src->fmt == dst->fmt){
        LOGD("    layout unchanged, memcpy directly\n");
        memcpy(dst_ptr, src_ptr, src->n_elems * type_size);
        return 0;
    }

    int ret = 0;
    if (src->fmt == RKNN_TENSOR_NCHW && dst->fmt == RKNN_TENSOR_NHWC){
        ret = _rknn_app_nchw_2_nhwc(src, src_ptr, dst, dst_ptr, type_size);
    }else if (src->fmt == RKNN_TENSOR_NCHW && dst->fmt == RKNN_TENSOR_NC1HWC2){
        ret = _rknn_app_nchw_2_nc1hwc2(src, src_ptr, dst, dst_ptr, type_size);
    }else if (src->fmt == RKNN_TENSOR_NHWC && dst->fmt == RKNN_TENSOR_NC1HWC2){
        ret = _rknn_app_nhwc_2_nc1hwc2(src, src_ptr, dst, dst_ptr, type_size);
    }else if (src->fmt == RKNN_TENSOR_NHWC && dst->fmt == RKNN_TENSOR_NCHW){
        ret = _rknn_app_nhwc_2_nchw(src, src_ptr, dst, dst_ptr, type_size);
    }else if (src->fmt == RKNN_TENSOR_NC1HWC2 && dst->fmt == RKNN_TENSOR_NCHW){
        ret = _rknn_app_nc1hwc2_2_nchw(src, src_ptr, dst, dst_ptr, type_size);
    }else{
        LOGD("    rknn_layout_convert: not support layout convert from %s to %s\n", get_format_string(src->fmt), get_format_string(dst->fmt));
        ret = -1;
    }

    return ret;
}

int get_type_size(rknn_tensor_type type){
    switch (type){
        case RKNN_TENSOR_INT8:
            return sizeof(int8_t);
        case RKNN_TENSOR_UINT8:
            return sizeof(uint8_t);
        case RKNN_TENSOR_INT16:
            return sizeof(int16_t);
        case RKNN_TENSOR_UINT16:
            return sizeof(uint16_t);
        case RKNN_TENSOR_INT32:
            return sizeof(int32_t);
        case RKNN_TENSOR_UINT32:
            return sizeof(uint32_t);
        case RKNN_TENSOR_INT64:
            return sizeof(int64_t);
        case RKNN_TENSOR_FLOAT16:
            return sizeof(half);
        case RKNN_TENSOR_FLOAT32:
            return sizeof(float);
        default:
            LOGD("    get_type_size error: not support dtype %d\n", type);
            return 0;
    }
}


int rknn_app_dtype_convert(unsigned char* src_ptr,
                         rknn_tensor_type src_dtype,
                         unsigned char* dst_ptr,
                         rknn_tensor_type dst_dtype,
                         int n_elems, float scale, int zero_point, bool verbose)
{
    int type_size = get_type_size(dst_dtype);

    if (src_dtype == dst_dtype){
        // for keep same function logic, still create new buffer.
        memcpy(dst_ptr, src_ptr, n_elems * type_size);
        return 0;
    }

    if (verbose){
        LOGD("    rknn_dtype_convert: convert from %s to %s\n", get_type_string(src_dtype), get_type_string(dst_dtype));
    }
    int convert_success = 0;
    switch (src_dtype){
        case RKNN_TENSOR_FLOAT32:
            if (dst_dtype == RKNN_TENSOR_FLOAT16){
                float_to_half_array((float*)src_ptr, (half*)dst_ptr, n_elems);

            } else if (dst_dtype == RKNN_TENSOR_INT8){
                for (int i = 0; i < n_elems; i++){
                    ((int8_t*)dst_ptr)[i] = (int8_t)(((float*)src_ptr)[i] / scale + zero_point);
                }
            } else if (dst_dtype == RKNN_TENSOR_INT16){
                for (int i = 0; i < n_elems; i++){
                    ((int16_t*)dst_ptr)[i] = (int16_t)(((float*)src_ptr)[i] / scale + zero_point);
                }
            } else if (dst_dtype == RKNN_TENSOR_INT32){
                for (int i = 0; i < n_elems; i++){
                    ((int32_t*)dst_ptr)[i] = (int32_t)(((float*)src_ptr)[i]);
                }
            } else if (dst_dtype == RKNN_TENSOR_INT64){
                for (int i = 0; i < n_elems; i++){
                    ((int64_t*)dst_ptr)[i] = (int64_t)(((float*)src_ptr)[i]);
                }
            } else {
                convert_success = -1;
            }
            break;

        case RKNN_TENSOR_FLOAT16:
            if (dst_dtype == RKNN_TENSOR_FLOAT32){
                half_to_float_array((half*)src_ptr, (float*)dst_ptr, n_elems);
            } else {
                convert_success = -1;
            }
            break;

        case RKNN_TENSOR_INT8:
            if (dst_dtype == RKNN_TENSOR_FLOAT32){
                for (int i = 0; i < n_elems; i++){
                    ((float*)dst_ptr)[i] = (float)(((int8_t*)src_ptr)[i] - zero_point) * scale;
                }
            } else {
                convert_success = -1;
            }
            break;

        case RKNN_TENSOR_INT16:
            if (dst_dtype == RKNN_TENSOR_FLOAT32){
                for (int i = 0; i < n_elems; i++){
                    ((float*)dst_ptr)[i] = (float)(((int16_t*)src_ptr)[i] - zero_point) * scale;
                }
            } else {
                convert_success = -1;
            }
            break;

        case RKNN_TENSOR_INT32:
            if (dst_dtype == RKNN_TENSOR_FLOAT32){
                for (int i = 0; i < n_elems; i++){
                    ((float*)dst_ptr)[i] = (float)(((int32_t*)src_ptr)[i]);
                }
            } else {
                convert_success = -1;
            }
            break;

        case RKNN_TENSOR_INT64:
            if (dst_dtype == RKNN_TENSOR_FLOAT32){
                for (int i = 0; i < n_elems; i++){
                    ((float*)dst_ptr)[i] = (float)(((int64_t*)src_ptr)[i]);
                }
            } else {
                convert_success = -1;
            }
            break;

        default:
            convert_success = -1;
            break;
    }

    if (convert_success == -1){
        LOGD("    rknn_dtype_convert: not support dtype convert from %s to %s\n", get_type_string(src_dtype), get_type_string(dst_dtype));
    }

    return convert_success;
}

void dump_tensor_attr(rknn_tensor_attr* attr)
{
  LOGD("index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n, passthrough=%d,",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3], attr->dims[4],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale, attr->pass_through);
}

void print_execution_time(rknn_perf_run* obj)
{
  LOGD("rknn model execution time = %ldms", obj->run_duration);
}