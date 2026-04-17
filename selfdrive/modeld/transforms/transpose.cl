__kernel void nchw_to_nhwc(
    __global const float* input,  // Input tensor in NCHW format
    __global float* output,       // Output tensor in NHWC format
    const int N,                  // Batch size
    const int C,                  // Number of channels
    const int H,                  // Height
    const int W)                  // Width
{
    // Total number of elements in the tensor.
    int total_elements = N * C * H * W;

    // Compute the global index for this work-item.
    int gid = get_global_id(0);

    // Make sure we do not process beyond the end of the tensor.
    if (gid >= total_elements) {
        return;
    }

    // Convert the flat index (NCHW) to (n, c, h, w) indices.
    int w_index = gid % W;
    int temp = gid / W;
    int h_index = temp % H;
    temp = temp / H;
    int c_index = temp % C;
    int n_index = temp / C;

    // Compute the corresponding index in NHWC.
    // In NHWC, the layout is: (n * H * W * C) with the innermost dimension being channels.
    int out_index = ((n_index * H + h_index) * W + w_index) * C + c_index;

    // Write the value to the output tensor.
    output[out_index] = input[gid];
}
