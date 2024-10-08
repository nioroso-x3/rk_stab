# rk_stab
Image stabilization and UDP RTP streaming application. Focused on RK3588. Streams two independ H265 streams and writes a high bandwidth H264 stream to the SD card.

Requires OpenCV with GStreamer support, OpenCL C++ headers and libjsonrpc. There are two example scripts showing how to use the streamer. Stabilization can be set to just translation, rotation is optional. I had to use OpenCL due to the terrible driver support for the rk3588 GPU, gltransform does not work properly for rotations, constantly dropping frames with both panform and the propietary driver.

Some resolutions crash the RGA scaler, like 800x600. Use multiples of 16 for the transmission resolution dimensions.

A H264 high bandwidth stream is saved to the SD card, to avoid loading the H265 encoder. Both streams are stabilized if stabilization is enabled. A raw video shm source can be used to capture the original decompressed video to save it, process it through image recognitioning, or other uses.

Uses JSON-RPC to control various options, see set_params.sh for examples.
