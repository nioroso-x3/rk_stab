#include <iostream>
#include <thread>
#include <cmath>
#include <string>
#include <csignal>
#include <filesystem>
#include <chrono>
#define CL_HPP_TARGET_OPENCL_VERSION 300
#include <CL/opencl.hpp>
#include <gst/gst.h>
//#include <gst/gl/gl.h>
#include <opencv2/opencv.hpp>
#include <gst/app/gstappsink.h>
#include "stubserver.h"
#include <jsonrpccpp/server/connectors/httpserver.h>
#include "stabilization.h"

#define CROP_P 0.05
using namespace cv;
using namespace jsonrpc;
//GstElement *gltransformation0;
//GstElement *gltransformation1;
GstElement *appsrc;
GstElement *videocrop;
GstElement *x265enc0, *x265enc1;
GstElement *rtph265pay0, *rtph265pay1;
GstElement *udpsink0, *udpsink1;
GstElement *selector0, *selector1;
GstPad *sel0_s0;
GstPad *sel0_s1;
GstPad *sel1_s0;
GstPad *sel1_s1;
stabilizer st;
VideoCapture cap;
std::thread frames;
int width = 1920;
int height = 1080;
int width_t0 = 1280;
int height_t0 = 720;
int width_t1 = width_t0/2;
int height_t1 = height_t0/2;
int fps = 30;
int crop_x = width*CROP_P;
int crop_y = height*CROP_P;
int bitrate0 = 2000*1000;
int bitrate1 = bitrate0/3;
int stab = 3;
int s0_cur = 0;
int s1_cur = 1;
int mtu0 = 1024;
int mtu1 = 1024;
std::string clients0;
std::string clients1;

// Signal handler
void signalHandler(int signal){

}

class JSONServer : public stubserver {
public:
  JSONServer(AbstractServerConnector &connector, serverVersion_t type);

  virtual void setParam(const Json::Value &args);
  virtual Json::Value getParam();
};

JSONServer::JSONServer(AbstractServerConnector &connector, serverVersion_t type) : stubserver(connector,type) {}
void JSONServer::setParam(const Json::Value &args){
    //print parameters
    auto itr = args.begin();
    int i = 0;
    for (itr = args.begin(); itr != args.end(); itr++){
      auto name = args.getMemberNames()[i];
      if (args[name].size() > 1) {
          for (auto x:args[name]) {
              std::cout << name << ": " << x << std::endl;
          }
      } else {
        std::cout << name + " ";
        std::cout << args[name] << std::endl;
      }
      i++;
    }
    if (args.isMember("stream0_bitrate") && args["stream0_bitrate"].isInt()){
      bitrate0 = args["stream0_bitrate"].asInt()*1000;
      g_object_set(x265enc0, "bps", bitrate0, NULL);

    }
    if (args.isMember("stream1_bitrate") && args["stream1_bitrate"].isInt()){ 
      bitrate1 = args["stream1_bitrate"].asInt()*1000;
      g_object_set(x265enc1, "bps", bitrate1, NULL);
    }
    if (args.isMember("stream0_mtu") && args["stream0_mtu"].isInt()){
      mtu0 = args["stream0_mtu"].asInt();
      g_object_set(rtph265pay0, "mtu", mtu0, NULL);

    }
    if (args.isMember("stream1_mtu") && args["stream1_mtu"].isInt()){ 
      mtu1 = args["stream1_mtu"].asInt();
      g_object_set(rtph265pay1, "mtu", mtu1, NULL);
    }


    if (args.isMember("stab") && args["stab"].isInt()) stab = args["stab"].asInt();

    if (args.isMember("clients0") && args["clients0"].isString()){
      clients0 = args["clients0"].asString();
      g_object_set(G_OBJECT(udpsink0), "clients", clients0.c_str(), NULL);

    }

    if (args.isMember("clients1") && args["clients1"].isString()){
      clients1 = args["clients1"].asString();
      g_object_set(G_OBJECT(udpsink1), "clients", clients1.c_str(), NULL);
    }

    if (args.isMember("output0_stream") && args["output0_stream"].isInt()){
      if(args["output0_stream"].asInt() == 0){ 
        g_object_set(selector0, "active-pad", sel0_s0, NULL);
        s0_cur = 0;
      }
      if(args["output0_stream"].asInt() == 1){
        g_object_set(selector0, "active-pad", sel0_s1, NULL); 
	s0_cur = 1;
      }
    }
    if (args.isMember("output1_stream") && args["output1_stream"].isInt()){
      if(args["output1_stream"].asInt() == 0){ 
        g_object_set(selector1, "active-pad", sel1_s0, NULL);
        s1_cur = 0;
      }
      if(args["output1_stream"].asInt() == 1){
        g_object_set(selector1, "active-pad", sel1_s1, NULL); 
	s1_cur = 1;
      }
    }

}

Json::Value JSONServer::getParam(){
  Json::Value result;
  result["stream0_bitrate"] = bitrate0/1000;
  result["stream1_bitrate"] = bitrate1/1000;
  result["stream0_mtu"] = mtu0;
  result["stream1_mtu"] = mtu1;
  result["stab"] = stab;
  result["clients0"] = clients0;
  result["clients1"] = clients1;
  result["output0_stream"] = s0_cur;
  result["output1_stream"] = s1_cur;
  return result;
}


// OpenCL kernel for rotating an image
const char *rotateKernelSource = R"(
__kernel void rotate_image(__global uchar4* input, __global uchar4* output, int width, int height, float cosTheta, float sinTheta) {
    int x = get_global_id(0);
    int y = get_global_id(1);
    int xc = width / 2;
    int yc = height / 2;

    int nx = (int)(cosTheta * (x - xc) - sinTheta * (y - yc) + xc);
    int ny = (int)(sinTheta * (x - xc) + cosTheta * (y - yc) + yc);

    if(nx >= 0 && nx < width && ny >= 0 && ny < height) {
        output[y * width + x] = input[ny * width + nx];
    } else {
        output[y * width + x] = (uchar4)(0, 0, 0, 0); 
    }
}
)";


// Callback to push data to appsrc
void process_frame() {
    Mat frame;
    Mat *image;
    Mat prevFrame;
    Mat output = Mat::zeros(width,height,CV_8UC4);
    
    int frameSize = width*height*4;

    std::cout << "Init OpenCL\n";
    // Set up OpenCL environment
      
    std::vector<cl::Platform> platforms;
    cl::Platform::get(&platforms);
    cl::Platform platform = platforms.front();

    std::vector<cl::Device> devices;
    platform.getDevices(CL_DEVICE_TYPE_GPU, &devices);
    cl::Device device = devices.front();
 
    cl::Context context(device);
    cl::CommandQueue queue(context, device);
    cl::Program program(context, rotateKernelSource, true);
    cl::Kernel rotateKernel(program, "rotate_image");
    cl::NDRange globalSize(width, height);
  
    std::cout << "Started reading frames\n";
    unsigned long int frame_count = 0;
    //some timers for benchmarking
    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;
    auto t3 = t1;
    auto t4 = t1;

    // Allocate OpenCL input and output buffers
    cl::Buffer inputBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, frameSize*sizeof(unsigned char));
    cl::Buffer outputBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, frameSize*sizeof(unsigned char));
    // Set fixed kernel arguments
    rotateKernel.setArg(0, inputBuffer);
    rotateKernel.setArg(1, outputBuffer);
    rotateKernel.setArg(2, width);
    rotateKernel.setArg(3, height);
    while(true){
        static GstClockTime timestamp = 0;    
        cap >> frame;
        t1 = std::chrono::high_resolution_clock::now();
        if (stab > 0){
            float x,y,a;
            st.stabilize(frame,&x,&y,&a);
            //avoid stabilization if the correction is too small
            //std::cout << x << " " << y << " "<< a << "\n";
            if (x < 0.002f && x > -0.002f) x = 0.0f;
            if (y < 0.002f && y > -0.002f) y = 0.0f;
            if (a < 0.001f && a > -0.001f) a = 0.0f;
            gfloat gx = x;
            gfloat gy = y;
            //clamp rotation angle to 2 degrees
            if (a > 0.035) a = 0.035;
            if (a < -0.035) a = -0.035;
           
            prevFrame = st.getPrevFrame();
            if (prevFrame.empty()) continue;
            image = &prevFrame;
            if(stab > 2){
                t3 = std::chrono::high_resolution_clock::now();
                cvtColor(prevFrame,prevFrame,COLOR_BGR2BGRA);        
                rotateKernel.setArg(4, std::cos(a));
                rotateKernel.setArg(5, std::sin(a));

               // Run the kernel
                queue.enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, frameSize*sizeof(unsigned char), (unsigned char*)prevFrame.datastart); 
                queue.enqueueNDRangeKernel(rotateKernel, cl::NullRange, globalSize, cl::NullRange);
                queue.enqueueReadBuffer(outputBuffer, CL_TRUE, 0, frameSize*sizeof(unsigned char), (unsigned char*)output.datastart);
                cvtColor(output,prevFrame,COLOR_BGRA2BGR);        
                t4 = std::chrono::high_resolution_clock::now();
            }
            g_object_set(videocrop, "top",    crop_y-(int)(gy*height),
                                    "bottom", crop_y+(int)(gy*height),
                                    "left",   crop_x-(int)(gx*width),
                                    "right",  crop_x+(int)(gx*width),
                                    NULL);
           
        }
        else{
            image = &frame;
            g_object_set(videocrop, "top",    0,
                                    "bottom", 0,
                                    "left",   0,
                                    "right",  0,
                                     NULL);

        } 
        // Create a new buffer
        GstBuffer *buffer;
        guint size;
        GstFlowReturn ret;

        size = image->total() * image->elemSize();
        buffer = gst_buffer_new_allocate(NULL, size, NULL);

        // Set the buffer data
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);
        memcpy(map.data, image->data, size);
        gst_buffer_unmap(buffer, &map);

        // Set the buffer timestamp and duration
        GST_BUFFER_PTS(buffer) = timestamp;
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, fps);
        timestamp += GST_BUFFER_DURATION(buffer);

        // Push the buffer into the appsrc
        g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);

        if (ret != GST_FLOW_OK) {
            g_printerr("Error pushing buffer to appsrc\n");
        }
        t2 = std::chrono::high_resolution_clock::now();
        int delay = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        int rot = std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count();
        //if (frame_count % 120 == 0) std::cout << "loop ms: " << delay-rot  << "\n" << "rot ms: " << rot << "\n";
        frame_count++;
    }
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    //std::signal(SIGINT, signalHandler);
    if (argc < 7) {
         std::cout << "Arguments are v4l2 device, clients for stream 1, clients for stream 2, ts filename and port for JSON RPC listener.\nInput width, input height, stream 1 width, stream 1 height, stream 2 width, stream 2 height, framerate, bitrate 1 kbps, bitrate 2 kbps, stabilization mode.\n";
     return -1;
    }
    const char* device = argv[1];
    clients0 = std::string(argv[2]);
    clients1 = std::string(argv[3]);
    const char* fout = argv[4];
    const char* shmout = argv[5];
    int port = std::stoi(argv[6]);
    if (argc == 17){
    std::cout << "Setting extra params\n";
        width = std::stoi(argv[7]);
        height = std::stoi(argv[8]);
        width_t0 = std::stoi(argv[9]);
        height_t0 = std::stoi(argv[10]);
        width_t1 = std::stoi(argv[11]);
        height_t1 = std::stoi(argv[12]);
        fps = std::stoi(argv[13]);
        bitrate0 = std::stoi(argv[14])*1000;
        bitrate1 = std::stoi(argv[15])*1000;
        stab = std::stoi(argv[16]);
        crop_x = width*CROP_P;
        crop_y = height*CROP_P;    
    }
    std::string filename = std::string(shmout);
    if (std::filesystem::exists(filename)) {
        if (std::filesystem::remove(filename)) {
            std::cout << "Pipe deleted successfully: " << filename << std::endl;
        } 
    else {
        std::cerr << "Pipe in use?" << filename << std::endl;
        }
    } 
    HttpServer httpserver(port);
    JSONServer s(httpserver,JSONRPC_SERVER_V2);
    s.StartListening();
    std::string cvpipeline = "v4l2src device=" + std::string(device) + " io-mode=4 ! "
                             "image/jpeg,width="+ std::to_string(width) +",height="+ std::to_string(height) +",framerate="+ std::to_string(fps) +"/1 ! "
                             "queue ! mppjpegdec format=16 ! video/x-raw,format=BGR ! tee name=t t. ! "
                             "queue ! appsink max-buffers=2 drop=true sync=false t. ! "
                             "queue ! mpph264enc bps=20000000 gop="+std::to_string(fps*10)+" rc-mode=1 ! h264parse config-interval=-1 ! mpegtsmux ! filesink location="+std::string(fout)+" t. ! "
                             "queue ! shmsink socket-path=" + std::string(shmout) + " wait-for-connection=false" ;


    cap.open(cvpipeline.c_str(),CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera with GStreamer pipeline." << std::endl;
        return -1;
    }
    
    // GStreamer pipeline elements
    GstElement *pipeline, *videoconvert, *queue0, *queue1, *x264enc,  *tee, *queue_h265_0, *queue_h264, *tsmux, *filesink, *h264parse,  *queue_h265_1;
    GstElement *tee_hr, *tee_lr;
    GstElement *queue_hr_0, *queue_hr_1, *queue_lr_0, *queue_lr_1;

    GstCaps *caps;
    // Create GStreamer pipeline
    pipeline = gst_pipeline_new("video-pipeline"); 
    queue0 = gst_element_factory_make("queue", "queue0");
    queue1 = gst_element_factory_make("queue", "queue1");
    appsrc = gst_element_factory_make("appsrc", "source");
    selector0 = gst_element_factory_make("input-selector", "selector0");
    selector1 = gst_element_factory_make("input-selector", "selector1");
    
    videocrop = gst_element_factory_make("videocrop", "videocrop");
    g_object_set(videocrop, "top", crop_y,
                            "bottom", crop_y,
                            "left", crop_x,
                            "right", crop_x,
                            NULL);

    x265enc0 = gst_element_factory_make("mpph265enc", "x265-encoder0");
    g_object_set(x265enc0, "bps", bitrate0,
                          "header-mode", 1, 
                          "max-pending", 1,
                          "rc-mode", 1,
                          "gop", fps,
                          "width", width_t0,
                          "height", height_t0,
                          NULL);

    x265enc1 = gst_element_factory_make("mpph265enc", "x265-encoder1");
    g_object_set(x265enc1, "bps", bitrate1,
                           "header-mode", 1, 
                           "max-pending", 1,
                           "rc-mode", 1,
                           "gop", fps,
                           "width", width_t1,
                           "height", height_t1,
                           NULL);

    x264enc = gst_element_factory_make("mpph264enc", "x264-encoder");
    g_object_set(x264enc, "bps", 20000000,
                          "bps-min", 10000000,
                          "bps-max", 30000000,
                          "header-mode", 1, 
                          "max-pending", 16,
                          "max-reenc", 3,
                          "gop", fps*10,
                          "rc-mode", 0,
                          "width", width,
                          "height",height,
                          NULL);
    tsmux = gst_element_factory_make("mpegtsmux", "tsmux");

    rtph265pay0 = gst_element_factory_make("rtph265pay", "h265-payloader0");
    g_object_set(rtph265pay0, "config-interval", -1,
                              "aggregate-mode", 1,
                              "mtu", mtu0,
                              NULL);
    rtph265pay1 = gst_element_factory_make("rtph265pay", "h265-payloader1");
    g_object_set(rtph265pay1, "config-interval", -1,
                              "aggregate-mode", 1,
                              "mtu", mtu1,
                              NULL);
    filesink = gst_element_factory_make("filesink", "file-sink");
    g_object_set(G_OBJECT(filesink), "location", fout, "sync", FALSE, "async", FALSE, NULL);

    udpsink0 = gst_element_factory_make("multiudpsink", "udp-sink_0");
    g_object_set(G_OBJECT(udpsink0), "clients", clients0.c_str(), "sync", FALSE, "async", FALSE, NULL);

    udpsink1 = gst_element_factory_make("multiudpsink", "udp-sink_1");
    g_object_set(G_OBJECT(udpsink1), "clients", clients1.c_str(), "sync", FALSE, "async", FALSE, NULL);

    // Configure tee elements and queues
    tee = gst_element_factory_make("tee", "tee");
    tee_hr = gst_element_factory_make("tee", "tee_hr");
    tee_lr = gst_element_factory_make("tee", "tee_lr");
    queue_h264 = gst_element_factory_make("queue", "queue_h264");
    queue_h265_0 = gst_element_factory_make("queue", "queue_h265_0");
    queue_h265_1 = gst_element_factory_make("queue", "queue_h265_1");
   
    queue_hr_0 = gst_element_factory_make("queue", "queue_hr_0");
    queue_hr_1 = gst_element_factory_make("queue", "queue_hr_1");
    queue_lr_0 = gst_element_factory_make("queue", "queue_lr_0");
    queue_lr_1 = gst_element_factory_make("queue", "queue_lr_1");
 
    // Configure the appsrc element
    g_object_set(G_OBJECT(appsrc), "caps",
                 gst_caps_new_simple("video/x-raw",
                                     "format", G_TYPE_STRING, "BGR",
                                     "width", G_TYPE_INT, width,
                                     "height", G_TYPE_INT, height,
                                     "framerate", GST_TYPE_FRACTION, fps, 1,
                                     NULL), NULL);

    g_object_set(G_OBJECT(appsrc), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(appsrc), "is-live", TRUE, NULL);

    // Set the UDP sink properties

    // Build the pipeline
    gst_bin_add_many(GST_BIN(pipeline), appsrc, queue0, videocrop, tee, 
                                        queue_h265_0, x265enc0, rtph265pay0, tee_hr, 
                                        queue_h265_1, x265enc1, rtph265pay1, tee_lr,
                                        //queue_h264, x264enc, tsmux, filesink,
					queue_hr_0, selector0, udpsink0,
					queue_lr_0, selector0,
					queue_hr_1, selector1, udpsink1,
					queue_lr_1, selector1,
                                        NULL);
    if (!gst_element_link_many(appsrc, queue0, videocrop, tee, NULL) || 
        !gst_element_link_many(queue_h265_0, x265enc0, rtph265pay0, tee_hr, NULL) ||    
        !gst_element_link_many(queue_h265_1, x265enc1, rtph265pay1, tee_lr, NULL) ||    
        //!gst_element_link_many(queue_h264, x264enc, tsmux, filesink, NULL) ||    
        !gst_element_link_many(queue_hr_0, selector0, udpsink0, NULL) ||    
        !gst_element_link_many(queue_lr_0, selector0, NULL) ||    
        !gst_element_link_many(queue_hr_1, selector1, udpsink1, NULL) ||    
        !gst_element_link_many(queue_lr_1, selector1, NULL) ||    
	!gst_element_link_many(tee, queue_h265_0, NULL) || 
        !gst_element_link_many(tee, queue_h265_1, NULL) || 
        //!gst_element_link_many(tee, queue_h264, NULL)  ||
	!gst_element_link_many(tee_hr, queue_hr_0, NULL) ||
	!gst_element_link_many(tee_hr, queue_hr_1, NULL) ||
	!gst_element_link_many(tee_lr, queue_lr_0, NULL) ||
	!gst_element_link_many(tee_lr, queue_lr_1, NULL)

       ) 
    {
        g_printerr("GStreamer elements could not be linked.\n");
        gst_object_unref(pipeline);
        return -1;
    }
    sel0_s0 = gst_element_get_static_pad(selector0, "sink_0");
    sel0_s1 = gst_element_get_static_pad(selector0, "sink_1");
    sel1_s0 = gst_element_get_static_pad(selector1, "sink_0");
    sel1_s1 = gst_element_get_static_pad(selector1, "sink_1");

    g_object_set(selector0, "active-pad", sel0_s0, NULL);
    g_object_set(selector1, "active-pad", sel1_s1, NULL);


    // Set the pipeline to the playing state
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // Run the main loop
    GMainLoop *main_loop = g_main_loop_new(NULL, FALSE);
    frames = std::thread(&process_frame);
    frames.detach();
    g_main_loop_run(main_loop);

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(main_loop);
    cap.release();
    s.StopListening();
    return 0;
}
