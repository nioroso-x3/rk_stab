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
#include "stabilization.h"
#define CROP_P 0.05
using namespace cv;

GstElement *gltransformation0;
GstElement *gltransformation1;
GstElement *appsrc;
GstElement *videocrop;
GstElement *x265enc
stabilizer st;
VideoCapture cap;
std::thread frames;
int width = 1920;
int height = 1080;
int t_width = 1280;
int t_height = 720;
int fps = 30;
int crop_x = width*CROP_P;
int crop_y = height*CROP_P;
bool stab = true;
void signalHandler(int signal){

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
        output[y * width + x] = (uchar4)(0, 0, 0, 0); // Transparent background
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
    while(true){
        static GstClockTime timestamp = 0;    
        cap >> frame;
	if (stab){
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
           auto t1 = std::chrono::high_resolution_clock::now();
           cvtColor(prevFrame,prevFrame,COLOR_BGR2BGRA);    	
           cl::Buffer inputBuffer(context, CL_MEM_READ_WRITE, frameSize*sizeof(unsigned char));
           cl::Buffer outputBuffer(context, CL_MEM_READ_WRITE, frameSize*sizeof(unsigned char));
           // Set kernel arguments
           rotateKernel.setArg(0, inputBuffer);
           rotateKernel.setArg(1, outputBuffer);
           rotateKernel.setArg(2, width);
           rotateKernel.setArg(3, height);
           rotateKernel.setArg(4, std::cos(a));
           rotateKernel.setArg(5, std::sin(a));

           // Run the kernel
	   queue.enqueueWriteBuffer(inputBuffer, CL_TRUE, 0, frameSize*sizeof(unsigned char), (unsigned char*)prevFrame.datastart); 
           queue.enqueueNDRangeKernel(rotateKernel, cl::NullRange, globalSize, cl::NullRange);
           queue.enqueueReadBuffer(outputBuffer, CL_TRUE, 0, frameSize*sizeof(unsigned char), (unsigned char*)output.datastart);
	   cvtColor(output,prevFrame,COLOR_BGRA2BGR);    	
           auto t2 = std::chrono::high_resolution_clock::now();

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
	if (frame_count % 120 == 0) std::cout << "rot ms: " << std::chrone::curation_cast<std::chrone::milliseconds>(t2-t1) << "\n";
	frame_count++;
    }
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);
    //std::signal(SIGINT, signalHandler);
    if (argc < 5) {
         std::cout << "Arguments are v4l2 device, multiudpsink clients and ts filename.\nWidth, height, transmission width, transmission height, framerate, bitrate in kbps and stabilization are optional, default 1280x720@30 2000 kbps.\n";
	 return -1;
    }
    const char* device = argv[1];
    const char* clients = argv[2];
    const char* fout = argv[3];
    const char* shmout = argv[4];
    int bitrate = 2000*1000;
    if (argc == 12){
        width = std::stoi(argv[5]);
      	height = std::stoi(argv[6]);
        t_width = std::stoi(argv[7]);
      	t_height = std::stoi(argv[8]);
        fps = std::stoi(argv[9]);
        bitrate = std::stoi(argv[10])*1000;
	stab = (bool)std::stoi(argv[11]);
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


    std::string cvpipeline = "v4l2src device=" + std::string(device) + " io-mode=4 ! "
                             "image/jpeg,width="+ std::to_string(width) +",height="+ std::to_string(height) +",framerate="+ std::to_string(fps) +"/1 ! "
                             "queue ! mppjpegdec format=16 ! video/x-raw,format=BGR ! tee name=t t. ! "
                             "queue ! appsink max-buffers=2 drop=false sync=false t. ! "
			     "queue ! shmsink socket-path=" + std::string(shmout) + " wait-for-connection=false" ;


    cap.open(cvpipeline.c_str(),CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera with GStreamer pipeline." << std::endl;
        return -1;
    }
    
    // GStreamer pipeline elements
    GstElement *pipeline, *videoconvert, *queue0, *glupload, *glcolorconvert0, *gltransform0, *gltransform1, *glcolorconvert1, *gldownload, *enccaps, *queue1, *x264enc, *rtph265pay, *udpsink, *tee, *queue_h265, *queue_h264, *tsmux, *filesink, *h264parse;
    GstCaps *caps;
    // Create GStreamer pipeline
    pipeline = gst_pipeline_new("video-pipeline"); 
    queue0 = gst_element_factory_make("queue", "queue0");
    queue1 = gst_element_factory_make("queue", "queue1");
    appsrc = gst_element_factory_make("appsrc", "source");
    videoconvert = gst_element_factory_make("videoconvert", "videoconvert");
    /*
    glcolorconvert0 = gst_element_factory_make("glcolorconvert", "glcolorconvert0");
    glupload = gst_element_factory_make("glupload", "glupload");
    gltransformation0 = gst_element_factory_make("gltransformation", "gltransformation0");
    gltransformation1 = gst_element_factory_make("gltransformation", "gltransformation1");
    glcolorconvert1 = gst_element_factory_make("glcolorconvert", "glcolorconvert1");
    gldownload = gst_element_factory_make("gldownload", "gldownload");
    */
    enccaps = gst_element_factory_make("capsfilter", "enccaps");
    caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "I420",
        NULL);
    g_object_set(enccaps, "caps", caps, NULL);
    gst_caps_unref(caps);
    videocrop = gst_element_factory_make("videocrop", "videocrop");
    g_object_set(videocrop, "top", crop_y,
                            "bottom", crop_y,
                            "left", crop_x,
                            "right", crop_x,
                            NULL);

    x265enc = gst_element_factory_make("mpph265enc", "x265-encoder");
    g_object_set(x265enc, "bps", bitrate,
                          "header-mode", 1, 
                          "max-pending", 1,
                          "rc-mode", 1,
                          "gop", fps,
			  "width", t_width,
			  "height", t_height,
			  NULL);

    x264enc = gst_element_factory_make("mpph264enc", "x264-encoder");
    g_object_set(x264enc, "bps", 10000000,
		          "bps-min", 5000000,
			  "bps-max", 15000000,
                          "header-mode", 1, 
			  "max-pending", 16,
			  "gop", fps*10,
			  "rc-mode", 0,
			  NULL);
    tsmux = gst_element_factory_make("mpegtsmux", "tsmux");

    rtph265pay = gst_element_factory_make("rtph265pay", "h265-payloader");
    g_object_set(rtph265pay, "config-interval", -1,
		                         "aggregate-mode", 1,
			                       "mtu", 896,
			                        NULL);
    filesink = gst_element_factory_make("filesink", "file-sink");
    g_object_set(G_OBJECT(filesink), "location", fout, "sync", FALSE, "async", FALSE, NULL);

    udpsink = gst_element_factory_make("multiudpsink", "udp-sink");
    g_object_set(G_OBJECT(udpsink), "clients", clients, "sync", FALSE, "async", FALSE, NULL);

    // Configure tee elements and queues
    tee = gst_element_factory_make("tee", "tee");
    queue_h264 = gst_element_factory_make("queue", "queue_h264");
    queue_h265 = gst_element_factory_make("queue", "queue_h265");

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
		                        queue_h265, x265enc, rtph265pay, udpsink, 
					queue_h264, x264enc, tsmux, filesink,
					NULL);
    if (!gst_element_link_many(appsrc, queue0, videocrop, tee, NULL) || 
	!gst_element_link_many(queue_h265, x265enc, rtph265pay, udpsink, NULL) ||    
	!gst_element_link_many(queue_h264, x264enc, tsmux, filesink, NULL) ||    
        !gst_element_link_many(tee, queue_h265, NULL) || 
        !gst_element_link_many(tee, queue_h264, NULL)
	) {
        g_printerr("GStreamer elements could not be linked.\n");
        gst_object_unref(pipeline);
        return -1;
    }


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
    return 0;
}
