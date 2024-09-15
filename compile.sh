#g++ -g inline.cpp stabilization.cpp -o inline `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0 gstreamer-gl-1.0` #-lOpenCL -fpermissive
g++ -O3 inline.cpp stabilization.cpp  -o inline `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` -lOpenCL -ljsoncpp -lmicrohttpd -ljsonrpccpp-common -ljsonrpccpp-server -fpermissive 
#g++ -O3 inline_img2d.cpp stabilization.cpp  -o inline_img `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0` -lOpenCL -ljsoncpp -lmicrohttpd -ljsonrpccpp-common -ljsonrpccpp-server -fpermissive 
#g++ -g parallel.cpp stabilization.cpp -o parallel `pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0 gstreamer-gl-1.0 opencl`
#g++ -o gst_app main.cpp `pkg-config --cflags --libs gstreamer-1.0 gstreamer-gl-1.0`

