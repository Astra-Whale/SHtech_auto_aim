ENV Requirements:

opencv 4.1.1 
cuda 10.2
MVSlib  // for hikvision camera
libdarknet.so //from https://github.com/AlexeyAB/darknet


Compile:

mkdir build
cd build 
cmake .. && make 

If you compile on NX, please replace "/opt/MVS/lib/64" with "/opt/MVS/lib/aarch64/", which is the path of hikcam dynamaic library. 
In next version, conditional compile command will be added.


Run:

run cmd "./run.sh" at rootdir of the project


Config:

Executable input parameter is configured at launch.cfg, mainly about input source, log, and display. Some display option is temporarily disabled.