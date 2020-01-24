# Human pose detection using OpenVino
This project assumes OpenVino is already installed in the system.

# Running the program.
After successfully building the solution, the generated exe needs the following files to run.

### From OpenVino Inference engine.
* plugins.xml
After building and installing OpenVino this is usually stored in <openvino install folder>/deployment_tools/inference_engine/bin/intel64/Release (or Debug depending on the build settings)
* MKLDNNPlugin.dll
In the same location as plugins.xml. This is necessary when running on CPU
* Other plugin dlls as necessary depending on the device it runs on. Check the contents of plugins.xml to understand which dll file is necessary for a target device.

### Required parameters
The application needs two parameters to run
* -m flag to determine the xml file of the trained human pose model.
You can download this file using the utility tool provided at <openvino install folder>/deployment_tools/tools/model_downloader/downloader.py.
* -i flag to determine the target device. The options are CPU, GPU or other device/processors. Refer to Intel documentation.
A sample run command will be as follows:
```
human_pose_detection.exe -m "<path to the xml file directory>\human-pose-detection-0001.xml" -i cpu
```
