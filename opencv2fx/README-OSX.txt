To compile and install on OSX:
- Install OpenCV first (using MacPorts or Homebrew).
   We call OPENCV_PATH the path where it was installed (e.g. /opt/local for MacPorts)
- Compile and install in the OFX Plugins directory:
  xcodebuild -configuration Release OPENCV_PATH=/opt/local install
  sudo mv /tmp/opencv2fx.dst/Library/OFX/Plugins/opencv2fx /Library/OFX/Plugins
