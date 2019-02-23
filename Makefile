###############################################################################
#																			  #
#	Makefile - Team 2959 2019 Raspberry PI program makefile.				  #
#																			  #
#	by Jim Barringham														  #
#																			  #
#	This file was adapted from the FRC Raspberry Pi C++ example project.	  #
#																			  #
###############################################################################

# Path to our compiler/linker
CXX=Tools/raspbian9/bin/arm-raspbian9-linux-gnueabihf-g++

# Output executable name
EXE=build\rpi2959

# Destination folder...Unused for Windows builds
DESTDIR?=/home/pi/

.PHONY: clean build install

# Required steps for "build" target
build: ${EXE}

# Required steps for "install" target
install: build
	cp ${EXE} runCamera ${DESTDIR}

# Required steps for "clean" target
clean:
	cmd /c del "build\*.o"
	cmd /c del "${EXE}"

# Required steps to build our target executable.  This is the list of object files...
# there should be one .o file for each .cpp file in the project
# below that are all the linker command line parameters, consisting mostly of all
# the possible libraries that we might use
${EXE}: build/Analyzer.o build/Main.o build/Pipeline.o
	${CXX} -pthread -o $@ build/Main.o build/Analyzer.o build/Pipeline.o \
	    -LTools/lib \
	    -lwpilibc \
	    -lwpiHal \
	    -lcameraserver \
	    -lcscore \
	    -lntcore \
	    -lwpiutil \
	    -lopencv_ml \
	    -lopencv_objdetect \
	    -lopencv_shape \
	    -lopencv_stitching \
	    -lopencv_superres \
	    -lopencv_videostab \
	    -lopencv_calib3d \
	    -lopencv_features2d \
	    -lopencv_highgui \
	    -lopencv_videoio \
	    -lopencv_imgcodecs \
	    -lopencv_video \
	    -lopencv_photo \
	    -lopencv_imgproc \
	    -lopencv_flann \
	    -lopencv_core

# The command line to get from Analyzer.cpp to build/Analyzer.o
build/Analyzer.o: Analyzer.cpp
	${CXX} -pthread -O -c -o $@ -ITools/include $<

# The command line to get from Main.cpp to build/Main.o
build/Main.o: Main.cpp
	${CXX} -pthread -O -c -o $@ -ITools/include $<

# The command line to build build/Pipeline.o from Pipeline.cpp
build/Pipeline.o: Pipeline.cpp
	${CXX} -pthread -O -c -o $@ -ITools/include $<
