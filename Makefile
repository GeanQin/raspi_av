all:
	arm-linux-gnueabihf-gcc src/RaspiVid.c src/RaspiHelpers.c src/RaspiGPS.c src/RaspiCommonSettings.c src/RaspiPreview.c src/RaspiCamControl.c src/libgps_loader.c src/RaspiCLI.c -lmmal_util -lmmal_core -lbcm_host -lvcos -lpthread -ldl -I./include -L./lib
