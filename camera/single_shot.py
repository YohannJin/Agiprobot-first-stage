#####################################################################
# created by Yuhan Jin on 27.09.2022
# questions -> lioking95@gmail.com
# camera driver code for Dahang mercury2-usb3.0 series
# pre: install iai-gxipy package with your python interpreter
#####################################################################
import gxipy as gx
import cv2
import datetime
from PIL import Image


def main():
    ####################################################################
    # define image parameter
    ####################################################################
    # image size
    # maximal 1440*1080
    width_set = 1440
    height_set = 1080

    # FPS
    # maximal 227
    fps_set = 1

    # the frame number to be sampled
    max_frame_num = 1

    print("#############################################################")
    print("from camera sample RGB numpy array and show it using opencv")
    print("#############################################################")
    print("Initializing......")

    # open camera
    ####################################################################
    # create a device manager
    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print("Number of enumerated devices is 0")
        return
    else:
        print("-------------------------------------------------------------")
        print("device created with ID: %d" % dev_num)

    # open the first device
    cam = device_manager.open_device_by_index(1)

    # exit when the camera is a mono camera
    if cam.PixelColorFilter.is_implemented() is False:
        print("This code is for RGB camera, however, this is a mono one.")
        cam.close_device()
        return
    else:
        print("-------------------------------------------------------------")
        print("RGB camera successfully opened, the serial number is as follow:", dev_info_list[0].get("sn"))

    # contineous acquisition, revert the setting to use trigger sample
    cam.TriggerMode.set(gx.GxSwitchEntry.OFF)
    cam.AcquisitionFrameRateMode.set(gx.GxSwitchEntry.ON)

    ####################################################################
    # set image parameter
    ####################################################################
    # set required image size
    cam.Width.set(width_set)
    cam.Height.set(height_set)
    # set required fps
    cam.AcquisitionFrameRate.set(fps_set)
    print("-------------------------------------------------------------")
    print("required FPS: %d" % fps_set)

    ####################################################################
    # set exposure time
    ####################################################################
    # range 20 - 100000
    # correspond to 2ms - 1s
    # cam.ExposureTime.set(20000.0)
    # set exposure automatically
    cam.ExposureAuto.set(gx.GxAutoEntry.CONTINUOUS)
    exposure_get = cam.ExposureTime.get()
    print("-------------------------------------------------------------")
    print("initial exposure value: %d" % exposure_get)

    ####################################################################
    # set gain
    ####################################################################
    # 10.0 by default
    # set manually
    # cam.Gain.set(10.0)
    # set automatically
    cam.GainAuto.set(gx.GxAutoEntry.CONTINUOUS)
    gain_get = cam.Gain.get()
    print("-------------------------------------------------------------")
    print("initial gain value: %d" % gain_get)

    ####################################################################
    # set white balance
    ####################################################################
    cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS)

    ####################################################################
    # read parameters for improving image quality
    ####################################################################
    if cam.GammaParam.is_readable():
        gamma_value = cam.GammaParam.get()
        print("-------------------------------------------------------------")
        print("gamma value: %d" % gamma_value)
        print("-------------------------------------------------------------")
        gamma_lut = gx.Utility.get_gamma_lut(gamma_value)
        # print("gamma lut: %d" %gamma_lut)
        # print("-------------------------------------------------------------")
    else:
        gamma_lut = None
    if cam.ContrastParam.is_readable():
        contrast_value = cam.ContrastParam.get()
        print("contrast value: %d" % contrast_value)
        print("-------------------------------------------------------------")
        contrast_lut = gx.Utility.get_contrast_lut(contrast_value)
        # print("contrast lut: %d" % contrast_lut)
        # print("-------------------------------------------------------------")
    else:
        contrast_lut = None
    if cam.ColorCorrectionParam.is_readable():
        color_correction_param = cam.ColorCorrectionParam.get()
        print("color correction parameter: %d" % color_correction_param)
        print("-------------------------------------------------------------")
    else:
        color_correction_param = 0

    ####################################################################
    # start data acquisition
    ####################################################################
    print("starting the camera stream...")
    cam.stream_on()

    for i in range(max_frame_num):
        ####################################################################
        # debugging info
        ####################################################################
        print("-------------------------------------------------------------")
        print("sample Nr. %d" % i)
        fps_get = cam.CurrentAcquisitionFrameRate.get()
        print("FPS: %d" % fps_get)
        gain_get = cam.Gain.get()
        print("gain value: %d" % gain_get)
        exposure_get = cam.ExposureTime.get()
        print("exposure value: %d" % exposure_get)
        # wb_get = cam.BalanceRatio.get()
        # print("balance ratio: %d" % wb_get)

        # get raw image and convert to RGB
        raw_image = cam.data_stream[0].get_image()  # get a single image
        rgb_image = raw_image.convert("RGB")  # convert RAW to RGB

        # image quality improvement
        rgb_image.image_improvement(color_correction_param, contrast_lut, gamma_lut)

        if rgb_image is None:
            continue

        # create numpy array about image
        numpy_image = rgb_image.get_numpy_array()
        numpy_image = cv2.cvtColor(numpy_image, cv2.COLOR_RGB2BGR)  # convert to BGR(for opencv)

        # show image
        cv2.namedWindow('live stream', cv2.WINDOW_AUTOSIZE)  # create a window
        cv2.imshow('live stream', numpy_image)  # output the current frame on the window

        ###############################################
        # save the images
        ###############################################
        current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H_%M_%S')
        img = Image.fromarray(numpy_image, 'RGB')
        img.save("./images/" + str(i) + str("-") + current_time + ".jpg")

        # use esc to exit
        if cv2.waitKey(1) & 0xFF == 27:
            break

    ####################################################################
    # stop sampling
    ####################################################################
    print("-------------------------------------------------------------")
    print("closing the stream...")
    print("-------------------------------------------------------------")
    cam.stream_off()
    cam.close_device()


if __name__ == "__main__":
    main()
