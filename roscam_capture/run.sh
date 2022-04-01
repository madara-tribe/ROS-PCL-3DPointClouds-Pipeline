# run usb cam
roslaunch roscam_capture usb_cam.launch 

# compressed usbcam image and preprocess
roslaunch roscam_capture usb_cam.launch
roslaunch roscam_capture image_republish.launch use_compressed:=true topic:=~
python3 img_republish.py

