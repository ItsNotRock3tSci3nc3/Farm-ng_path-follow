import cv2

try:
    count = cv2.cuda.getCudaEnabledDeviceCount()
    print("CUDA device count in OpenCV:", count)
    if count > 0:
        print("OpenCV GPU available!")
    else:
        print("OpenCV built without GPU support.")
except:
    print("cv2.cuda not available.")
