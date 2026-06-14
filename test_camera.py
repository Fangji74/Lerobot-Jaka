import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("错误：无法打开摄像头")
    exit()

# 读取一帧
ret, frame = cap.read()

if ret:
    # 直接保存，不显示窗口
    cv2.imwrite('snapshot.jpg', frame)
    print("照片已保存为 snapshot.jpg")
    print(f"照片尺寸: {frame.shape[1]} x {frame.shape[0]} 像素")
else:
    print("无法获取画面")

cap.release()