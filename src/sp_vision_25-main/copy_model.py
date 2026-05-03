import os
import shutil

# 源目录
source_dir = r"c:\桌面资料\tmp资料\door_dataset\runs\detect\train-2\weights\best_openvino_model"
# 目标目录
target_dir = r"c:\ETO\CBP\cpp\RM\RM2025\src\sp_vision_25-main\assets"

# 检查源目录是否存在
if not os.path.exists(source_dir):
    print(f"源目录不存在: {source_dir}")
    exit(1)

# 检查目标目录是否存在
if not os.path.exists(target_dir):
    print(f"目标目录不存在: {target_dir}")
    exit(1)

# 复制文件
files_to_copy = ["best.xml", "best.bin"]

print("开始复制OpenVINO文件...")
for file_name in files_to_copy:
    source_file = os.path.join(source_dir, file_name)
    target_file = os.path.join(target_dir, "yolov8_door" + os.path.splitext(file_name)[1])
    
    if os.path.exists(source_file):
        shutil.copy2(source_file, target_file)
        print(f"复制 {file_name} 到 {target_file}")
    else:
        print(f"文件不存在: {source_file}")

print("\n复制完成!")
print("请在配置文件中设置:")
print("yolo_name: yolov8")
print("yolov8_model_path: assets/yolov8_door.xml")