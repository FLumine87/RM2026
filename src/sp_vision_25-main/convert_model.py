import os
import shutil
from ultralytics import YOLO

# 模型路径
pt_model_path = r"c:\桌面资料\tmp资料\door_dataset\runs\detect\train-2\weights\best.pt"
output_dir = r"c:\ETO\CBP\cpp\RM\RM2025\src\sp_vision_25-main\assets"

# 主函数
def main():
    print("开始模型转换流程...")
    
    try:
        # 加载模型
        print("=== 步骤1: 加载YOLO模型 ===")
        model = YOLO(pt_model_path)
        print("模型加载成功!")
        
        # 导出为OpenVINO格式
        print("\n=== 步骤2: 导出为OpenVINO格式 ===")
        export_result = model.export(format='openvino', imgsz=416)
        print(f"导出结果: {export_result}")
        
        # 找到导出的文件
        export_dir = os.path.dirname(pt_model_path)
        openvino_dir = os.path.join(export_dir, "best_openvino_model")
        
        if not os.path.exists(openvino_dir):
            print(f"导出目录不存在: {openvino_dir}")
            return
        
        openvino_files = []
        for file in os.listdir(openvino_dir):
            if file.endswith('.xml') or file.endswith('.bin'):
                openvino_files.append(os.path.join(openvino_dir, file))
        
        if not openvino_files:
            print("未找到导出的OpenVINO文件")
            return
        
        # 复制到assets目录
        print("\n=== 步骤3: 复制到assets目录 ===")
        for file in openvino_files:
            dest_file = os.path.join(output_dir, "yolov8_door" + os.path.splitext(file)[1])
            shutil.copy2(file, dest_file)
            print(f"复制 {os.path.basename(file)} 到 {dest_file}")
        
        print("\n=== 转换完成 ===")
        print("模型已成功转换为OpenVINO格式，并保存到assets目录")
        print("请在配置文件中设置:")
        print("yolo_name: yolov8")
        print("yolov8_model_path: assets/yolov8_door.xml")
        
    except Exception as e:
        print(f"转换过程出错: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()