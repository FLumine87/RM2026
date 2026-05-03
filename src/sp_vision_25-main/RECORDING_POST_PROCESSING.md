# 录制视频赛后处理指南（Windows系统）

## 目录
1. [环境准备](#环境准备)
2. [文件结构说明](#文件结构说明)
3. [解压文件](#解压文件)
4. [视频拼接](#视频拼接)
5. [插帧处理](#插帧处理)
6. [格式转换（MP4/AVI）](#格式转换)
7. [损坏文件处理](#损坏文件处理)
8. [部分块未压缩处理](#部分块未压缩处理)
9. [完整处理流程示例](#完整处理流程示例)
10. [常见问题解决](#常见问题解决)

---

## 1. 环境准备

### 1.1 安装必要工具

#### FFmpeg（核心工具）
- **下载地址**：https://ffmpeg.org/download.html#build-windows
- **选择版本**：下载 `full_build` 的 Windows 版本
- **配置环境变量**：将 `ffmpeg.exe` 所在目录添加到系统 PATH

#### 7-Zip（解压工具）
- **下载地址**：https://www.7-zip.org/
- **安装路径**：建议安装到 `C:\Program Files\7-Zip\`

### 1.2 验证安装

打开命令提示符（CMD）或 PowerShell，执行以下命令验证：

```bash
ffmpeg -version
7z
```

如果显示版本信息，说明安装成功。

---

## 2. 文件结构说明

录制完成后，`records` 目录下的文件结构如下：

```
records/
├── 2024-01-15_10-00-00.avi      # 第一个2分钟块（未压缩）
├── 2024-01-15_10-00-00.txt      # 对应的陀螺仪数据
├── 2024-01-15_10-02-00.7z       # 第二个2分钟块（已压缩）
├── 2024-01-15_10-04-00.7z       # 第三个2分钟块（已压缩）
└── ...
```

- **.7z 文件**：已压缩的视频+数据块
- **.avi 文件**：未压缩的视频文件
- **.txt 文件**：陀螺仪四元数数据（格式：`时间戳 w x y z`）

---

## 3. 解压文件

### 3.1 单个文件解压

```bash
# 解压单个块
7z x "2024-01-15_10-02-00.7z" -o"output_folder"
```

### 3.2 批量解压所有块

创建 `extract_all.bat` 脚本：

```batch
@echo off
setlocal enabledelayedexpansion

:: 创建输出目录
mkdir extracted

:: 批量解压所有7z文件
for %%f in (*.7z) do (
    echo 正在解压: %%f
    7z x "%%f" -o"extracted\" -y
)

echo 解压完成！
pause
```

### 3.3 解压到指定目录

```bash
# 将所有块解压到统一目录
7z x *.7z -o"all_records" -y
```

---

## 4. 视频拼接

### 4.1 创建文件列表

首先创建 `file_list.txt`，按时间顺序列出所有视频文件：

```
file '2024-01-15_10-00-00.avi'
file '2024-01-15_10-02-00.avi'
file '2024-01-15_10-04-00.avi'
file '2024-01-15_10-06-00.avi'
```

**自动生成文件列表脚本**（`create_list.bat`）：

```batch
@echo off
dir /b /od *.avi > temp.txt
for /f %%f in (temp.txt) do (
    echo file '%%f' >> file_list.txt
)
del temp.txt
echo 文件列表已生成
```

### 4.2 使用 FFmpeg 拼接

```bash
# 无损拼接（推荐）
ffmpeg -f concat -safe 0 -i file_list.txt -c copy output_concat.avi

# 重新编码拼接（确保兼容性）
ffmpeg -f concat -safe 0 -i file_list.txt -c:v libx264 -crf 18 -c:a copy output_encoded.avi
```

### 4.3 拼接时同步数据文件

手动合并所有 `.txt` 文件：

```bash
# 按时间顺序合并txt文件
copy /b 2024-01-15_10-00-00.txt + 2024-01-15_10-02-00.txt + 2024-01-15_10-04-00.txt all_data.txt
```

---

## 5. 插帧处理

### 5.1 插帧到30fps（推荐）

```bash
# 使用光流法插帧（质量较好）
ffmpeg -i input.avi -r 30 -filter:v "minterpolate='fps=30:mi_mode=mci'" output_30fps.avi

# 简单复制插帧（速度快）
ffmpeg -i input.avi -r 30 output_30fps_fast.avi

# GPU加速插帧（如果有NVIDIA显卡）
ffmpeg -i input.avi -r 30 -filter:v "minterpolate='fps=30:mi_mode=dup'" -c:v h264_nvenc output_30fps_gpu.avi
```

### 5.2 插帧参数说明

| 参数 | 说明 |
|------|------|
| `-r 30` | 输出帧率为30fps |
| `mi_mode=mci` | 中值光流插值（推荐） |
| `mi_mode=dup` | 帧复制模式（最快） |
| `mi_mode=blend` | 帧混合模式（平滑） |

---

## 6. 格式转换

### 6.1 转换为MP4（H.264编码）

```bash
# 高质量MP4
ffmpeg -i input.avi -c:v libx264 -crf 18 -preset slow -c:a aac -b:a 192k output.mp4

# 平衡质量与文件大小
ffmpeg -i input.avi -c:v libx264 -crf 23 -preset medium output.mp4

# 小文件（质量较低）
ffmpeg -i input.avi -c:v libx264 -crf 28 -preset fast output_small.mp4
```

### 6.2 转换为AVI（MJPG编码，兼容测试代码）

```bash
# MJPG编码（无损质量）
ffmpeg -i input.mp4 -c:v mjpeg -q:v 1 output.avi

# H.264编码的AVI
ffmpeg -i input.mp4 -c:v libx264 -crf 18 output_h264.avi
```

### 6.3 批量格式转换

```batch
@echo off
for %%f in (*.avi) do (
    ffmpeg -i "%%f" -c:v libx264 -crf 23 "%%~nf.mp4"
)
```

---

## 7. 损坏文件处理

### 7.1 修复损坏的AVI文件

```bash
# 使用FFmpeg修复损坏的视频
ffmpeg -i broken.avi -c:v copy -c:a copy -fflags +genpts fixed.avi

# 重新编码修复
ffmpeg -i broken.avi -c:v libx264 -crf 23 fixed.mp4
```

### 7.2 跳过损坏帧

```bash
# 尝试解码所有帧，跳过损坏的帧
ffmpeg -i broken.avi -ignore_loop 0 -max_error_rate 0.1 -c:v libx264 output.mp4
```

### 7.3 提取可用部分

```bash
# 提取视频的前10分钟
ffmpeg -i input.avi -t 00:10:00 -c copy first_10min.avi

# 从第5分钟开始提取
ffmpeg -i input.avi -ss 00:05:00 -c copy from_5min.avi
```

---

## 8. 部分块未压缩处理

### 8.1 识别未压缩的块

```bash
# 列出所有未压缩的avi文件
dir *.avi /b

# 列出所有已压缩的7z文件
dir *.7z /b
```

### 8.2 手动压缩未压缩的块

```bash
# 压缩单个块
7z a -mx=9 "2024-01-15_10-00-00.7z" "2024-01-15_10-00-00.avi" "2024-01-15_10-00-00.txt"

# 批量压缩所有未压缩的块
for %%f in (*.avi) do (
    set "name=%%~nf"
    7z a -mx=9 "!name!.7z" "!name!.avi" "!name!.txt"
)
```

### 8.3 混合处理（部分压缩+部分未压缩）

```batch
@echo off
:: 先解压所有压缩文件
7z x *.7z -o"all_files" -y

:: 复制未压缩的文件到同一目录
copy *.avi all_files\
copy *.txt all_files\

:: 切换到目录处理
cd all_files

:: 创建文件列表
dir /b /od *.avi > temp.txt
for /f %%f in (temp.txt) do (
    echo file '%%f' >> file_list.txt
)
del temp.txt

:: 拼接视频
ffmpeg -f concat -safe 0 -i file_list.txt -c copy output.avi

:: 合并数据文件
copy /b *.txt all_data.txt

echo 处理完成！
```

---

## 9. 完整处理流程示例

### 9.1 流程概览

```
1. 解压所有块 → 2. 合并数据 → 3. 拼接视频 → 4. 插帧 → 5. 格式转换
```

### 9.2 脚本1：自动解压所有文件（`01_extract.bat`）

用于解压所有压缩文件，并将未压缩的文件统一收集到 `extracted` 目录。

```batch
@echo off
setlocal enabledelayedexpansion

echo ==============================================
echo          脚本1：自动解压所有文件
echo ==============================================

:: 创建输出目录
mkdir extracted 2>NUL

:: 解压所有7z文件
echo [1/2] 正在解压压缩文件...
for %%f in (*.7z) do (
    echo 解压: %%f
    7z x "%%f" -o"extracted\" -y
)

:: 复制未压缩的avi文件
echo [2/2] 正在复制未压缩文件...
copy *.avi extracted\ 2>NUL
copy *.txt extracted\ 2>NUL

echo ==============================================
echo             解压完成！
echo 输出目录：extracted\
echo ==============================================

pause
```

### 9.3 脚本2：视频拼接与插帧（`02_concat_interpolate.bat`）

用于根据文件列表拼接视频，并进行插帧处理。需要先手动编辑 `file_list.txt` 选择要处理的文件。

```batch
@echo off
setlocal enabledelayedexpansion

:: 配置参数
set "FPS=30"
set "CRF=23"
set "OUTPUT_NAME=output_interpolated"

echo ==============================================
echo          脚本2：视频拼接与插帧
echo ==============================================

:: 检查文件列表是否存在
if not exist file_list.txt (
    echo 错误：未找到 file_list.txt 文件！
    echo 请先创建 file_list.txt，格式如下：
    echo file 'video1.avi'
    echo file 'video2.avi'
    pause
    exit /b
)

:: 1. 拼接视频
echo [1/2] 正在拼接视频...
ffmpeg -f concat -safe 0 -i file_list.txt -c copy temp_concat.avi

:: 2. 插帧处理
echo [2/2] 正在插帧到 %FPS% fps...
ffmpeg -i temp_concat.avi -r %FPS% -filter:v "minterpolate='fps=%FPS%'" -c:v libx264 -crf %CRF% "%OUTPUT_NAME%.avi"

:: 清理临时文件
del temp_concat.avi

echo ==============================================
echo             处理完成！
echo 输出文件：%OUTPUT_NAME%.avi
echo ==============================================

pause
```

**使用方法**：
1. 在 `extracted` 目录下创建 `file_list.txt`，内容示例：
   ```
   file '2024-01-15_10-00-00.avi'
   file '2024-01-15_10-02-00.avi'
   file '2024-01-15_10-04-00.avi'
   ```
2. 运行脚本，自动拼接并插帧

### 9.4 脚本3：AVI转MP4（`03_convert_to_mp4.bat`）

用于将AVI文件批量转换为MP4格式。

```batch
@echo off
setlocal enabledelayedexpansion

:: 配置参数
set "CRF=23"

echo ==============================================
echo          脚本3：AVI转MP4批量转换
echo ==============================================

:: 创建输出目录
mkdir mp4_output 2>NUL

:: 批量转换所有AVI文件
set "count=0"
for %%f in (*.avi) do (
    set /a count+=1
    echo [%%count%] 正在转换: %%f
    ffmpeg -i "%%f" -c:v libx264 -crf %CRF% -c:a aac "mp4_output\%%~nf.mp4" -y
)

echo ==============================================
echo             转换完成！
echo 转换文件数：%count%
echo 输出目录：mp4_output\
echo ==============================================

pause
```

### 9.5 文件列表生成器（`00_generate_list.bat`）

用于自动生成按时间排序的文件列表，方便手动选择。

```batch
@echo off
setlocal enabledelayedexpansion

echo ==============================================
echo          文件列表生成器
echo ==============================================

:: 生成按时间排序的文件列表
dir /b /od *.avi > temp_list.txt

echo 已生成文件列表：
echo -------------------
type temp_list.txt
echo -------------------

echo.
echo 请根据需要编辑 file_list.txt，格式示例：
echo file '2024-01-15_10-00-00.avi'
echo file '2024-01-15_10-02-00.avi'

:: 创建模板文件
echo file 'your_video.avi' > file_list.txt
echo file 'another_video.avi' >> file_list.txt

echo.
echo 已创建 file_list.txt 模板，请手动编辑选择要处理的文件

pause
```

### 9.6 使用流程示例

```
1. 运行 01_extract.bat → 解压所有文件到 extracted\
2. 进入 extracted\ 目录
3. 运行 00_generate_list.bat → 查看可用文件
4. 编辑 file_list.txt → 选择要拼接的视频
5. 运行 02_concat_interpolate.bat → 拼接并插帧
6. 运行 03_convert_to_mp4.bat → 转换为MP4（可选）
```

---

## 10. 常见问题解决

### 10.1 FFmpeg 命令找不到

**问题**：执行 `ffmpeg` 时提示"不是内部或外部命令"

**解决方案**：
1. 确认 FFmpeg 已正确安装
2. 将 FFmpeg 目录添加到系统环境变量 PATH
3. 重启命令提示符

### 10.2 视频拼接失败

**问题**：`Invalid data found when processing input`

**解决方案**：
1. 检查 `file_list.txt` 中的文件路径是否正确
2. 使用 `-safe 0` 参数
3. 确保所有视频文件格式一致

### 10.3 插帧速度慢

**问题**：插帧处理时间过长

**解决方案**：
1. 使用 `-preset fast` 参数
2. 如果有NVIDIA显卡，使用GPU加速：`-c:v h264_nvenc`
3. 降低输出分辨率：`-vf scale=1280:720`

### 10.4 陀螺仪数据与视频不同步

**问题**：测试时发现数据与视频不匹配

**解决方案**：
1. 确保合并txt文件时顺序正确
2. 检查是否有缺失的块
3. 使用时间戳进行对齐

### 10.5 文件损坏无法解压

**问题**：7z解压时提示"数据错误"

**解决方案**：
1. 尝试使用 `7z t` 命令测试文件完整性
2. 如果是最后一个块损坏，可能是断电导致，可跳过该块
3. 使用 `7z x -y` 强制解压可恢复的部分

---

## 11. 处理脚本集合

### 11.1 脚本1：自动解压所有文件（`01_extract.bat`）

用于解压所有压缩文件，并将未压缩的文件统一收集到 `extracted` 目录。

```batch
@echo off
setlocal enabledelayedexpansion

echo ==============================================
echo          脚本1：自动解压所有文件
echo ==============================================

:: 创建输出目录
mkdir extracted 2>NUL

:: 解压所有7z文件
echo [1/2] 正在解压压缩文件...
for %%f in (*.7z) do (
    echo 解压: %%f
    7z x "%%f" -o"extracted\" -y
)

:: 复制未压缩的avi文件
echo [2/2] 正在复制未压缩文件...
copy *.avi extracted\ 2>NUL
copy *.txt extracted\ 2>NUL

echo ==============================================
echo             解压完成！
echo 输出目录：extracted\
echo ==============================================

pause
```

### 11.2 脚本2：文件列表生成器（`00_generate_list.bat`）

用于自动生成按时间排序的文件列表，方便手动选择。

```batch
@echo off
setlocal enabledelayedexpansion

echo ==============================================
echo          文件列表生成器
echo ==============================================

:: 生成按时间排序的文件列表
dir /b /od *.avi > temp_list.txt

echo 已生成文件列表：
echo -------------------
type temp_list.txt
echo -------------------

echo.
echo 请根据需要编辑 file_list.txt，格式示例：
echo file '2024-01-15_10-00-00.avi'
echo file '2024-01-15_10-02-00.avi'

:: 创建模板文件
echo file 'your_video.avi' > file_list.txt
echo file 'another_video.avi' >> file_list.txt

echo.
echo 已创建 file_list.txt 模板，请手动编辑选择要处理的文件

pause
```

### 11.3 脚本3：视频拼接与插帧（`02_concat_interpolate.bat`）

用于根据文件列表拼接视频，并进行插帧处理。需要先手动编辑 `file_list.txt` 选择要处理的文件。

```batch
@echo off
setlocal enabledelayedexpansion

:: 配置参数
set "FPS=30"
set "CRF=23"
set "OUTPUT_NAME=output_interpolated"

echo ==============================================
echo          脚本3：视频拼接与插帧
echo ==============================================

:: 检查文件列表是否存在
if not exist file_list.txt (
    echo 错误：未找到 file_list.txt 文件！
    echo 请先创建 file_list.txt，格式如下：
    echo file 'video1.avi'
    echo file 'video2.avi'
    pause
    exit /b
)

:: 1. 拼接视频
echo [1/2] 正在拼接视频...
ffmpeg -f concat -safe 0 -i file_list.txt -c copy temp_concat.avi

:: 2. 插帧处理
echo [2/2] 正在插帧到 %FPS% fps...
ffmpeg -i temp_concat.avi -r %FPS% -filter:v "minterpolate='fps=%FPS%'" -c:v libx264 -crf %CRF% "%OUTPUT_NAME%.avi"

:: 清理临时文件
del temp_concat.avi

echo ==============================================
echo             处理完成！
echo 输出文件：%OUTPUT_NAME%.avi
echo ==============================================

pause
```

### 11.4 脚本4：AVI转MP4（`03_convert_to_mp4.bat`）

用于将AVI文件批量转换为MP4格式。

```batch
@echo off
setlocal enabledelayedexpansion

:: 配置参数
set "CRF=23"

echo ==============================================
echo          脚本4：AVI转MP4批量转换
echo ==============================================

:: 创建输出目录
mkdir mp4_output 2>NUL

:: 批量转换所有AVI文件
set "count=0"
for %%f in (*.avi) do (
    set /a count+=1
    echo [%%count%] 正在转换: %%f
    ffmpeg -i "%%f" -c:v libx264 -crf %CRF% -c:a aac "mp4_output\%%~nf.mp4" -y
)

echo ==============================================
echo             转换完成！
echo 转换文件数：%count%
echo 输出目录：mp4_output\
echo ==============================================

pause
```

### 11.5 使用流程示例

```
1. 运行 01_extract.bat → 解压所有文件到 extracted\
2. 进入 extracted\ 目录
3. 运行 00_generate_list.bat → 查看可用文件
4. 编辑 file_list.txt → 选择要拼接的视频
5. 运行 02_concat_interpolate.bat → 拼接并插帧
6. 运行 03_convert_to_mp4.bat → 转换为MP4（可选）
```

---

## 附录：常用命令速查表

| 操作 | 命令 |
|------|------|
| 解压单个文件 | `7z x file.7z` |
| 批量解压 | `7z x *.7z -o"output"` |
| 视频拼接 | `ffmpeg -f concat -i list.txt -c copy out.avi` |
| 插帧到30fps | `ffmpeg -i in.avi -r 30 -filter:v minterpolate out.avi` |
| AVI转MP4 | `ffmpeg -i in.avi -c:v libx264 -crf 23 out.mp4` |
| MP4转AVI | `ffmpeg -i in.mp4 -c:v mjpeg out.avi` |
| 修复损坏视频 | `ffmpeg -i broken.avi -c:v copy fixed.avi` |

---

**文档版本**：v1.0  
**适用系统**：Windows 10/11  
**最后更新**：2024年1月