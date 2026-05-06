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
chcp 65001 >nul
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
chcp 65001 >nul
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
# 使用FFmpeg修复损坏的视频（无损复制，保持原格式）
ffmpeg -i broken.avi -c:v copy -c:a copy -fflags +genpts fixed.avi

# 重新编码修复（保持AVI格式，MJPEG编码）
ffmpeg -i broken.avi -c:v mjpeg -q:v 2 -c:a pcm_s16le fixed.avi
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
chcp 65001 >nul
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
chcp 65001 >nul
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
chcp 65001 >nul
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
chcp 65001 >nul
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

:: 使用chcp 65001后再改回936，确保file_list.txt使用ANSI编码
chcp 65001 >nul

echo ==============================================
echo          文件列表生成器
echo ==============================================

:: 删除旧的文件列表
if exist file_list.txt del file_list.txt

:: 切换回ANSI编码生成文件列表，避免UTF-8 BOM问题
:: 使用 /on 按文件名排序（因为文件名已包含时间戳）
chcp 936 >nul
(
    for /f "delims=" %%f in ('dir /b /on *.avi') do (
        echo file '%%f'
    )
) > file_list.txt

:: 切回UTF-8显示中文
chcp 65001 >nul

echo 已生成文件列表 file_list.txt：
echo -------------------
type file_list.txt
echo -------------------

echo.
echo 生成完成！如果视频文件损坏，请先运行 01_fix_avi.bat 修复

pause
```

### 9.6 使用流程示例

```
1. 运行 01_extract.bat → 解压所有文件到 extracted\
2. 进入 extracted\ 目录
3. 运行 00_generate_list.bat → 生成文件列表
4. （可选）运行 01_fix_avi.bat → 如果视频文件损坏则修复
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
chcp 65001 >nul
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

:: 使用chcp 65001后再改回936，确保file_list.txt使用ANSI编码
chcp 65001 >nul

echo ==============================================
echo          文件列表生成器
echo ==============================================

:: 删除旧的文件列表
if exist file_list.txt del file_list.txt

:: 切换回ANSI编码生成文件列表，避免UTF-8 BOM问题
:: 使用 /on 按文件名排序（因为文件名已包含时间戳）
chcp 936 >nul
(
    for /f "delims=" %%f in ('dir /b /on *.avi') do (
        echo file '%%f'
    )
) > file_list.txt

:: 切回UTF-8显示中文
chcp 65001 >nul

echo 已生成文件列表 file_list.txt：
echo -------------------
type file_list.txt
echo -------------------

echo.
echo 生成完成！如果视频文件损坏，请先运行 01_fix_avi.bat 修复

pause
```

### 11.3 脚本3：修复损坏的AVI文件（`01_fix_avi.bat`）

用于修复损坏或不完整的AVI文件，使其能够正常拼接。修复后保持AVI格式。

```batch
@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

echo ==============================================
echo          脚本3：修复损坏的AVI文件
echo ==============================================

:: 创建修复后的文件目录
mkdir fixed 2>NUL

:: 修复所有AVI文件
set "count=0"
for %%f in (*.avi) do (
    set /a count+=1
    echo [%%count%] 正在修复: %%f
    
    :: 使用FFmpeg重新编码修复，保持AVI格式（MJPEG编码）
    ffmpeg -i "%%f" -c:v mjpeg -q:v 2 -c:a pcm_s16le "fixed\%%~nf_fixed.avi" -y -hide_banner
    
    :: 检查修复是否成功
    if exist "fixed\%%~nf_fixed.avi" (
        echo    修复成功！
    ) else (
        echo    修复失败！
    )
)

echo ==============================================
echo             修复完成！
echo 修复文件数：%count%
echo 输出目录：fixed\
echo 注：修复后的文件为MJPEG编码的AVI格式，保持与原格式兼容
echo ==============================================

pause
```

### 11.4 脚本4：视频拼接与插帧（`02_concat_interpolate.bat`）

用于根据文件列表拼接视频，并进行插帧处理。如果原始视频文件损坏，请先运行 `01_fix_avi.bat` 修复。

**GPU加速版本**（推荐）：

```batch
@echo off
setlocal enabledelayedexpansion

REM ==================== 配置参数（请根据实际情况修改）====================
set "TARGET_FPS=30"                    REM 目标帧率（你的原视频15fps，翻倍到30）
set "CRF=18"                           REM 编码质量（越小质量越高，推荐16-20）
set "OUTPUT_NAME=output_interpolated"  REM 输出文件名（不含扩展名）
set "GPU_ID=0"                         REM GPU设备ID（0为第一张显卡）

REM RIFE 相关配置
set "RIFE_PATH=C:\aaa\rife-ncnn-vulkan"    REM 请修改为你的实际路径！
set "RIFE_MODEL=models/rife-v4.6"      REM 模型选择（rife-v4.6质量最高）
set "ENABLE_TTA_SPATIAL=1"             REM 启用空间TTA（质量提升明显，1=启用 0=关闭）
set "ENABLE_TTA_TEMPORAL=1"            REM 启用时间TTA（减少闪烁，1=启用 0=关闭）

REM FFmpeg 配置
set "FFMPEG_PATH=ffmpeg"               REM 如果ffmpeg不在PATH中，填写完整路径，如 C:\ffmpeg\bin\ffmpeg.exe
set "USE_NVENC=1"                      REM 使用NVENC硬件编码（1=启用 0=软件编码）
REM ================================================================

REM 设置UTF-8编码显示中文
chcp 65001 >nul

echo ==============================================
echo      RIFE 视频拼接与插帧脚本 (GPU加速版)
echo ==============================================
echo.
echo 配置信息：
echo   目标帧率：%TARGET_FPS% fps
echo   编码质量：CRF %CRF%
echo   RIFE路径：%RIFE_PATH%
echo   空间TTA：%ENABLE_TTA_SPATIAL% (1=启用, 可提升画质但速度变慢)
echo   时间TTA：%ENABLE_TTA_TEMPORAL% (1=启用, 可减少闪烁但速度变慢)
echo.

REM 检查文件列表是否存在
if not exist file_list.txt (
    echo [错误] 未找到 file_list.txt 文件！
    echo.
    echo 请先创建 file_list.txt，格式如下：
    echo   file 'video1.avi'
    echo   file 'video2.avi'
    echo   file 'video3.mp4'
    echo.
    echo 注意：所有视频的分辨率、帧率、编码格式应一致
    pause
    exit /b
)

REM 检查 RIFE 是否存在
if not exist "%RIFE_PATH%\rife-ncnn-vulkan.exe" (
    echo [错误] 未找到 RIFE 程序！
    echo 请检查 RIFE_PATH 是否正确：%RIFE_PATH%
    echo.
    echo 提示：请从以下地址下载并解压到指定目录
    echo   https://github.com/nihui/rife-ncnn-vulkan/releases
    pause
    exit /b
)

REM 检查 FFmpeg
where %FFMPEG_PATH% >nul 2>nul
if %errorlevel% neq 0 (
    echo [错误] 未找到 FFmpeg！
    echo 请安装 FFmpeg 或设置正确的 FFMPEG_PATH
    pause
    exit /b
)

REM ==================== 步骤1：拼接视频 ====================
echo [1/4] 正在拼接视频...
%FFMPEG_PATH% -f concat -safe 0 -i file_list.txt -c copy temp_concat.mkv

if %errorlevel% neq 0 (
    echo [错误] 视频拼接失败！
    pause
    exit /b
)
echo [完成] 视频拼接成功
echo.

REM ==================== 步骤2：拆分为图片帧 ====================
echo [2/4] 正在将视频拆分为图片帧...
set "INPUT_DIR=input_frames"
set "OUTPUT_DIR=output_frames"

if exist "%INPUT_DIR%" rmdir /s /q "%INPUT_DIR%"
if exist "%OUTPUT_DIR%" rmdir /s /q "%OUTPUT_DIR%"
mkdir "%INPUT_DIR%"
mkdir "%OUTPUT_DIR%"

REM 拆帧（PNG格式，保证质量）
%FFMPEG_PATH% -i temp_concat.mkv -qscale:v 1 "%INPUT_DIR%\frame_%%08d.png"

if %errorlevel% neq 0 (
    echo [错误] 视频拆帧失败！
    pause
    exit /b
)

REM 计算帧数
set "frame_count=0"
for /f %%a in ('dir /b "%INPUT_DIR%\*.png" 2^>nul ^| find /c /v ""') do set "frame_count=%%a"
echo [完成] 成功拆分 %frame_count% 帧
echo.

REM ==================== 步骤3：RIFE 插帧处理 ====================
echo [3/4] 正在使用 RIFE 进行 AI 插帧（这可能需要较长时间）...
echo   输入帧数：%frame_count%
echo   目标倍数：2倍（%TARGET_FPS% fps）
echo.

cd /d "%RIFE_PATH%"

REM 构建 RIFE 参数
set "RIFE_ARGS=-i "%CD%\%INPUT_DIR%" -o "%CD%\%OUTPUT_DIR%" -m %RIFE_MODEL% -n 2 -g %GPU_ID% -j 2:4:2"

if "%ENABLE_TTA_SPATIAL%"=="1" set "RIFE_ARGS=%RIFE_ARGS% -x"
if "%ENABLE_TTA_TEMPORAL%"=="1" set "RIFE_ARGS=%RIFE_ARGS% -z"

echo 执行命令：rife-ncnn-vulkan.exe %RIFE_ARGS%
echo.

rife-ncnn-vulkan.exe %RIFE_ARGS%

if %errorlevel% neq 0 (
    echo [错误] RIFE 插帧失败！
    cd /d "%~dp0"
    pause
    exit /b
)

cd /d "%~dp0"

REM 计算输出帧数
set "output_frame_count=0"
for /f %%a in ('dir /b "%OUTPUT_DIR%\*.png" 2^>nul ^| find /c /v ""') do set "output_frame_count=%%a"
echo [完成] RIFE 处理完成，输出 %output_frame_count% 帧
echo.

REM ==================== 步骤4：合并为视频 ====================
echo [4/4] 正在将图片帧合并为视频...

REM 构建编码参数
set "ENCODE_ARGS=-framerate %TARGET_FPS% -i "%OUTPUT_DIR%\frame_%%08d.png" -c:v"

if "%USE_NVENC%"=="1" (
    REM NVIDIA NVENC 硬件编码
    set "ENCODE_ARGS=%ENCODE_ARGS% h264_nvenc -rc vbr -cq %CRF% -b:v 0 -preset p7 -tune hq"
    echo 使用 NVENC 硬件编码（高质量模式）
) else (
    REM 软件编码（质量稍高但速度慢）
    set "ENCODE_ARGS=%ENCODE_ARGS% libx264 -crf %CRF% -preset slow"
    echo 使用 libx264 软件编码（高质量模式）
)

REM 添加音频（从原视频复制）
%FFMPEG_PATH% %ENCODE_ARGS% -pix_fmt yuv420p -movflags +faststart -i temp_concat.mkv -map 0:v -map 1:a? -c:a copy "%OUTPUT_NAME%.mp4"

if %errorlevel% neq 0 (
    echo [错误] 视频合并失败！
    pause
    exit /b
)
echo [完成] 视频合并成功
echo.

REM ==================== 清理临时文件 ====================
echo 正在清理临时文件...
if exist temp_concat.mkv del temp_concat.mkv
if exist "%INPUT_DIR%" rmdir /s /q "%INPUT_DIR%"
if exist "%OUTPUT_DIR%" rmdir /s /q "%OUTPUT_DIR%"

echo.
echo ==============================================
echo              处理完成！
echo ==============================================
echo   输入帧率：%~2%（自动检测）
echo   输出帧率：%TARGET_FPS% fps
echo   输出文件：%OUTPUT_NAME%.mp4
echo   编码质量：CRF %CRF%
echo   RIFE配置：TTA空间=%ENABLE_TTA_SPATIAL%  TTA时间=%ENABLE_TTA_TEMPORAL%
echo ==============================================
echo.
echo 提示：
echo   1. 如果对质量不满意，可以降低 CRF 值（如 16）
echo   2. 如果处理速度太慢，可以关闭 TTA（设置 ENABLE_TTA_SPATIAL=0）
echo   3. 输出文件已添加音频轨道（如果有）
echo.

pause
```

#### 脚本参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FPS` | 30 | 目标帧率 |
| `CRF` | 23 | 视频质量（0-51，越小质量越高） |
| `OUTPUT_NAME` | output_interpolated | 输出文件名 |
| `USE_GPU` | 1 | 是否使用GPU加速（需安装支持CUDA的FFmpeg） |
| `USE_DAIN` | 0 | 是否使用DAIN-APP高质量插帧（需单独安装） |

#### 使用建议

1. **默认模式（推荐）**：`USE_GPU=1`, `USE_DAIN=0` - 使用NVIDIA GPU加速，性能提升5-10倍
2. **高质量模式**：`USE_GPU=0`, `USE_DAIN=1` - 使用DAIN-APP，插帧质量更高但速度较慢
3. **兼容性模式**：`USE_GPU=0`, `USE_DAIN=0` - 纯CPU处理，兼容性最好但最慢

---

#### 如何安装支持CUDA的FFmpeg

**方法1：使用NVIDIA官方构建（推荐）**

1. 访问 [NVIDIA Video Codec SDK](https://developer.nvidia.com/video-codec-sdk) 下载页面
2. 下载包含FFmpeg的完整工具包，或直接从以下地址下载预编译版本：
   - GitHub: [NVIDIA/ffmpeg](https://github.com/NVIDIA/FFmpeg)
   - 第三方镜像: [gyan.dev/ffmpeg/builds](https://www.gyan.dev/ffmpeg/builds/)

**方法2：使用Chocolatey（Windows）**

```powershell
choco install ffmpeg-full --version=6.0
```

**方法3：手动编译（高级用户）**

```bash
# 安装依赖
git clone https://git.videolan.org/git/ffmpeg/nv-codec-headers.git
cd nv-codec-headers && make install && cd ..

# 编译FFmpeg
git clone https://github.com/FFmpeg/FFmpeg.git
cd FFmpeg
./configure --enable-nvenc --enable-cuda --enable-cuvid --enable-nvdec
make -j$(nproc)
make install
```

---

#### 验证GPU加速是否生效

运行以下命令检查FFmpeg是否支持CUDA：

```bash
ffmpeg -encoders | findstr nvenc
```

正常输出应包含：
```
 V..... h264_nvenc           NVIDIA NVENC H.264 encoder (codec h264)
 V..... hevc_nvenc           NVIDIA NVENC HEVC encoder (codec hevc)
```

运行以下命令检查是否支持硬件解码：

```bash
ffmpeg -hwaccels
```

正常输出应包含：
```
cuda
```

运行脚本时，如果看到类似以下输出说明GPU加速成功：

```
Stream mapping:
  Stream #0:0 -> #0:0 (rawvideo (native) -> h264 (h264_nvenc))
Press [q] to stop, [?] for help
frame=   50 fps= 30 q=23.0 size=    1024kB time=00:00:01.66 bitrate=5053.5kbits/s speed=10.0x
```

> **注意**：使用GPU加速需要满足以下条件：
> - NVIDIA显卡（Kepler架构及以上，建议GTX 10系列及更高）
> - 安装最新的NVIDIA驱动
> - 安装支持CUDA的FFmpeg版本

### 11.4 脚本4：AVI转MP4（`03_convert_to_mp4.bat`）

用于将AVI文件批量转换为MP4格式。

```batch
@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

:: 配置参数
set "CRF=23"
set "INPUT_FILE=output_interpolated.avi"

echo ==============================================
echo          脚本4：AVI转MP4（仅转换拼接视频）
echo ==============================================

:: 检查输入文件是否存在
if not exist "%INPUT_FILE%" (
    echo 错误：未找到 %INPUT_FILE% 文件！
    echo 请先运行 02_concat_interpolate.bat 进行视频拼接和插帧
    pause
    exit /b
)

:: 创建输出目录
mkdir mp4_output 2>NUL

:: 转换拼接好的视频
echo 正在转换: %INPUT_FILE%
ffmpeg -i "%INPUT_FILE%" -c:v libx264 -crf %CRF% -c:a aac "mp4_output\%INPUT_FILE:.avi=.mp4%" -y

echo ==============================================
echo             转换完成！
echo 输出文件：mp4_output\%INPUT_FILE:.avi=.mp4%
echo ==============================================

pause
```

### 11.5 使用流程示例

```
1. 运行 01_extract.bat → 解压所有文件到 extracted\
2. 进入 extracted\ 目录
3. 运行 00_generate_list.bat → 生成文件列表
4. （可选）运行 01_fix_avi.bat → 如果视频文件损坏则修复
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