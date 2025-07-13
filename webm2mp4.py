#!/usr/bin/env python3
import subprocess
import os
import sys

def convert_webm_to_mp4(input_file, output_file=None):
    if output_file is None:
        output_file = os.path.splitext(input_file)[0] + '.mp4'
    
    cmd = [
        'ffmpeg',
        '-i', input_file,
        '-c:v', 'libx264',
        '-crf', '23',
        '-c:a', 'aac',
        '-y',  # 覆盖输出文件
        output_file
    ]
    
    try:
        subprocess.run(cmd, check=True)
        print(f"转换成功: {input_file} -> {output_file}")
    except subprocess.CalledProcessError as e:
        print(f"转换失败: {e}")

def batch_convert(directory='.'):
    for filename in os.listdir(directory):
        if filename.lower().endswith('.webm'):
            input_path = os.path.join(directory, filename)
            convert_webm_to_mp4(input_path)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if os.path.isfile(arg) and arg.lower().endswith('.webm'):
                convert_webm_to_mp4(arg)
            else:
                print(f"无效的输入文件: {arg}")
    else:
        batch_convert()