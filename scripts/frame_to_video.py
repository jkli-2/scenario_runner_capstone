import os
import glob
import re
import subprocess
import shutil

script_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(script_dir, '..'))
input_dir = os.path.join(root_dir, '_out')
output_dir = os.path.join(root_dir, '_renamed')
vid_out_dir = os.path.join(root_dir, '_vid')

input_pattern = os.path.join(input_dir, 'ego_*_*.png')
renamed_pattern = "frame_%06d.png"
output_video = os.path.join(vid_out_dir, 'ego_video.mp4')
framerate = 5

os.makedirs(output_dir, exist_ok=True)
os.makedirs(vid_out_dir, exist_ok=True)

image_files = glob.glob(input_pattern)

if not image_files:
    print(f"No input images found in {input_dir}")
    exit(1)

def extract_frame_number(path):
    match = re.search(r'_(\d{6})\.png$', path)
    return int(match.group(1)) if match else float('inf')

image_files.sort(key=extract_frame_number)

print(f"Renaming {len(image_files)} images into {output_dir}/ ...")

for i, src_path in enumerate(image_files):
    dst_path = os.path.join(output_dir, renamed_pattern % (i + 1))
    shutil.copy(src_path, dst_path)

ffmpeg_cmd = [
    "ffmpeg",
    "-y",  # Overwrite if exists
    "-framerate", str(framerate),
    "-i", os.path.join(output_dir, "frame_%06d.png"),
    "-c:v", "libx264",
    "-pix_fmt", "yuv420p",
    output_video
]

print(f"Running FFmpeg to generate video at {output_video}...")
subprocess.run(ffmpeg_cmd)

print(f"Done! Video saved to: {output_video}")
