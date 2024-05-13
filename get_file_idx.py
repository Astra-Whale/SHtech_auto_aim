import glob
import shutil
import os
idx = -1
os.makedirs("/home/magician/autoaim_log", exist_ok=True)

for name in glob.glob("/home/magician/autoaim_log/*.log"):
    this_idx = int(os.path.basename(name)[:-4])
    if this_idx>idx:
        idx = this_idx

for name in glob.glob("/home/magician/autoaim_log/*.log"):
    this_idx = int(os.path.basename(name)[:-4])
    if this_idx + 10 < idx:
        shutil.remove(name)
print(f"/home/magician/autoaim_log/{idx+1:04d}.log")
