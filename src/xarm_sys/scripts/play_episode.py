import pickle
import cv2
import numpy as np
import sys

if len(sys.argv) != 2:
    print("Usage: python3 play_episode.py <path_to_pkl_file>")
    exit()

pkl_path = sys.argv[1]

# 加载数据
with open(pkl_path, 'rb') as f:
    episode = pickle.load(f)

print(f"Loaded {len(episode)} steps.")

# 播放
for i, step in enumerate(episode):
    img = step['image']
    state = step['state']

    # 在图像上叠加状态信息
    text = f"State: {np.round(state, 2)}"
    img_vis = img.copy()
    cv2.putText(img_vis, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Episode Playback", img_vis)
    key = cv2.waitKey(100)  # 每帧播放 100ms
    if key == 27:  # ESC 退出
        break

cv2.destroyAllWindows()
