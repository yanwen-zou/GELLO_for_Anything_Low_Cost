#!/usr/bin/env python3
import rospy
import numpy as np
import pickle
import os
import datetime
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from threading import Lock
import signal
import sys

import tensorflow as tf
import rlds
import cv2
import pyttsx3
from num2words import num2words
import re
import subprocess

from xarm.wrapper import XArmAPI
arm = XArmAPI('192.168.1.222')

exit_flag = False

def handle_shutdown(signum, frame):
    global exit_flag
    exit_flag = True
    print("\n[Shutdown] Ctrl+C pressed. Cleaning up...")
    rospy.signal_shutdown("Ctrl+C exit")

signal.signal(signal.SIGINT, handle_shutdown)

def rlds_signature(example: dict):
    def nested_spec(d):
        if isinstance(d, dict):
            return {k: nested_spec(v) for k, v in d.items()}
        return d
    return nested_spec(example)

def speak(text: str):
    try:
        def replace_numbers(match):
            return num2words(int(match.group()))
        text = re.sub(r'\b\d+\b', replace_numbers, text)
        subprocess.run(["espeak-ng", "-v", "en", "-s", "150", text])
    except Exception as e:
        rospy.logwarn(f"Speech failed: {e}")

class Recorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = Lock()

        self.latest_image_1 = None  # /cam_1
        self.latest_image_2 = None  # /cam_2
        self.preview_image_1 = None
        self.preview_image_2 = None
        self.latest_state = None
        self.latest_action = None
        self.episode = []
        self.recording = False

        rospy.Subscriber('/cam_1', Image, self.image1_callback)
        rospy.Subscriber('/cam_2', Image, self.image2_callback)
        rospy.Subscriber('/robot_state', Float64MultiArray, self.state_callback)
        rospy.Subscriber('/robot_action', Float64MultiArray, self.action_callback)

    def image1_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert cam_1 image: {e}")
            return
        with self.lock:
            self.latest_image_1 = cv_image
            self.preview_image_1 = cv_image.copy()
            self.try_record()

    def image2_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert cam_2 image: {e}")
            return
        with self.lock:
            self.latest_image_2 = cv_image
            self.preview_image_2 = cv_image.copy()
            self.try_record()

    def state_callback(self, msg):
        with self.lock:
            self.latest_state = np.array(msg.data, dtype=np.float64)
            self.try_record()

    def action_callback(self, msg):
        with self.lock:
            self.latest_action = np.array(msg.data, dtype=np.float64)
            self.try_record()

    def try_record(self):
        if not self.recording:
            return

        if (self.latest_image_1 is not None and
            self.latest_image_2 is not None and
            self.latest_state is not None and
            self.latest_action is not None):

            ts = {
                "image_1": self.latest_image_1.copy(),
                "image_2": self.latest_image_2.copy(),
                "state": self.latest_state.copy(),
                "action": self.latest_action.copy()
            }
            self.episode.append(ts)
            rospy.loginfo(f"Recorded step {len(self.episode)}")
            self.latest_state = None
            self.latest_action = None

    def get_last_frames(self):
        with self.lock:
            img1 = self.preview_image_1.copy() if self.preview_image_1 is not None else None
            img2 = self.preview_image_2.copy() if self.preview_image_2 is not None else None
            return img1, img2

    def save_episode(self, episode_id: int):
        if not self.episode:
            rospy.logwarn("No data recorded, skipping save.")
            return

        save_dir = os.path.expanduser("~/ros_save_data/pick_cylinder")
        os.makedirs(save_dir, exist_ok=True)

        tfrecord_path = os.path.join(save_dir, f"lerobot_episode_{episode_id}")
        try:
            rlds_episode = []
            for i, ts in enumerate(self.episode):
                img1_encoded = cv2.imencode('.jpg', ts["image_1"])[1].tobytes()
                img2_encoded = cv2.imencode('.jpg', ts["image_2"])[1].tobytes()

                rlds_episode.append({
                    "observation": {
                        "image_1": img1_encoded,
                        "image_2": img2_encoded,
                        "state": ts["state"]
                    },
                    "action": ts["action"],
                    "is_first": i == 0,
                    "is_last": i == len(self.episode) - 1,
                    "is_terminal": i == len(self.episode) - 1,
                })

            ds = tf.data.Dataset.from_generator(
                lambda: (step for step in rlds_episode),
                output_signature=rlds_signature({
                    "observation": {
                        "image_1": tf.TensorSpec(shape=(), dtype=tf.string),
                        "image_2": tf.TensorSpec(shape=(), dtype=tf.string),
                        "state": tf.TensorSpec(shape=(len(self.episode[0]['state']),), dtype=tf.float64),
                    },
                    "action": tf.TensorSpec(shape=(len(self.episode[0]['action']),), dtype=tf.float64),
                    "is_first": tf.TensorSpec(shape=(), dtype=tf.bool),
                    "is_last": tf.TensorSpec(shape=(), dtype=tf.bool),
                    "is_terminal": tf.TensorSpec(shape=(), dtype=tf.bool),
                })
            )

            tf.data.experimental.save(ds, tfrecord_path)
            rospy.loginfo(f"✅ RLDS TFRecord episode saved to {tfrecord_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save RLDS TFRecord: {e}")

        self.episode = []

def get_next_episode_id(save_dir: str) -> int:
    existing = [f for f in os.listdir(save_dir) if f.startswith("lerobot_episode_")]
    episode_ids = []
    for name in existing:
        try:
            num = int(name.split("_")[-1])
            episode_ids.append(num)
        except:
            continue
    return max(episode_ids, default=0) + 1

if __name__ == '__main__':
    try:
        rospy.init_node('recorder')
        recorder = Recorder()

        num_episodes = 2
        record_duration = 40
        rest_duration = 30
        continue_recording = True
        save_dir = os.path.expanduser("~/ros_save_data/pick_cylinder")
        os.makedirs(save_dir, exist_ok=True)

        start_id = get_next_episode_id(save_dir) if continue_recording else 1
        # init_qpos = np.radians([14.1, -8, -24.7, 196.9, 62.3, -8.8])

        cv2.namedWindow("Preview: cam_1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Preview: cam_2", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Preview: cam_1", 640, 360)
        cv2.resizeWindow("Preview: cam_2", 640, 360)

        for episode_id in range(start_id, start_id + num_episodes):
            if exit_flag:
                break

            msg = f"recording episode {episode_id}"
            rospy.loginfo(f"🔴 {msg}, duration {record_duration}s...")
            speak(msg)

            recorder.recording = True
            start_time = time.time()
            while time.time() - start_time < record_duration and not rospy.is_shutdown() and not exit_flag:
                frame1, frame2 = recorder.get_last_frames()
                if frame1 is not None:
                    cv2.putText(frame1, f"Recording: {int(time.time()-start_time)} s", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("Preview: cam_1", frame1)
                if frame2 is not None:
                    cv2.putText(frame2, f"Recording: {int(time.time()-start_time)} s", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.imshow("Preview: cam_2", frame2)
                key = cv2.waitKey(10)
                if key == ord('q'):
                    rospy.loginfo("⏹️ Recording interrupted by user.")
                    break

            recorder.recording = False

            if exit_flag:
                break

            


            speak(f"recording completed, saving episode {episode_id}")

            # init_qpos = np.array([14.1, -8, -24.7, 196.9, 62.3, -8.8])
            # init_qpos = np.radians(init_qpos)
            # arm.set_servo_angle(angle=init_qpos,speed=8,is_radian=True)

            recorder.save_episode(episode_id)

            rospy.loginfo(f"✅ Episode {episode_id} saved. Resting for {rest_duration}s...\n")
            speak(f"rest for {rest_duration} seconds")
            speak("press Q to skip rest")

            rest_start = time.time()
            while time.time() - rest_start < rest_duration and not exit_flag:
                frame1, frame2 = recorder.get_last_frames()
                if frame1 is not None:
                    cv2.putText(frame1, f"Resting: {int(time.time()-rest_start)} s", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                    cv2.imshow("Preview: cam_1", frame1)
                if frame2 is not None:
                    cv2.putText(frame2, f"Resting: {int(time.time()-rest_start)} s", (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                    cv2.imshow("Preview: cam_2", frame2)
                key = cv2.waitKey(100)
                if key == ord('q'):
                    rospy.loginfo("⏭️ Rest skipped by user.")
                    break

        speak("all episodes recorded and saved")

        rospy.loginfo("🎉 All episodes completed.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received.Shutdown.")


