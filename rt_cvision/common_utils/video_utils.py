import cv2
import os

def video_to_images(video_path, output_folder):
    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the video file
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Unable to open video file {video_path}")
        return

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            # Break the loop if there are no frames left to read
            break

        # Define the filename for each frame
        frame_filename = os.path.join(output_folder, f"frame_{frame_count:04d}.jpg")
        # Save the frame as an image
        cv2.imwrite(frame_filename, frame)
        frame_count += 1

    cap.release()
    print(f"Extracted and saved {frame_count} frames in '{output_folder}'.")

if __name__ == "__main__":
    # Set the path to your video file and the folder where images will be saved
    video_path = "/media/appuser/rt_cvision/AGR_gate02_gate02_2025-03-07_11-15-00_2025-03-07_11-30-00.mp4"  # Replace with your video file
    output_folder = "/media/appuser/rt_cvision/frames"        # Folder where frames will be stored
    video_to_images(video_path, output_folder)
