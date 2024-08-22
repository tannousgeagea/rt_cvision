import os
import cv2
import logging



def save_snapshot(params):
    try:
        assert 'snapshot' in params, f"Missing argument in save_snapshot: snapshot"
        assert 'snapshot_dir' in params, f"Missing argument in save_snapshot: snapshot_dir"
        assert 'filename' in params, f"Missing argument in save_snapshot: filename"
        
        os.makedirs(params.get('snapshot_dir'), exist_ok=True)
        cv2.imwrite(f"{params.get('snapshot_dir')}/{params.get('filename')}", params.get('snapshot'))
        
    except Exception as err:
        logging.error(f"Error saving impurity snapshots: {err}")
        
        
def save_experiment(params):
    try:
        assert 'cv_image' in params, f"Missing argument in save_snapshot: cv_image"
        assert 'experiment_dir' in params, f"Missing argument in save_snapshot: experiment_dir"
        assert 'filename' in params, f"Missing argument in save_snapshot: filename"
        
        os.makedirs(params.get('experiment_dir'), exist_ok=True)
        cv2.imwrite(f"{params.get('experiment_dir')}/{params.get('filename')}", params.get('cv_image'))
        
    except Exception as err:
        logging.error(f"Error saving impurity experiment images {params.get('filename')}: {err}")