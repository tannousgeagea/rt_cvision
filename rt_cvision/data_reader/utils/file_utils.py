import os
import logging

def examine_src(src:str, VALID_FORMAT:list = ['jpeg', 'png', 'jpg', 'webp']):
    images = []
    try:
        if os.path.isdir(src):
            images = sorted([os.path.join(src, f) for f in os.listdir(src) if f.endswith(tuple(VALID_FORMAT))])
            return images
        elif os.path.isfile(src):
            images = [src] if src.endswith(tuple(VALID_FORMAT)) else []
            return images
    except Exception as err:
        logging.error(f'Error examining files src: {src}: {err}')
        
    return images