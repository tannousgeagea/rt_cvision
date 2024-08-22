import os
import logging

class Logger:
    def __init__(self, log_path):
        self.log_path = log_path

    def setLogger(self):
        try:
            logging.basicConfig(level=logging.INFO)
            self.logger = logging.getLogger()
            self.logger.setLevel(logging.INFO)
            formatter = logging.Formatter('%(asctime)s:%(name)s:%(levelname)s:%(message)s:%(funcName)s:%(lineno)d')
            if not os.path.exists(self.log_path):
                os.makedirs(self.log_path)

            if os.path.exists(self.log_path + "/log.txt"):
                os.remove(self.log_path + "/log.txt")
            
            log_file= self.log_path + '/log.txt'
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(logging.INFO)
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
        except Exception as err:
            print('❌  Error: %s' %err)