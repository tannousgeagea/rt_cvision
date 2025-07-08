import logging

class Logger:
    def __init__(self, name: str = '', level=logging.INFO) -> None:
        self.name = name
        self.prefix = f"[{self.name}]: " if self.name else ""
        self.logger = logging.getLogger(name or 'root')
        self.logger.setLevel(level)

        # Prevent double handlers
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s: %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

        # Avoid log message propagation to root logger
        self.logger.propagate = False

    def debug(self, msg: str):
        logging.debug(self.prefix + msg)

    def info(self, msg: str):
        logging.info(self.prefix + msg)

    def warning(self, msg: str):
        logging.warning(self.prefix + msg)

    def error(self, msg: str):
        logging.error(self.prefix + msg)

    def critical(self, msg: str):
        logging.critical(self.prefix + msg)

    def exception(self, msg: str):
        logging.exception(self.prefix + msg)

    def set_level(self, level):
        self.logger.setLevel(level)