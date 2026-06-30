import logging

import colorlog

DEFAULT_LOG_FORMAT = "%(log_color)s[%(asctime)s.%(msecs)03d] [%(levelname)s] %(name)s: %(message)s"
DEFAULT_LOG_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def default_logger(
    name: str = "logger", level=logging.INFO, fmt=DEFAULT_LOG_FORMAT, datefmt=DEFAULT_LOG_DATE_FORMAT, stream=None
) -> logging.Logger:
    """
    Create a default logger with colored output.

    Args:
        name (str, optional): Name of the logger (default is 'logger').
        level (int, optional): Logging level (default is logging.INFO).
        log_format (str, optional): Log format string (default is DEFAULT_LOG_FORMAT).
        stream: Stream to write log messages (default is None, which uses the console).

    Returns:
        logging.Logger: A configured logger instance.

    This function creates and configures a logger with colored output using the colorlog library. You can specify
    the logger's name, logging level, log format, and the output stream (e.g., sys.stdout).

    Example:
        logger = default_logger(name='my_logger', level=logging.DEBUG)
        logger.info("This is an info message.")
    """
    logger = colorlog.getLogger(name)
    if not logger.hasHandlers():
        handler = colorlog.StreamHandler(stream) if stream is not None else colorlog.StreamHandler()
        handler.setFormatter(colorlog.ColoredFormatter(fmt=fmt, datefmt=datefmt))
        logger.addHandler(handler)
    logger.setLevel(level)
    return logger
