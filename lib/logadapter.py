import logging as logg


class LogAdapter(logg.LoggerAdapter):
    def process(self, msg, kwargs):
        ms = str(msg)
        ms = ms.replace("\n", "\n|    ")
        return ms, kwargs


# set style of logging
log_fmt = logg.Formatter('%(asctime)s %(levelname)s - %(message)s', '%m.%d.%Y %H:%M:%S')
root_logger = logg.getLogger()
root_logger.setLevel(logg.DEBUG)

# output log to logfile
file_handler = logg.FileHandler("runconfigs.log")
file_handler.setFormatter(log_fmt)
root_logger.addHandler(file_handler)

# output log to stderr
console_handler = logg.StreamHandler()
console_handler.setFormatter(log_fmt)
root_logger.addHandler(console_handler)

# set levelnames to be 4 chars long
logg.addLevelName(logg.NOTSET,   'NOTS')
logg.addLevelName(logg.DEBUG,    'DBUG')
logg.addLevelName(logg.INFO,     'INFO')
logg.addLevelName(logg.WARNING,  'WARN')
logg.addLevelName(logg.ERROR,    'ERRR')
logg.addLevelName(logg.CRITICAL, 'CRIT')

logging = LogAdapter(logg.getLogger(), None)
