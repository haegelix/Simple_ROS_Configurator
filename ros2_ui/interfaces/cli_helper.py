import subprocess
import re
from ros2_ui.exceptions.packageexception import PackageException
from ros2_ui.interfaces.Log import logging

pattern_empty_string = re.compile(r'[\n\t\r _-]*')


def runcommand(command: str, shortname: str, logger=logging):
    # Split command without splitting nested Strings.
    pattern = re.compile(r'''((?:[^ "']|"[^"]*"|'[^']*')+)''')
    cmd = pattern.split(command)

    # delete al empty strings
    cmd = [c for c in cmd if c and c != ' ']

    # unquote
    cmd = [c.replace("\"", "").replace("'", "") for c in cmd]
    # logging.debug(cmd)

    # run the command and deal with output
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    return_val = proc.wait()
    stdout, stderr = proc.communicate()
    if not len(pattern_empty_string.sub("", str(stdout))) == 0:  # stdout not empty --> log it
        logger.info("STDOUT of '" + shortname + "...':\n" + str(stdout))
    if not len(pattern_empty_string.sub("", str(stderr))) == 0:  # stderr not empty --> log it
        logger.info("STDERR of '" + shortname + "...':\n" + str(stderr))
    logger.info(shortname + " returned code " + str(return_val))
    if return_val != 0:
        raise PackageException("Aborted because one step failed!")
