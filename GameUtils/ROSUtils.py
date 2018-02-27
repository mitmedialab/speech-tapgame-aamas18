"""
This is a Utilities class for the Unity Games! Handy little scripts, and other small functions,
which should be idempotent and assume no state or context
"""
# -*- coding: utf-8 -*-
# pylint: disable=import-error

import subprocess

from . import GlobalSettings


def is_rostopic_present(topic_name):
    """
    If on a ROS installed computer, runs a little script to see whether a certain topic is present
    Helpful for improving the resiliance / flexibility of certain other modules
    """

    if GlobalSettings.USE_ROS:
        cmd = "rostopic list | grep " + topic_name
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
        result = process.communicate()[0].decode('utf-8') #interpret bytes as string
        print(result)
        return result is not ''
    else:
        return False
