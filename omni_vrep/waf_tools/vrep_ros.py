#! /usr/bin/env python
# encoding: utf-8
# Federico Allocati - 2016

"""
Quick n dirty V-REP ROS plugin detection
"""

import os
from waflib.Configure import conf
import re


def options(opt):
    opt.add_option('--vrep_ros', type='string', help='path to vrep_ros', dest='vrep_ros')


@conf
def check_vrep_ros(conf, **kw):
    required = 'required' in kw and kw.get('required', False)

    if conf.options.vrep_ros:
        includes_check = [conf.options.vrep_ros + '/include']
        libs_check = [conf.options.vrep_ros + '/lib']
    else:
        if 'ROS_PACKAGE_PATH' not in os.environ:
            conf.start_msg('Checking for V-REP ROS plugin')
            if required:
                conf.fatal('ROS_PACKAGE_PATH not in environmental variables, use --vrep_ros=/path/to/vrep_ros')
            else:
                conf.end_msg('ROS_PACKAGE_PATH not in environmental variables, use --vrep_ros=/path/to/vrep_ros', 'YELLOW')
            return

        path = os.environ['ROS_PACKAGE_PATH']
        paths = re.split(":", path)
        path = os.path.join(paths[0], '../devel')

        includes_check = [path + '/include']
        libs_check = [path + '/lib']

    conf.start_msg('Checking for V-REP ROS plugin includes')
    try:
        res = conf.find_file('vrep_common/VrepInfo.h', includes_check)
    except:
        res = False

    if res:
        conf.end_msg('ok')
    else:
        if conf.options.vrep_ros:
            msg = 'not found in %s' % conf.options.vrep_ros
        elif 'ROS_PACKAGE_PATH' in os.environ:
            msg = 'not found in %s (from ROS_PACKAGE_PATH environmental variable)' % path
        else:
            msg = 'not found, use --vrep_ros=/path/to/vrep_ros'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
        return

    conf.start_msg('Checking for V-REP ROS plugin libs')
    try:
        res = conf.find_file('libv_repExtRos.so', libs_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_VREP_ROS = [os.path.expanduser(include) for include in includes_check]
        conf.env.LIBPATH_VREP_ROS = [os.path.expanduser(lib) for lib in libs_check]
        conf.env.LIB_VREP_ROS = ['v_repExtRos']
        conf.env.DEFINES_VREP_ROS = ['USE_VREP_ROS']
        conf.end_msg('ok')
    else:
        if conf.options.vrep_ros:
            msg = 'not found in %s' % conf.options.vrep_ros
        elif 'ROS_PACKAGE_PATH' in os.environ:
            msg = 'not found in %s (from ROS_PACKAGE_PATH environmental variable)' % path
        else:
            msg = 'not found, use --vrep_ros=/path/to/vrep_ros'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
