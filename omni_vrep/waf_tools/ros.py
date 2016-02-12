#! /usr/bin/env python
# encoding: utf-8
# Federico Allocati - 2016

"""
Quick n dirty ROS detection
"""

import os
from waflib.Configure import conf
from waflib import Utils


def options(opt):
    opt.add_option('--ros', type='string', help='path to ros', dest='ros')


@conf
def check_ros(conf, **kw):
    required = 'required' in kw and kw.get('required', False)

    if conf.options.ros:
        includes_check = [conf.options.ros + '/include']
        libs_check = [conf.options.ros + '/lib']
    else:
        if 'ROS_DISTRO' not in os.environ:
            conf.start_msg('Checking for ROS')
            if required:
                conf.fatal('ROS_DISTRO not in environmental variables, use --ros=/path/to/ros')
            else:
                conf.end_msg('ROS_DISTRO not in environmental variables, use --ros=/path/to/ros', 'YELLOW')
            return

        includes_check = ['/opt/ros/' + os.environ['ROS_DISTRO'] + '/include']
        libs_check = ['/opt/ros/' + os.environ['ROS_DISTRO'] + '/lib']

    conf.start_msg('Checking for ROS includes')
    try:
        res = conf.find_file('ros/ros.h', includes_check)
    except:
        res = False

    if res:
        conf.end_msg('ok')
    else:
        if conf.options.ros:
            msg = 'not found in %s' % conf.options.ros
        elif 'ROS_DISTRO' in os.environ:
            msg = 'not found in %s (from ROS_DISTRO environmental variable)' % os.environ['ROS_DISTRO']
        else:
            msg = 'not found, use --ros=/path/to/ros'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
        return

    libs = Utils.to_list(kw.get('lib', []))

    conf.start_msg('Checking for ROS libs')
    try:
        for lib in libs:
            res = res and conf.find_file('lib'+lib+'.so', libs_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_ROS = [os.path.expanduser(include) for include in includes_check]
        conf.env.LIBPATH_ROS = [os.path.expanduser(lib) for lib in libs_check]
        conf.env.LIB_ROS = libs
        conf.env.DEFINES_ROS = ['USE_ROS']
        conf.end_msg('ok')
    else:
        if conf.options.ros:
            msg = 'not found in %s' % conf.options.ros
        elif 'ROS_DISTRO' in os.environ:
            msg = 'not found in %s (from ROS_DISTRO environmental variable)' % os.environ['ROS_DISTRO']
        else:
            msg = 'not found, use --ros=/path/to/ros'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
