#! /usr/bin/env python
# encoding: utf-8
# Federico Allocati - 2016

"""
Quick n dirty DART detection
"""

import os
from waflib.Configure import conf


def options(opt):
    opt.add_option('--dart', type='string', help='path to dart', dest='dart')


@conf
def check_dart(conf, **kw):
    required = 'required' in kw and kw.get('required', False)
    require_graphics = 'require_graphics' in kw and kw.get('require_graphics', False)

    includes_check = ['/usr/include', '/usr/local/include']
    libs_check = ['/usr/local/lib', '/usr/lib']
    resibots_dir = conf.options.resibots if hasattr(conf.options, 'resibots') and conf.options.resibots else None

    if resibots_dir:
        includes_check = [resibots_dir + '/include'] + includes_check
        libs_check = [resibots_dir + '/lib'] + libs_check

    if conf.options.dart:
        includes_check = [conf.options.dart + '/include'] + includes_check
        libs_check = [conf.options.dart + '/lib'] + libs_check

    conf.start_msg('Checking for DART includes')
    res = False
    try:
        for incl in ['dart', 'dart-core']:
            res = res and conf.find_file('dart/' + incl + '.h', includes_check)
    except:
        res = False

    if res:
        conf.end_msg('ok')
    else:
        if conf.options.dart and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.dart, resibots_dir)
        elif conf.options.dart or resibots_dir:
            msg = 'not found in %s' % (conf.options.dart if conf.options.dart else resibots_dir)
        else:
            msg = 'not found, use --dart=/path/to/dart or --resibots=/path/to/resibots'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
        return

    conf.start_msg('Checking for DART libs')
    try:
        for lib in ['dart', 'dart-core']:
            res = res and conf.find_file('lib' + lib + '.so', libs_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_DART = [os.path.expanduser(include) for include in includes_check]
        conf.env.LIBPATH_DART = [os.path.expanduser(lib) for lib in libs_check]
        conf.env.LIB_DART = ['dart', 'dart-core']
        conf.env.DEFINES_DART = ['USE_DART']
        conf.end_msg('ok')
    else:
        if conf.options.dart and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.dart, resibots_dir)
        elif conf.options.dart or resibots_dir:
            msg = 'not found in %s' % (conf.options.dart if conf.options.dart else resibots_dir)
        else:
            msg = 'not found, use --dart=/path/to/dart or --resibots=/path/to/resibots'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')

    conf.start_msg('Checking for DART OSG includes')
    try:
        res = conf.find_file('osgDart/osgDart.h', includes_check)
    except:
        res = False

    if res:
        conf.end_msg('ok')
    else:
        if conf.options.dart and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.dart, resibots_dir)
        elif conf.options.dart or resibots_dir:
            msg = 'not found in %s' % (conf.options.dart if conf.options.dart else resibots_dir)
        else:
            msg = 'not found, use --dart=/path/to/dart or --resibots=/path/to/resibots'

        if require_graphics:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
        return

    conf.start_msg('Checking for DART OSG libs')
    try:
        res = conf.find_file('libosgDart.so', libs_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_DART_GRAPHIC = [os.path.expanduser(include) for include in includes_check]
        conf.env.LIBPATH_DART_GRAPHIC = [os.path.expanduser(lib) for lib in libs_check]
        conf.env.LIB_DART_GRAPHIC = ['dart', 'dart-core', 'osgDart']
        conf.env.DEFINES_DART_GRAPHIC = ['USE_DART_GRAPHIC']
        conf.end_msg('ok')
    else:
        if conf.options.dart and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.dart, resibots_dir)
        elif conf.options.dart or resibots_dir:
            msg = 'not found in %s' % (conf.options.dart if conf.options.dart else resibots_dir)
        else:
            msg = 'not found, use --dart=/path/to/dart or --resibots=/path/to/resibots'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
