#! /usr/bin/env python
# encoding: utf-8
# Federico Allocati - 2016

"""
Quick n dirty omni_dart detection
"""

import os
from waflib.Configure import conf


def options(opt):
    opt.add_option('--omni_dart', type='string', help='path to omni_dart', dest='omni_dart')


@conf
def check_omni_dart(conf, **kw):
    required = 'required' in kw and kw.get('required', False)
    includes_check = ['/usr/include', '/usr/local/include']
    resibots_dir = conf.options.resibots if hasattr(conf.options, 'resibots') and conf.options.resibots else None

    if resibots_dir:
        includes_check = [resibots_dir + '/include'] + includes_check

    if conf.options.omni_dart:
        includes_check = [conf.options.omni_dart + '/include'] + includes_check

    conf.start_msg('Checking for omni_dart includes')
    try:
        res = conf.find_file('omni_dart/omnipointer.hpp', includes_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_OMNI_DART = [os.path.expanduser(include) for include in includes_check]
        conf.env.DEFINES_OMNI_DART = ['USE_OMNI_DART']
        conf.end_msg('ok')
    else:
        if conf.options.omni_dart and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.omni_dart, resibots_dir)
        elif conf.options.omni_dart or resibots_dir:
            msg = 'not found in %s' % (conf.options.omni_dart if conf.options.omni_dart else resibots_dir)
        else:
            msg = 'not found, use --omni_dart=/path/to/omni_dart or --resibots=/path/to/resibots'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
