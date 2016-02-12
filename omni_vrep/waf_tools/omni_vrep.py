#! /usr/bin/env python
# encoding: utf-8
# Federico Allocati - 2016

"""
Quick n dirty omni_vrep detection
"""

import os
from waflib.Configure import conf


def options(opt):
    opt.add_option('--omni_vrep', type='string', help='path to omni_vrep', dest='omni_vrep')


@conf
def check_omni_vrep(conf, **kw):
    required = 'required' in kw and kw.get('required', False)
    includes_check = ['/usr/include', '/usr/local/include']
    resibots_dir = conf.options.resibots if hasattr(conf.options, 'resibots') and conf.options.resibots else None

    if resibots_dir:
        includes_check = [resibots_dir + '/include'] + includes_check

    if conf.options.omni_vrep:
        includes_check = [conf.options.omni_vrep + '/include'] + includes_check

    conf.start_msg('Checking for omni_vrep includes')
    try:
        res = conf.find_file('omni_vrep/omnipointer.hpp', includes_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_OMNI_VREP = [os.path.expanduser(include) for include in includes_check]
        conf.env.DEFINES_OMNI_VREP = ['USE_OMNI_VREP']
        conf.end_msg('ok')
    else:
        if conf.options.omni_vrep and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.omni_vrep, resibots_dir)
        elif conf.options.omni_vrep or resibots_dir:
            msg = 'not found in %s' % (conf.options.omni_vrep if conf.options.omni_vrep else resibots_dir)
        else:
            msg = 'not found, use --omni_vrep=/path/to/omni_vrep or --resibots=/path/to/resibots'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
