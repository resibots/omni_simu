#! /usr/bin/env python
# encoding: utf-8
# JB Mouret - 2009
# Federico Allocati - 2016

"""
Quick n dirty eigen detection
"""

from waflib.Configure import conf


def options(opt):
    opt.add_option('--eigen', type='string', help='path to eigen', dest='eigen')


@conf
def check_eigen(conf, **kw):
    required = 'required' in kw and kw.get('required', False)
    includes_check = ['/usr/include/eigen3', '/usr/local/include/eigen3', '/usr/include', '/usr/local/include']
    resibots_dir = conf.options.resibots if hasattr(conf.options, 'resibots') and conf.options.resibots else None

    if resibots_dir:
        includes_check = [resibots_dir + '/include'] + includes_check

    if conf.options.eigen:
        includes_check = [conf.options.eigen + '/include'] + includes_check

    conf.start_msg('Checking for Eigen includes')
    try:
        res = conf.find_file('Eigen/Core', includes_check)
    except:
        res = False

    if res:
        conf.env.INCLUDES_EIGEN = includes_check
        conf.env.DEFINES_EIGEN = ['USE_EIGEN']
        conf.end_msg('ok')
    else:
        if conf.options.eigen and resibots_dir:
            msg = 'not found in %s nor in %s' % (conf.options.eigen, resibots_dir)
        elif conf.options.eigen or resibots_dir:
            msg = 'not found in %s' % (conf.options.eigen if conf.options.eigen else resibots_dir)
        else:
            msg = 'not found, use --eigen=/path/to/eigen or --resibots=/path/to/resibots'

        if required:
            conf.fatal(msg)
        else:
            conf.end_msg(msg, 'YELLOW')
