# Copyright (c) 2018 Zilogic Systems.
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing LPC17XX devices with lpc21isp tool.'''

from os import path

from .. import log
from .core import ZephyrBinaryRunner, RunnerCaps


class LPC21ISPBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for lpc21isp.'''

    def __init__(self, cfg, port, baud='115200', osc_freq='12000',
                 flashtool='lpc21isp', control='-control'):
        super(LPC21ISPBinaryRunner, self).__init__(cfg)
        self.bin_name = cfg.kernel_bin
        self.port = port
        self.baud = baud
        self.osc_freq = osc_freq
        self.flashtool = flashtool
        self.control = control

    @classmethod
    def name(cls):
        return 'lpc21isp'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'})

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument('--flashtool', default='lpc21isp',
                            help='Specify flashtool path, default: lpc21isp')
        parser.add_argument('--port', default='/dev/ttyUSB0',
                            help='Serial port to flash, default: /dev/ttyUSB0')
        parser.add_argument('--baud-rate', default='115200',
                            help='Serial baud rate, default: 115200')
        parser.add_argument('--osc-freq', default='12000',
                            help='Flash frequency, default: 12000')
        parser.add_argument('--control', default='-control',
                            help='Control signal, default: -control')

    @classmethod
    def create(cls, cfg, args):
        return LPC21ISPBinaryRunner(
            cfg, args.port, baud=args.baud_rate,
            osc_freq=args.osc_freq, flashtool=args.flashtool,
            control=args.control)

    def do_run(self, command, **kwargs):
        cmd_flash = [self.flashtool, self.control, '-bin', self.bin_name, self.port,
                     self.baud, self.osc_freq]

        log.inf("Flashing LPC on {} ({}bps)".format(self.port, self.baud))
        self.check_call(cmd_flash)
