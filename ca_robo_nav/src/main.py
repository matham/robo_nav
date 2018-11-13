#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.dirname(__file__))

from robopy.bot import Bot

if __name__ == '__main__':
	Bot().run()
