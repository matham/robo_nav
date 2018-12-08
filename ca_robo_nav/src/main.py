#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.dirname(__file__))

from robopy.bot import Bot
from robopy.people import PersonControl

if __name__ == '__main__':
	PersonControl().run()
