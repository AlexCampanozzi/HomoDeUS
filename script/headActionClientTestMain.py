#! /usr/bin/env python
import sys

sys.path.append(".")
from headActionClient import HeadActionClient

if __name__ == '__main__':

    client = HeadActionClient()
    client.gotoAngle(-30, 0)