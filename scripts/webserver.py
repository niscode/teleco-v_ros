#!/usr/bin/env python
# -*- coding: utf-8 -*-

# webサーバ起動用に作成

import rospy, os
import SimpleHTTPServer

def kill():
    os.system("kill -KILL " + str(os.getpid()))

# ~/teleco-v_src/contents内のディレクトリを参照
os.chdir(os.path.dirname(__file__) + "/../contents")
rospy.init_node("webserver")
rospy.on_shutdown(kill)
SimpleHTTPServer.test()