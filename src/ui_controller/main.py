#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.Qt import *
from scripts.mainwindow import GUI

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec())