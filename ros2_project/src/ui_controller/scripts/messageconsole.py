# coding=utf-8

import os, time
from PySide6.QtWidgets import QWidget
from ui.ui_messageconsole import Ui_MessageConsole
from PySide6.QtCore import Slot
from pathlib import Path

BASE_PATH = Path(__file__).parent.resolve()

class MessageConsole(QWidget):
    def __init__(self,):
        super().__init__()
        self.ui = Ui_MessageConsole()
        self.ui.setupUi(self)

        self.ui.textBrowser.document().setMaximumBlockCount(10000)
        # self.log_dir = find_dir_path(dir_name="logs")
        # if not os.path.exists(self.log_dir):
        #     os.mkdir(self.log_dir)
        # self.logPath = os.path.join(self.log_dir, "{}.txt".format(time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime())))
        self.logPath = os.path.join(os.path.dirname(BASE_PATH), "logs")
        if not os.path.isdir(self.logPath):
            os.makedirs(self.logPath)
        self.logPath = os.path.join(self.logPath, f"{time.strftime('%Y-%m-%d', time.localtime())}.txt")

    @Slot(str)
    def showMessage(self, text):
        """"""
        self.log = open(self.logPath, 'a')
        saveTime = '[' + time.strftime('%H:%M:%S', time.localtime()) + ']: '
        self.ui.textBrowser.append(saveTime + text)
        print(saveTime + text)
        self.log.write(saveTime + text + '\n')
        self.log.close()