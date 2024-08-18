import sys
from PyQt5.QtWidgets import *
from PyQt5 import uic
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

ui_path = r'Calculator.ui'
form_class = uic.loadUiType(ui_path)[0]

class WindowClass(QMainWindow, form_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.le_view.setEnabled(False)
        self.text_val = ''

        self.btn_C.clicked.connect(self.btn_clicked)
        self.btn_num0.clicked.connect(self.btn_clicked)
        self.btn_num1.clicked.connect(self.btn_clicked)
        self.btn_num2.clicked.connect(self.btn_clicked)
        self.btn_num3.clicked.connect(self.btn_clicked)
        self.btn_num4.clicked.connect(self.btn_clicked)
        self.btn_num5.clicked.connect(self.btn_clicked)
        self.btn_num6.clicked.connect(self.btn_clicked)
        self.btn_num7.clicked.connect(self.btn_clicked)
        self.btn_num8.clicked.connect(self.btn_clicked)
        self.btn_num9.clicked.connect(self.btn_clicked)
        self.btn_result.clicked.connect(self.btn_clicked)
        self.btn_sub.clicked.connect(self.btn_clicked)
        self.btn_add.clicked.connect(self.btn_clicked)
        self.btn_mul.clicked.connect(self.btn_clicked)
        self.btn_div.clicked.connect(self.btn_clicked)

    def btn_clicked(self):
        btn_val = self.sender().text()
        if btn_val == 'C':
            print('clear')
            self.le_view.setText('0')
            self.text_val=''
        elif btn_val == '=':
            print('=')
            try:
                resultVal = eval(self.text_val.lstrip("0"))
                self.le_view.setText(str(resultVal))
            except:
                self.le_view.setText('error')
            self.text_val = ''
        else:
            if btn_val == 'X':
                btn_val = '*'
            self.text_val = self.text_val + btn_val
            print(self.text_val)
            self.le_view.setText(self.text_val)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.exec_()