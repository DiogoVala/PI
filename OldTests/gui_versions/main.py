from PyQt5.QtWidgets import QApplication
from PyQt5.QtQml import QQmlApplicationEngine

import sys

if __name__ == "__main__":
    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine("main_file.qml")
    engine.quit.connect(app.quit)
    win = engine.rootObjects()[0]
    win.show()
    sys.exit(app.exec_())
