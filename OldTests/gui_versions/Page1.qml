import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

Page {
    Column {
    spacing: 2
        Button {
            text: "Parâmetros"
            width: 80
            height: 72
            font.pixelSize : 12

            onClicked: {
                mystackview.push(page2);
            }
        }
        Button {
            text: "Gráficos"
            width: 80
            height: 72
            font.pixelSize : 12

            onClicked: {
                mystackview.push(page3);
            }
        }
        Button {
            text: "Informações"
            width: 80
            height: 72
            font.pixelSize : 12

            onClicked: {
                mystackview.push(page4);
            }
        }
    }
    Label {
        text: "Termorregulador"
        font.pixelSize : 20
        x: 160
        y: 90
    }
}
