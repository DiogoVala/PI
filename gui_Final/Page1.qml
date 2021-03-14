import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

Page {
    Column {
    spacing: 2
        Button {
            text: "Def. Param"
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page2);
            }
        }
        Button {
            text: "Graf."
            width: 80
            height: 72

            onClicked: {
                mystackview.push(page3);
            }
        }
        Button {
            text: "Info."
            width: 80
            height: 72

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
