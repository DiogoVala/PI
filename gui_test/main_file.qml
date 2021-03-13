import QtQuick.Controls 2.3
import QtQuick 2.11
import QtCharts 2.15

ApplicationWindow {
    id: mainWindow
    title: "My Application"
    width: 400
    height: 400

    Row {
    spacing: 10
        Rectangle {
            id: rect
            property bool isRed: true
            width: 50
            height: 50
            color: isRed ? "red" : "green"
            Text {
                anchors.centerIn: parent
                text: "Pre-heat?"
                color: "black"
            }

            MouseArea {
                anchors.fill: parent
                onClicked: {
                    rect.isRed = !rect.isRed
                }
            }
             
        }

        ComboBox {
            model: ["First", "Second", "Third"]
        }

        Tumbler {
            model: 5
        }

        Slider {
            from: 1
            value: 25
            to: 100
            orientation: Qt.Vertical
        }
    }
}
 